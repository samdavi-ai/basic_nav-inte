"""Offline deterministic tests; no real biometric images, hardware or robot motion."""
import cv2
import numpy as np
from adapters.perception.models import BoundingBox
from adapters.perception.person_tracks import PersonTracks, appearance
from adapters.perception.face_gallery import Gallery
from adapters.perception.visual_odometry import VisualOdometry
from adapters.perception.perception_service import PerceptionService


def test_two_people_and_short_occlusion():
    image = np.zeros((80,100,3),np.uint8)
    rows = [("person",.8,BoundingBox(x,10,x+20,60)) for x in (5,60)]
    tracker = PersonTracks()
    first = tracker.update(rows,image,1)
    second = tracker.update(rows[::-1],image,1.1)
    assert [p["track_id"] for p in first] == [p["track_id"] for p in reversed(second)]
    tracker.update([],image,1.2)
    assert tracker.update(rows,image,1.4)[0]["track_id"] == first[0]["track_id"]


def test_expired_tracks_and_appearance_ambiguity():
    image = np.full((80,100,3),(10,100,240),np.uint8)
    rows = [("person",.9,BoundingBox(x,10,x+20,60)) for x in (5,60)]
    tracker = PersonTracks()
    first = tracker.update(rows,image,1,reid=True)
    again = tracker.update(rows[:1],image,3,reid=True)[0]
    assert again["track_id"] != first[0]["track_id"]
    assert again["reid_state"] == "ambiguous" and again["reid_identity_id"] is None


def test_appearance_candidate_state_persists_for_active_track():
    image = np.full((80, 100, 3), (10, 100, 240), np.uint8)
    box = BoundingBox(5, 10, 35, 65)
    descriptor = appearance(image[10:65, 5:35])
    tracker = PersonTracks()
    tracker.gallery["prior-track"] = (descriptor, 1.0)

    first = tracker.update([("person", .9, box)], image, 2.0, reid=True)[0]
    second = tracker.update([("person", .9, box)], image, 2.2, reid=True)[0]

    assert first["reid_state"] == "appearance_candidate"
    assert second["reid_state"] == "appearance_candidate"
    assert second["reid_identity_id"] == "prior-track"
    assert first["track_id"] == second["track_id"]


def test_consent_delete_and_ambiguity(tmp_path):
    gallery = Gallery(tmp_path/"private")
    import pytest
    a = np.eye(3)[0]
    with pytest.raises(ValueError):
        gallery.enroll("a",a)
    gallery.enroll("a",a,consent=True)
    assert gallery.match(a,.8,.1)==("matched","a")
    assert gallery.match(np.eye(3)[1],.8,.1)==("unknown",None)
    gallery.enroll("b",a,consent=True)
    assert gallery.match(a,.8,.1)==("ambiguous",None)
    assert gallery.delete("a") and gallery.delete("b")
    assert gallery.match(a,.8,.1)==("unknown",None)


def test_map_roundtrip_loss_and_relocalization(tmp_path,monkeypatch):
    path = tmp_path/"map.npz"
    monkeypatch.setenv("VIRA_VISUAL_MAP",str(path))
    img = np.random.default_rng(18).integers(0,255,(160,240,3),np.uint8)
    depth = np.full(img.shape[:2],2.,np.float32)
    intrinsics = [210.,0,120.,0,210.,80.,0,0,1.]
    tracker = VisualOdometry()
    assert tracker.update(img,depth,intrinsics)["state"]=="initializing"
    tracker.save()
    monkeypatch.setenv("VIRA_VISUAL_MAP_INPUT",str(path))
    restored = VisualOdometry()
    assert restored.update(img*0,depth,intrinsics)["state"]=="lost"
    result = restored.update(img,depth,intrinsics)
    assert result["state"]=="relocalized"
    np.testing.assert_allclose(result["optical_pose"],np.eye(4),atol=.001)


def test_corrupted_map_does_not_break_perception(tmp_path, monkeypatch):
    path = tmp_path / "broken.npz"
    path.write_bytes(b"truncated")
    monkeypatch.setenv("VIRA_VISUAL_MAP_INPUT", str(path))
    odometry = VisualOdometry()
    for _ in range(2):
        status = odometry.update(np.zeros((32,32,3), np.uint8), np.ones((32,32)), np.eye(3))
        assert status["state"] == "map_unavailable"
        assert "optical_pose" not in status
        assert not odometry.map


def test_external_gallery_key_and_restart_deletion(tmp_path, monkeypatch):
    from cryptography.fernet import Fernet
    import pytest

    key = tmp_path / "external-key"
    key.write_bytes(Fernet.generate_key())
    key.chmod(0o600)
    monkeypatch.setenv("VIRA_FACE_KEY_FILE", str(key))
    directory = tmp_path / "gallery"
    vector = np.array([0.6, 0.8])
    Gallery(directory).enroll("consented", vector, consent=True)
    assert not (directory / "key").exists()
    assert Gallery(directory).match(vector, .9, .1) == ("matched", "consented")
    key.chmod(0o644)
    with pytest.raises(PermissionError):
        Gallery(directory).match(vector, .9, .1)
    key.chmod(0o600)
    assert Gallery(directory).delete("consented")
    assert Gallery(directory).match(vector, .9, .1) == ("unknown", None)


def test_capture_gap_cannot_resurrect_stale_appearance():
    image = np.full((80,100,3),(10,100,240),np.uint8)
    rows = [("person",.9,BoundingBox(5,10,30,65))]
    tracker = PersonTracks()
    first = tracker.update(rows,image,1,reid=True)[0]
    assert not tracker.update(rows,image,1,reid=True)
    late = tracker.update(rows,image,30,reid=True)[0]
    assert late["reid_identity_id"] != first["reid_identity_id"]
    assert late["track_id"] != first["track_id"]


def test_reload_rejects_changed_depth_resolution(tmp_path, monkeypatch):
    path = tmp_path / "calibrated.npz"
    monkeypatch.setenv("VIRA_VISUAL_MAP", str(path))
    frame = np.random.default_rng(10).integers(0,255,(160,240,3),np.uint8)
    depth = np.full((80,120), 2.0, np.float32)
    k = [210.,0,120.,0,210.,80.,0,0,1.]
    original = VisualOdometry()
    assert original.update(frame, depth, k)["state"] == "initializing"
    original.save()
    monkeypatch.setenv("VIRA_VISUAL_MAP_INPUT", str(path))
    result = VisualOdometry().update(frame, depth[::2,::2], k)
    assert result == {"state":"map_unavailable", "reason":"Saved map sensor geometry differs", "pose_current":False}


def test_nonmonotonic_odometry_input_has_no_current_pose():
    frame = np.random.default_rng(10).integers(0,255,(160,240,3),np.uint8)
    depth = np.full((80,120), 2.,np.float32)
    k = [210.,0,120.,0,210.,80.,0,0,1.]
    vo = VisualOdometry()
    vo.update(frame, depth, k, 200)
    for timestamp in (200, 150):
        stale = vo.update(frame, depth, k, timestamp)
        assert stale["state"] == "stale" and not stale["pose_current"]
        assert "optical_pose" not in stale and vo.frame == 1


def test_moving_foreground_cannot_anchor_camera_pose():
    frame = np.random.default_rng(10).integers(0,255,(160,240,3),np.uint8)
    depth = np.full((80,120),2.,np.float32)
    k = [210.,0,120.,0,210.,80.,0,0,1.]
    vo = VisualOdometry()
    vo.update(frame, depth, k, 100)
    lost = vo.update(frame, depth, k, 200, [{"bbox":[0,0,240,160]}])
    assert not lost["pose_current"] and lost["state"] == "lost"
    assert "optical_pose" not in lost and np.isfinite(depth).all()


def test_invalid_depth_features_are_rejected_without_numeric_warnings():
    import warnings

    frame = np.random.default_rng(21).integers(0, 255, (160, 240, 3), np.uint8)
    depth = np.full((80, 120), 2.0, np.float32)
    depth[::3, ::3] = np.nan
    k = [210., 0, 120., 0, 210., 80., 0, 0, 1.]
    with warnings.catch_warnings():
        warnings.simplefilter("error", RuntimeWarning)
        result = VisualOdometry().update(frame, depth, k)
    assert result["state"] == "initializing"


def test_out_of_order_people_do_not_fragment_live_track():
    image = np.zeros((80,100,3),np.uint8)
    rows = [("person",.8,BoundingBox(5,10,25,60))]
    tracker = PersonTracks()
    identity = tracker.update(rows,image,2)[0]["track_id"]
    assert tracker.update(rows,image,1.9) == []
    assert tracker.update(rows,image,2.1)[0]["track_id"] == identity


def test_face_pipeline_reports_invalid_roi_without_aborting_frame():
    service = PerceptionService.__new__(PerceptionService)
    service.degraded = False
    service.face_enabled = True
    service.face_status = "awaiting_person"
    service.face_samples = {}
    service.face_engine = None
    service.tracker = type("Tracker", (), {"tracks": {"track-1": object()}})()
    service.people = [{"track_id": "track-1", "bbox": [-20, -20, -1, -1]}]

    service._update_faces(np.zeros((32, 32, 3), np.uint8), 10.0)

    assert service.people[0]["face_state"] == "FACE_ROI_INVALID"
    assert service.people[0]["face_count"] is None
    assert service.people[0]["face_stamp_s"] == 10.0
    assert service.face_status == "FACE_ROI_INVALID"


def test_face_result_remains_attached_to_live_track():
    service = PerceptionService.__new__(PerceptionService)
    service.degraded = False
    service.face_enabled = True
    service.face_status = "awaiting_person"
    service.face_samples = {}
    service.face_engine = type("FaceEngine", (), {
        "detector": object(), "failed": False, "cascade": None,
        "detect_faces": lambda self, crop: [object()],
    })()
    service.tracker = type("Tracker", (), {"tracks": {"track-1": object()}})()
    service.people = [{"track_id": "track-1", "bbox": [2, 2, 28, 28]}]
    frame = np.zeros((32, 32, 3), np.uint8)

    service._update_faces(frame, 10.0)
    service.people = [{"track_id": "track-1", "bbox": [2, 2, 28, 28]}]
    service._update_faces(frame, 10.2)

    assert service.people[0]["face_state"] == "FACE_DETECTED"
    assert service.people[0]["face_count"] == 1
    assert service.face_status == "FACE_DETECTED"


def test_saved_map_requires_real_relocalization_and_valid_digest(tmp_path, monkeypatch):
    path = tmp_path / "versioned.npz"
    monkeypatch.setenv("VIRA_VISUAL_MAP", str(path))
    frame = np.random.default_rng(18).integers(0,255,(160,240,3),np.uint8)
    depth = np.full(frame.shape[:2],2.,np.float32)
    k = [210.,0,120.,0,210.,80.,0,0,1.]
    first = VisualOdometry()
    first.update(frame,depth,k,100)
    first.save()
    with np.load(path,allow_pickle=False) as archive:
        data = {name:archive[name] for name in archive.files}
    assert data["version"].item() == 3
    data["points0"] = data["points0"] + 2.0
    np.savez_compressed(path,**data)
    monkeypatch.setenv("VIRA_VISUAL_MAP_INPUT",str(path))
    corrupt = VisualOdometry().update(frame,depth,k,200)
    assert corrupt["state"] == "map_unavailable"
    assert "integrity" in corrupt["reason"]
    assert not corrupt["pose_current"]


def test_unmatched_saved_map_never_returns_identity_as_current(tmp_path,monkeypatch):
    path = tmp_path/"map.npz"
    monkeypatch.setenv("VIRA_VISUAL_MAP",str(path))
    first = np.random.default_rng(18).integers(0,255,(160,240,3),np.uint8)
    depth = np.full(first.shape[:2],2.,np.float32)
    k = [210.,0,120.,0,210.,80.,0,0,1.]
    tracker = VisualOdometry()
    tracker.update(first,depth,k)
    tracker.save()
    monkeypatch.setenv("VIRA_VISUAL_MAP_INPUT",str(path))
    different = np.random.default_rng(52).integers(0,255,first.shape,np.uint8)
    restored = VisualOdometry()
    result = restored.update(different,depth,k,100)
    assert result["state"] == "lost"
    assert result["pose_current"] is False
    assert "optical_pose" not in result
    assert len(restored.map) == 1
