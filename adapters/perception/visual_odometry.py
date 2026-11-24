"""Bounded local RGB-D feature map. The initial optical frame is its origin."""
from collections import deque

import hashlib
import os
from pathlib import Path
import tempfile
from zipfile import BadZipFile, ZipFile

import cv2
import numpy as np


class VisualOdometry:
    def __init__(self, features=500, keyframes=12):
        self.orb = cv2.ORB_create(nfeatures=features,edgeThreshold=12,fastThreshold=10)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
        self.map = deque(maxlen=keyframes)
        self.previous = None
        self.trajectory = deque(maxlen=128)
        self.lost = False
        self.frame = 0
        self.calibration = None
        self.map_error = None
        self.sensor_shapes = None
        self.dense = None
        self.dense_quality = {}
        self.last_stamp = None
        self.loss_since = None
        self.last_rgbd = None
        self.relocalization_pending = False
        self.database = os.getenv("VIRA_VISUAL_MAP")

    @staticmethod
    def _map_digest(data):
        digest = hashlib.sha256()
        for name in sorted(key for key in data if key != "map_sha256"):
            value = np.asarray(data[name])
            if value.dtype.hasobject:
                raise ValueError("Object arrays cannot be persisted in a visual map")
            digest.update(name.encode("utf-8"))
            digest.update(value.dtype.str.encode("ascii"))
            digest.update(np.asarray(value.shape,dtype="<i8").tobytes())
            digest.update(np.ascontiguousarray(value).tobytes())
        return digest.hexdigest()

    def save(self):
        if not self.database or not self.map or self.calibration is None:
            return
        path = Path(self.database)
        if path.exists():
            raise FileExistsError("Choose a new visual map output path")
        with tempfile.NamedTemporaryFile(dir=path.parent) as tmp:
            data = {"version": 3, "intrinsics": self.calibration, "count": len(self.map),
                    "sensor_shapes": self.sensor_shapes,
                    "calibration_hash": self._fingerprint(self.calibration, self.sensor_shapes)}
            for i,(pose,points,descriptors) in enumerate(self.map):
                data.update({f"pose{i}":pose,f"points{i}":points,f"features{i}":descriptors})
            data["map_sha256"] = np.asarray(self._map_digest(data))
            np.savez_compressed(tmp,**data)
            tmp.flush()
            os.fsync(tmp.fileno())
            os.link(tmp.name,path)
        directory = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(directory)
        finally:
            os.close(directory)

    @staticmethod
    def _fingerprint(intrinsics, shapes):
        digest = hashlib.sha256(np.asarray(intrinsics, dtype="<f8").tobytes())
        digest.update(np.asarray(shapes, dtype="<i8").tobytes())
        return digest.hexdigest()

    def _load(self, intrinsics):
        source = os.getenv("VIRA_VISUAL_MAP_INPUT")
        if not source:
            return
        # Bound decompression before loading arrays, including legacy maps.
        with ZipFile(source) as archive:
            members = archive.infolist()
            if len(members) > 3 * self.map.maxlen + 6 or sum(m.file_size for m in members) > 2_000_000:
                raise ValueError("Saved map exceeds size limits")
        loaded = []
        with np.load(source, allow_pickle=False) as saved:
            version = int(saved["version"].item()) if "version" in saved else 1
            if version not in (1, 2, 3):
                raise ValueError("Unsupported map version")
            if version == 3:
                values = {key: saved[key] for key in saved.files if key != "map_sha256"}
                if saved["map_sha256"].item() != self._map_digest(values):
                    raise ValueError("Saved map integrity check failed")
            if version in (2, 3) and (not np.array_equal(saved["sensor_shapes"], self.sensor_shapes) or
                                 saved["calibration_hash"].item() != self._fingerprint(intrinsics, self.sensor_shapes)):
                raise ValueError("Saved map sensor geometry differs")
            if saved["intrinsics"].shape != (3,3) or not np.allclose(saved["intrinsics"],intrinsics):
                raise ValueError("Saved camera calibration differs")
            count = int(saved["count"])
            if not 1<=count<=self.map.maxlen:
                raise ValueError("Invalid saved keyframe count")
            for i in range(count):
                pose,points,features = (saved[f"{key}{i}"] for key in ("pose","points","features"))
                if (pose.shape!=(4,4) or points.ndim!=2 or points.shape[1]!=3 or not 10 <= len(points) <= self.orb.getMaxFeatures()
                        or features.shape!=(len(points),32) or features.dtype!=np.uint8
                        or not np.isfinite(pose).all() or not np.isfinite(points).all()):
                    raise ValueError("Invalid saved visual map")
                rotation = pose[:3,:3]
                if not (np.allclose(pose[3], [0,0,0,1]) and
                        np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) and
                        np.isclose(np.linalg.det(rotation), 1.0)):
                    raise ValueError("Invalid keyframe transform")
                loaded.append((pose,points,features))
        self.map.extend(loaded)
        # A persisted keyframe is only a search reference. Do not assume the
        # restarted camera is still at that keyframe's pose.
        self.previous = None
        self.last_rgbd = None
        self.lost = True
        self.relocalization_pending = True

    def _dense_motion(self, image, depth, k):
        # Work at depth sensor resolution, preserving metric depth discontinuities.
        h, w = depth.shape
        gray = cv2.resize(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), (w, h),
                          interpolation=cv2.INTER_AREA)
        metric = depth.astype(np.float32, copy=True)
        metric[(metric < .1) | (metric > 10) | ~np.isfinite(metric)] = np.nan
        previous = self.last_rgbd
        self.last_rgbd = gray, metric
        if previous is None or self.lost or previous[0].shape != gray.shape or np.isfinite(metric).sum() < 10:
            return None
        kd = np.diag([w / image.shape[1], h / image.shape[0], 1.]) @ k
        if self.dense is None:
            self.dense = cv2.rgbd.RgbdICPOdometry_create(kd.astype(np.float32))
        try:
            ok, motion = self.dense.compute(*previous, None, gray, metric, None)
        except cv2.error:
            return None
        if not ok or not np.isfinite(motion).all():
            return None
        # Check the estimated warp against current depth on a regular image grid.
        v, u = np.mgrid[2:h:4, 2:w:4]
        z = previous[1][v,u]
        finite = np.isfinite(z)
        if not finite.any():
            return None
        u, v, z = u[finite], v[finite], z[finite]
        points = np.stack(((u-kd[0,2])*z/kd[0,0], (v-kd[1,2])*z/kd[1,1], z),axis=1)
        moved = points @ motion[:3,:3].T + motion[:3,3]
        moved = moved[moved[:,2] > .1]
        projections = moved @ kd.T
        xy = np.rint(projections[:,:2] / projections[:,2,None]).astype(int)
        inside = (xy[:,0]>=0)&(xy[:,0]<w)&(xy[:,1]>=0)&(xy[:,1]<h)
        residual = np.abs(metric[xy[inside,1],xy[inside,0]]-moved[inside,2])
        residual = residual[np.isfinite(residual)]
        if not residual.size:
            return None
        self.dense_quality = dict(depth_checks=int(residual.size),
                                  depth_residual_median_m=float(np.median(residual)),
                                  depth_consistent_fraction=float(np.mean(residual<=.15)))
        # Apply the same metric consistency criterion used by the sparse estimator.
        if residual.size < 8 or self.dense_quality["depth_consistent_fraction"] < .7:
            return None
        return motion

    def update(self, image, depth, intrinsics, stamp_ns=None, people=()):
        if stamp_ns is not None and self.last_stamp is not None and stamp_ns <= self.last_stamp:
            return {"state": "stale", "pose_current": False, "reason": "nonmonotonic_capture"}
        self.last_stamp = stamp_ns
        mask = None
        if people:
            mask = np.full(image.shape[:2], 255, np.uint8)
            for person in people:
                x1,y1,x2,y2 = person["bbox"]
                cv2.rectangle(mask,(int(np.floor(x1)),int(np.floor(y1))),
                              (int(np.ceil(x2)),int(np.ceil(y2))),0,-1)
            depth = depth.copy()
            depth[cv2.resize(mask, depth.shape[::-1], interpolation=cv2.INTER_NEAREST) == 0] = np.nan
        result = self._estimate(image, depth, intrinsics, mask)
        if result.get("method") == "dense_rgbd":
            result.update(self.dense_quality)
        result["pose_current"] = result["state"] in ("tracking", "relocalized")
        if result["state"] == "lost" and stamp_ns is not None:
            if self.loss_since is None:
                self.loss_since = stamp_ns
            result["recovery_state"] = "recovering" if self.map else "awaiting_features"
            result["loss_duration_s"] = (stamp_ns - self.loss_since) / 1e9
        if result["pose_current"]:
            if self.loss_since is not None and stamp_ns is not None:
                result["recovery_latency_s"] = (stamp_ns - self.loss_since) / 1e9
            self.loss_since = None
        return result

    def _estimate(self,image,depth,intrinsics,mask):
        self.frame += 1
        k = np.asarray(intrinsics,dtype=float).reshape(3,3)
        shapes = (*image.shape[:2], *depth.shape)
        if self.calibration is None:
            self.sensor_shapes = shapes
            try:
                self._load(k)
            except (OSError, ValueError, KeyError, TypeError, BadZipFile, EOFError) as error:
                self.map_error = str(error)[:160] or type(error).__name__
            self.calibration = k.copy()
        if self.map_error:
            return {"state": "map_unavailable", "reason": self.map_error}
        if self.sensor_shapes != shapes or not np.allclose(k,self.calibration):
            return {"state":"calibration_changed"}
        dense_motion = self._dense_motion(image, depth, k) if hasattr(cv2, "rgbd") else None
        points, desc = self.orb.detectAndCompute(image,mask)
        if (desc is None or len(points)<10) and dense_motion is None:
            self.lost = True
            return {"state":"lost","features":len(points)}
        uv = np.asarray([p.pt for p in points]).reshape(-1,2)
        dh,dw = depth.shape
        h,w = image.shape[:2]
        z = depth[(uv[:,1]*dh/h).astype(int).clip(0,dh-1),(uv[:,0]*dw/w).astype(int).clip(0,dw-1)]
        depth_valid = np.isfinite(z) & (z > .1) & (z < 10)
        xyz = np.zeros((len(uv), 3), dtype=float)
        xyz[:, 2] = z
        xyz[depth_valid, 0] = ((uv[depth_valid, 0]-k[0,2])/k[0,0])*z[depth_valid]
        xyz[depth_valid, 1] = ((uv[depth_valid, 1]-k[1,2])/k[1,1])*z[depth_valid]
        valid = depth_valid & np.isfinite(xyz).all(axis=1)
        state = "initializing"
        n = 0
        pose = np.eye(4)
        if self.previous is not None and dense_motion is not None and not self.relocalization_pending:
            pose = self.previous[0] @ np.linalg.inv(dense_motion)
            state = "tracking"
        elif self.previous is not None or self.map:
            references = ([self.previous] if self.previous is not None else [])
            references.extend(ref for ref in reversed(self.map) if ref is not self.previous)
            for world,ref_xyz,ref_desc in references:
                matches = [pair[0] for pair in self.matcher.knnMatch(ref_desc,desc,k=2)
                           if len(pair)==2 and pair[0].distance < .75*pair[1].distance]
                if len(matches)<10:
                    continue
                obj = np.asarray([ref_xyz[m.queryIdx] for m in matches])
                pixels = np.asarray([uv[m.trainIdx] for m in matches])
                ok,rot,trans,inliers = cv2.solvePnPRansac(obj,pixels,k,None,
                    iterationsCount=80,reprojectionError=2.,flags=cv2.SOLVEPNP_EPNP)
                if not ok or inliers is None or len(inliers)<max(8,.5*len(matches)):
                    continue
                transform = np.eye(4)
                transform[:3,:3] = cv2.Rodrigues(rot)[0]
                transform[:3,3] = trans.ravel()
                # RANSAC image inliers alone can accept numerically degenerate
                # PnP solutions on planar walls. Independently check metric
                # correspondences against synchronized *current-frame* depth.
                inlier = inliers.ravel()
                cur = np.array([m.trainIdx for m in matches])[inlier]
                predicted = obj[inlier] @ transform[:3,:3].T + transform[:3,3]
                metric = np.linalg.norm(predicted - xyz[cur],axis=1)
                consistent = valid[cur] & (predicted[:,2] > 0) & (metric <= .15)
                if np.count_nonzero(consistent) < max(8,.7*len(cur)):
                    continue
                # Use measured depth on both sides to refine the metric translation.
                src, dst = obj[inlier][consistent], xyz[cur][consistent]
                src_mean, dst_mean = src.mean(axis=0), dst.mean(axis=0)
                u, scales, vt = np.linalg.svd((src-src_mean).T @ (dst-dst_mean))
                if scales[1] < 1e-6:
                    continue
                correction = np.eye(3)
                correction[2,2] = np.sign(np.linalg.det(vt.T @ u.T))
                rotation = vt.T @ correction @ u.T
                translation = dst_mean - rotation @ src_mean
                refined = src @ rotation.T + translation
                if np.any(refined[:,2] <= 0):
                    continue
                projected = refined @ k.T
                reprojection = projected[:,:2] / projected[:,2,None] - pixels[inlier][consistent]
                if np.median(np.linalg.norm(reprojection, axis=1)) <= 2.:
                    transform[:3,:3], transform[:3,3] = rotation, translation
                pose = world @ np.linalg.inv(transform)
                state = "relocalized" if self.lost or self.relocalization_pending else "tracking"
                self.relocalization_pending = False
                n = len(inliers)
                break
            else:
                self.lost = True
                return {"state":"lost","features":len(points)}
        if valid.sum()<10 and dense_motion is not None and self.previous is not None:
            # Continue incremental RGB-D alignment without attaching old descriptors to a new pose.
            self.previous = (pose, np.empty((0,3)), np.empty((0,32),np.uint8))
            return dict(state=state, method="dense_rgbd", features=len(points), keyframes=len(self.map),
                        frame_id="visual_origin_optical", optical_pose=pose.tolist(),
                        trajectory=list(self.trajectory))
        if self.relocalization_pending:
            self.lost = True
            return {"state": "lost", "features": len(points), "recovery_state": "recovering"}
        if valid.sum()<10:
            self.lost = True
            return {"state":"lost","features":len(points)}
        if self.frame == 1 or self.frame % 5 == 0 or state == "relocalized":
            self.trajectory.append([self.frame, state, *pose[:3,3].tolist()])
        self.lost = False
        self.previous = (pose,xyz[valid],desc[valid])
        # A stationary robot can otherwise evict the entire route in 12 keyframes.
        # Compare keyframe viewpoints in the local optical map; preserve its origin.
        if self.frame % 5 == 0 or not self.map:
            covered = False
            for saved_pose, _, _ in self.map:
                rotation_delta = saved_pose[:3,:3].T @ pose[:3,:3]
                angle = np.arccos(np.clip((np.trace(rotation_delta)-1)/2, -1., 1.))
                if np.linalg.norm(saved_pose[:3,3]-pose[:3,3]) < .15 and angle < np.deg2rad(10):
                    covered = True
                    break
            if not covered and len(self.map) == self.map.maxlen and len(self.map) > 1:
                del self.map[1]
            if not covered:
                self.map.append(self.previous)
        return dict(state=state,method="dense_rgbd" if dense_motion is not None else "sparse_rgbd",features=len(points),inliers=n,keyframes=len(self.map),
                    frame_id="visual_origin_optical",optical_pose=pose.tolist(),
                    trajectory=list(self.trajectory))
