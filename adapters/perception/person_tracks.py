"""Short-lived camera-local tracks. Appearance candidates never assign a human name."""
from dataclasses import dataclass
from uuid import uuid4

import cv2
import numpy as np


@dataclass
class Track:
    id: str
    box: np.ndarray
    velocity: np.ndarray
    stamp: float
    hits: int = 1
    signature: object = None
    appearance_id: str | None = None
    appearance_state: str = "unknown"


def overlap(a, b):
    area = np.maximum(0, np.minimum(a[2:], b[2:]) - np.maximum(a[:2], b[:2])).prod()
    return area / max(1., (a[2:] - a[:2]).prod() + (b[2:] - b[:2]).prod() - area)


def appearance(crop):
    if crop.size == 0 or min(crop.shape[:2]) < 12:
        return None
    hsv = cv2.cvtColor(cv2.resize(crop, (24, 48)), cv2.COLOR_BGR2HSV)
    parts = [cv2.calcHist([region], [0, 1], None, [12, 4], [0, 180, 0, 256]).ravel()
             for region in np.array_split(hsv, 2)]
    vector = np.concatenate(parts)
    return vector / max(1., np.linalg.norm(vector))


class PersonTracks:
    def __init__(self, capacity=32, ttl=1., retention=10., threshold=.94, margin=.08):
        self.tracks = {}
        self.gallery = {}
        self.capacity, self.ttl, self.retention = capacity, ttl, retention
        self.threshold, self.margin = threshold, margin
        self.last_stamp = None

    def update(self, people, frame, stamp, reid=False):
        if self.last_stamp is not None and stamp <= self.last_stamp:
            # Late sensor packets must not erase associations for a newer capture.
            return []
        self.last_stamp = stamp
        self.gallery = {key: value for key, value in self.gallery.items() if stamp-value[1] <= self.retention}
        for key, track in list(self.tracks.items()):
            if stamp-track.stamp > self.ttl:
                if reid and stamp-track.stamp <= self.retention and track.signature is not None and track.appearance_id:
                    self.gallery[track.appearance_id] = (track.signature, track.stamp)
                del self.tracks[key]
        while len(self.gallery) > self.capacity:
            del self.gallery[min(self.gallery, key=lambda k: self.gallery[k][1])]
        boxes = [np.array(box.as_tuple(), dtype=float) for _, _, box in people[:self.capacity]]
        pairs = []
        for k, track in self.tracks.items():
            predicted = track.box + track.velocity * min(.5, stamp-track.stamp)
            for i, box in enumerate(boxes):
                score = overlap(predicted, box)
                if score >= .25:
                    pairs.append((score, k, i))
        assigned, used = {}, set()
        association = {}
        for score, key, i in sorted(pairs, reverse=True):
            if key not in used and i not in assigned:
                assigned[i] = key
                association[i] = score
                used.add(key)
        observations = []
        for i, box in enumerate(boxes):
            state = "unknown"
            if i in assigned:
                track = self.tracks[assigned[i]]
                track.hits += 1
                track.velocity = .5*track.velocity + .5*(box-track.box)/max(.001, stamp-track.stamp)
                track.box, track.stamp = box, stamp
                state = track.appearance_state
            else:
                if len(self.tracks) >= self.capacity:
                    continue
                track = Track(uuid4().hex, box, np.zeros(4), stamp)
                self.tracks[track.id] = track
                if reid:
                    x1,y1,x2,y2 = box.astype(int)
                    track.signature = appearance(frame[max(0,y1):y2, max(0,x1):x2])
                    if track.signature is not None:
                        ranks = sorted(((float(track.signature @ vec), k) for k,(vec,_) in self.gallery.items()),reverse=True)
                        if ranks and ranks[0][0] >= self.threshold:
                            if len(ranks)==1 or ranks[0][0]-ranks[1][0] >= self.margin:
                                track.appearance_id = ranks[0][1]
                                track.appearance_state = "appearance_candidate"
                                del self.gallery[track.appearance_id]
                                state = track.appearance_state
                            else:
                                state = "ambiguous"
                        else:
                            track.appearance_id = uuid4().hex
            observations.append(dict(detection_id=uuid4().hex, track_id=track.id,
                reid_identity_id=track.appearance_id, reid_state=state, enrolled_face_identity_id=None,
                stamp_s=stamp, bbox=box.tolist(), confidence=float(people[i][1]),
                track_observations=track.hits, predicted_box_iou=association.get(i),
                alternative_iou=max((s for s,k,j in pairs if j == i and k != track.id), default=None)))
        return observations
