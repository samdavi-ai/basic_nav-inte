"""Score visible simulated sweater identities independently of detector and tracker output."""
import json
import sys
from collections import Counter
from pathlib import Path

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
from adapters.perception.person_tracks import appearance


def labels(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    marked = {}
    for actor, low, high in (("magenta",140,165), ("cyan",80,100)):
        mask = cv2.inRange(hsv,(low,130,35),(high,255,255))
        y, x = np.nonzero(mask)
        if len(x) >= 20:
            marked[actor] = x, y
    return marked


def evaluate(folder):
    trial = json.loads((folder/"actors.json").read_text())
    counts = Counter()
    paired = Counter()
    previous = {}
    associations = []
    descriptors = []
    similarities = {"genuine": [], "impostor": []}
    for row in trial["samples"]:
        ns = row["stamp_ns"]
        path = folder/"actor_frames"/f"{ns}.png"
        if not path.exists():
            counts["missing_images"] += 1
            continue
        gt = labels(cv2.imread(str(path)))
        predicted = row["people"]
        names = list(gt)
        counts.update(frames=1, visible_sweaters=len(names), predictions=len(predicted))
        score = np.zeros((len(names), len(predicted)))
        for i, (x,y) in enumerate(gt.values()):
            for j, person in enumerate(predicted):
                l,t,r,b = person["bbox"]
                score[i,j] = np.mean((l<=x)&(x<=r)&(t<=y)&(y<=b))
        # Maximize valid pairs first, then fraction of the visible garment covered.
        rows, cols = linear_sum_assignment(-((score >= .6)*3 + score))
        matched = {}
        for i,j in zip(rows,cols):
            if score[i,j] < .6:
                continue
            actor, track = names[i],predicted[j]["track_id"]
            l,t,r,b = map(int,predicted[j]["bbox"])
            frame = cv2.imread(str(path))
            vector = appearance(frame[max(0,t):max(0,b),max(0,l):max(0,r)])
            if vector is not None:
                for earlier_ns, label, signature in descriptors:
                    if ns - earlier_ns >= 2_000_000_000:
                        group = "genuine" if label == actor else "impostor"
                        similarities[group].append(float(vector @ signature))
                descriptors.append((ns, actor, vector))
            matched[actor] = track
            paired[actor,track] += 1
            counts["matched"] += 1
            if actor in previous and previous[actor] != track:
                counts["id_switches_including_reentry"] += 1
            previous[actor] = track
        associations.append(dict(stamp_ns=ns,visible=names,matched=matched))
    identities = sorted({a for a,t in paired})
    tracks = sorted({t for a,t in paired})
    matrix = np.array([[paired[a,t] for t in tracks] for a in identities],dtype=int).reshape(len(identities),len(tracks))
    idx = linear_sum_assignment(-matrix)
    def divide(n,d):
        return n/d if d else None
    return dict(method="Visible garment pixel containment; each capture gets a one-to-one assignment",
                counts=dict(counts),precision=divide(counts["matched"],counts["predictions"]),
                recall=divide(counts["matched"],counts["visible_sweaters"]),
                idf1=divide(2*matrix[idx].sum().item(),counts["predictions"]+counts["visible_sweaters"]),
                appearance_pairs={key:dict(count=len(values),at_or_above_threshold=sum(x>=.94 for x in values))
                                  for key,values in similarities.items()},
                appearance_threshold=.94, associations=associations)


if __name__ == "__main__":
    print(json.dumps(evaluate(Path(sys.argv[1])),indent=2))
