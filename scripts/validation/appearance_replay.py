"""Local image replay. Numbered boxes in one photograph provide controlled correspondences."""
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from adapters.perception.detector import ObjectDetector
from adapters.perception.person_tracks import PersonTracks, appearance

frame = cv2.imread(sys.argv[1])
people = [r for r in ObjectDetector().detect(frame) if r[0]=="person"]
if len(people)<2:
    raise RuntimeError("At least two detected people are required")
people.sort(key=lambda row:row[2].cx)
tracker = PersonTracks()
start = time.perf_counter()
first = tracker.update(people,frame,0.,True)
short = tracker.update(people[::-1],frame,.2,True)
tracker.update([],frame,.4,True)
resumed = tracker.update(people,frame,.6,True)
# Build gallery only from the enrollment frame. Probe photometric changes
# and modest bbox perturbations of the same boxes, with no face embeddings.
def crop(box,variant):
    x,y,x2,y2=map(int,box.as_tuple())
    dx,dy=int((x2-x)*variant*.02),int((y2-y)*variant*.02)
    return cv2.convertScaleAbs(frame[y+dy:y2-dy,x+dx:x2-dx],alpha=1-.04*variant,beta=variant*2)

refs = [appearance(crop(p[2],0)) for p in people]
tp=fp=fn=tn=0
for variant in (1,2,3):
    for i,(_,_,box) in enumerate(people):
        signature = appearance(crop(box,variant))
        scores = np.array([float(signature@r) for r in refs])
        order = np.argsort(scores)[::-1]
        chosen = order[0] if scores[order[0]]>=tracker.threshold and scores[order[0]]-scores[order[1]]>=tracker.margin else None
        for j in range(len(refs)):
            positive = j==i
            predicted = chosen==j
            tp+=int(positive and predicted)
            fn+=int(positive and not predicted)
            fp+=int(not positive and predicted)
            tn+=int(not positive and not predicted)
print(json.dumps(dict(image=Path(sys.argv[1]).name,detected_people=len(first),unique_tracks=len(set(p["track_id"] for p in first)),
    reversed_order_consistent=[p["track_id"] for p in first] == [p["track_id"] for p in reversed(short)],
    short_occlusion_consistent=[p["track_id"] for p in first] == [p["track_id"] for p in resumed],
    tp=tp,fp=fp,fn=fn,tn=tn,precision=tp/(tp+fp) if tp+fp else None,recall=tp/(tp+fn),
    false_match_rate=fp/(fp+tn),elapsed_ms=(time.perf_counter()-start)*1000),indent=2))
