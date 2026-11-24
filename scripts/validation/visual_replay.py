"""Benchmark an installed detector on persisted Gazebo RGB images from RTAB-Map."""
import hashlib
import json
import resource
import sqlite3
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from adapters.perception.detector import ObjectDetector

db, output = map(Path, sys.argv[1:3])
with sqlite3.connect(db.resolve().as_uri() + "?mode=ro", uri=True) as conn:
    frames = [(i, cv2.imdecode(np.frombuffer(blob, dtype=np.uint8), cv2.IMREAD_COLOR))
              for i, blob in conn.execute("select id,image from Data where image is not null order by id")]
if not frames or any(frame is None for _, frame in frames):
    raise ValueError("Expected encoded RGB frames")
initial = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
detector = ObjectDetector()
if detector.ort_session is None:
    raise RuntimeError("Expected a real ONNX session")
latencies = []
detections = {}
detector.detect(frames[0][1])
for _ in range(3):
    for index, frame in frames:
        t = time.perf_counter()
        objects = detector.detect(frame)
        latencies.append((time.perf_counter()-t)*1000)
        detections[index] = [(name, round(score, 5), box.as_tuple()) for name, score, box in objects]
with output.open("x") as f:
    json.dump(dict(frames=len(frames), samples=len(latencies), providers=detector.ort_session.get_providers(),
                   latency_p50_ms=np.median(latencies), latency_p95_ms=np.percentile(latencies,95),
                   peak_rss_delta_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss-initial,
                   dataset_sha256=hashlib.sha256(db.read_bytes()).hexdigest(), detections=detections),f,indent=2)
