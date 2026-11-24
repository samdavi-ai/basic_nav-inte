"""Measure observed wall-clock throughput between the first and last resource samples."""
import json
from pathlib import Path
import re
import sys

import numpy as np


def summarize(path):
    rows = [json.loads(line) for line in path.open()]
    ticks = [i for i, row in enumerate(rows) if row["kind"] == "resources"]
    if len(ticks) < 2:
        raise ValueError("At least two wall-clock resource samples required")
    seconds = rows[ticks[-1]]["t"] - rows[ticks[0]]["t"]
    window = rows[ticks[0]:ticks[-1]]
    by_kind = {kind: [row for row in window if row["kind"] == kind]
               for kind in ("rgb", "info", "odom", "graph")}
    poses = {row["t"] for row in by_kind["odom"]}
    rgb = {row["t"] for row in by_kind["rgb"]}
    valid = {row["t"] for row in by_kind["info"] if not row["lost"]} & poses & rgb
    processes = {}
    for i in ticks:
        for sample in rows[i]["processes"]:
            processes.setdefault(sample["name"], []).append(sample)
    # Disjoint wall-clock windows expose stalls which a whole-run average hides.
    windows = []
    start = ticks[0]
    for end in ticks[1:]:
        elapsed = rows[end]["t"] - rows[start]["t"]
        if elapsed < 10:
            continue
        chunk = rows[start:end]
        stamps = {r["t"] for r in chunk if r["kind"] == "rgb"}
        windows.append({"wall_seconds": elapsed, "valid_pose_hz": len(stamps & valid)/elapsed,
                        "graph_hz": sum(r["kind"] == "graph" for r in chunk)/elapsed})
        start = end
    latency = {}
    backend_log = path.parent / "1_gracemo_rgbd_slam.log"
    if backend_log.is_file():
        log = backend_log.read_text(errors="replace")
        for name, pattern in (("odometry", r"Odom: quality=.*? delay=([\d.]+)s"),
                              ("mapping", r"Maps update=.*? delay=([\d.]+)s")):
            values = [float(value) for value in re.findall(pattern, log)]
            if values:
                latency[name] = dict(zip(("median", "p95", "max"), np.percentile(values, (50, 95, 100)).tolist()))
    return {
        "wall_seconds": seconds,
        "observer_messages_per_wall_second": {k: len(v)/seconds for k, v in by_kind.items()},
        "matched_nonlost_poses_per_wall_second": len(valid)/seconds,
        "complete_wall_windows": windows,
        "backend_reported_stamp_delay_s": latency,
        "processes": {name: {
            "samples": len(samples),
            "median_cpu_percent_one_core": float(np.median([s["cpu"] for s in samples])),
            "peak_rss_mib": max(s["rss"] for s in samples)/2**20,
            "first_rss_mib": samples[0]["rss"]/2**20,
            "last_rss_mib": samples[-1]["rss"]/2**20,
        } for name, samples in processes.items()},
    }


if __name__ == "__main__":
    print(json.dumps(summarize(Path(sys.argv[1])), indent=2))
