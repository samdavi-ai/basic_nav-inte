"""Summarize a completed RTAB-Map run without writing to its map."""
import hashlib
import json
from pathlib import Path
import re
import sqlite3
import struct
import sys
import zlib


def summarize(folder):
    lifecycle, = json.loads((folder / "process_lifecycle_evidence.json").read_text())
    if not lifecycle.get("kernel_exited"):
        raise ValueError("Incomplete launcher shutdown")
    path = (folder / "map.db").resolve(strict=True)
    with sqlite3.connect(path.as_uri() + "?mode=ro", uri=True) as db:
        integrity, = db.execute("PRAGMA integrity_check").fetchone()
        nodes, truth = db.execute("SELECT count(*),count(CASE WHEN ground_truth_pose != zeroblob(length(ground_truth_pose)) THEN 1 END) FROM Node").fetchone()
        links = dict(db.execute("SELECT type,count(*) FROM Link GROUP BY type"))
        missing_words, = db.execute("""SELECT count(DISTINCT f.word_id) FROM Feature f
            LEFT JOIN Word w ON f.word_id=w.id WHERE f.word_id>0 AND w.id IS NULL""").fetchone()
        version, = db.execute("SELECT version FROM Admin").fetchone()
        packed_ids, = db.execute("SELECT opt_ids FROM Admin").fetchone()
        optimized_ids = ([i for i, in struct.iter_unpack('<i', zlib.decompress(packed_ids))]
                         if packed_ids else [])
        missing_optimized, = db.execute("""SELECT count(DISTINCT f.word_id) FROM Feature f
            LEFT JOIN Word w ON f.word_id=w.id WHERE f.word_id>0 AND w.id IS NULL
            AND f.node_id IN (SELECT value FROM json_each(?))""", (json.dumps(optimized_ids),)).fetchone()
    log = (folder / "1_gracemo_rgbd_slam.log").read_text()
    quality = list(map(int, re.findall(r"Odom: quality=(\d+)", log)))
    with path.open('rb') as saved:
        fingerprint = hashlib.file_digest(saved, 'sha256').hexdigest()
    return dict(integrity=integrity, database_version=version, missing_dictionary_words=missing_words,
                optimized_node_count=len(optimized_ids), missing_optimized_dictionary_words=missing_optimized,
                mapper_finished_cleanly="[rtabmap-1]: process has finished cleanly" in log,
                sha256=fingerprint,
                keyframes=nodes, nonzero_truth_blobs=truth, directed_links_by_type=links,
                odometry_updates=len(quality), positive_quality=sum(q > 0 for q in quality),
                zero_quality=quality.count(0), visual_goal=(json.loads(
                    (folder / "visual_goal.json").read_text()).get("success", False) if (folder / "visual_goal.json").exists() else None))


if __name__ == "__main__":
    print(json.dumps(summarize(Path(sys.argv[1])), indent=2))
