"""Score blind stationary-camera restarts using only completed map databases."""
import argparse
import hashlib
import json
import sqlite3
import struct
import zlib
from pathlib import Path

from summarize_relocalization import load, summarize


def evaluate(source, root):
    path=source/"map.db"
    with path.open("rb") as f:
        digest=hashlib.file_digest(f,"sha256").hexdigest()
    with sqlite3.connect(path.resolve().as_uri()+"?mode=ro",uri=True) as db:
        packed,=db.execute("SELECT opt_ids FROM Admin").fetchone()
        ids={i for i, in struct.iter_unpack("<i",zlib.decompress(packed))}
    mapping=load(source)
    results=[]
    for directory in sorted(root.iterdir()):
        if not directory.is_dir():
            continue
        lifecycle=json.loads((directory/"trial.json").read_text())
        if any(lifecycle["exit_codes"].values()) or lifecycle.get("source_sha256")!=digest:
            raise ValueError(f"Unclean or mismatched trial {directory}")
        with sqlite3.connect((directory/"map.db").resolve().as_uri()+"?mode=ro",uri=True) as db:
            integrity,=db.execute("PRAGMA integrity_check").fetchone()
        trial=load(directory)
        graph=trial["graph"]
        # The rig is static for the entire trial. Read back the model pose from
        # Gazebo and use it only in post-processing, at the graph timestamps.
        trial["truth"]=[dict(t=r["t"],**lifecycle["gazebo_pose"]) for r in graph]
        def ambiguous(row):
            return row["stats"].get("Loop/RejectedHypothesis/",0)>0 or len(
                {k for k in (row["loop"],row["proximity"]) if k>0})>1
        trial["graph"]=[r for r in graph if not ambiguous(r)]
        score=summarize(mapping,trial,ids)
        score.update(trial=directory.name,domain=lifecycle["domain"],partition=lifecycle["partition"],
                     optimized_source_ids=sorted(ids),source_sha256=digest,sqlite_integrity=integrity,
                     rejected_or_conflicting=len(graph)-len(trial["graph"]),all_graph_messages=len(graph),
                     odom_nonlost=sum(not r["lost"] for r in trial["info"]),odom_updates=len(trial["info"]))
        score["accepted"]=integrity=="ok" and score["all_matches_within_target"]
        (directory/"evaluation.json").write_text(json.dumps(score,indent=2))
        results.append(score)
    return dict(attempts=len(results),accepted=sum(r["accepted"] for r in results),results=results)


if __name__=="__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("source",type=Path)
    parser.add_argument("root",type=Path)
    args=parser.parse_args()
    (args.root/"results.json").write_text(json.dumps(evaluate(args.source,args.root),indent=2))
