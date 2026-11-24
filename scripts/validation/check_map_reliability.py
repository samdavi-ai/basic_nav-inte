"""Test the production map validators on a closed source and disposable copies."""
import hashlib
import json
from pathlib import Path
import runpy
import shutil
import sqlite3
import sys
import tempfile

LAUNCH=Path(__file__).resolve().parents[2]/"ros2_ws/src/gracemo_nav2/launch/gazebo_rgbd_mapping.launch.py"


def check(source):
    validators=runpy.run_path(str(LAUNCH))
    def inspect(path):
        with sqlite3.connect(path.resolve().as_uri()+"?mode=ro",uri=True) as db:
            if db.execute("PRAGMA integrity_check").fetchone()!=("ok",):raise ValueError("SQLite integrity failed")
            validators["validate_camera_records"](db,320,240)
            validators["validate_saved_graph"](db)
    def digest():
        with source.open("rb") as stream:return hashlib.file_digest(stream,"sha256").hexdigest()
    before=digest()
    results={}
    cases={"valid":None,"incomplete":"UPDATE Admin SET opt_poses=NULL",
           "version":"UPDATE Admin SET version='unsupported'",
           "calibration":"UPDATE Data SET calibration=zeroblob(length(calibration))",
           "dictionary":"DELETE FROM Word"}
    with tempfile.TemporaryDirectory() as td:
        for case,sql in (cases|{"corrupt":None}).items():
            path=Path(td)/(case+".db")
            if case=="corrupt":path.write_bytes(b"invalid sqlite header")
            else:
                shutil.copyfile(source,path)
                if sql:
                    with sqlite3.connect(path) as db:db.execute(sql)
            try:inspect(path);accepted=True;reason=None
            except (ValueError,sqlite3.Error) as exc:accepted=False;reason=str(exc)
            results[case]=dict(accepted=accepted,expected=case=="valid",reason=reason)
            path.unlink()
    assert before==digest()
    return dict(source=source.as_posix(),sha256=before,cases=results,
                passed=all(r['accepted']==r['expected'] for r in results.values()))


if __name__=="__main__":
    report=check(Path(sys.argv[1]));print(json.dumps(report,indent=2))
    sys.exit(0 if report['passed'] else 1)
