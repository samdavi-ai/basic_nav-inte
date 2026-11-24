"""Call the existing registered visual navigation tool locally in private Gazebo."""
import json
import os
from pathlib import Path
import sys
import time
import requests
from adapters.gemini_robotics.tools import ToolRegistry

if not os.environ.get("GZ_PARTITION","").startswith("vira-restart-"):
    raise RuntimeError("Private Gazebo required")
url, folder = sys.argv[1],Path(sys.argv[2])
registry = ToolRegistry(kernel_url=url,use_direct_skills=False)
deadline = time.monotonic()+12
out = {"dispatched":False,"reason":"No current unique inanimate target"}
while time.monotonic() < deadline:
    data = requests.get(url+"/perception",timeout=2).json()
    if data.get("current"):
        observations = data["sample"]["observations"]
        for target in ("chair","sofa","monitor","table"):
            if sum(o["class_name"]==target for o in observations)!=1:
                continue
            k = requests.post(url+"/k/execute",json={"expression":f'graph::locate[entity:"{target}"]'},timeout=2)
            if not k.ok:
                continue
            result = registry.navigate_to_object(target,timeout_sec=65)
            out = dict(target=target,observed_at=data["sample"]["captured_at"],k=k.json(),
                       result=result.data,success=result.success,message=result.message,
                       dispatched=result.data.get("action_dispatched",False))
            break
    if "target" in out:
        break
    time.sleep(.2)
(folder/"visual_goal.json").write_text(json.dumps(out,indent=2))
if not out.get("success"):
    raise SystemExit(1)
