#!/usr/bin/env bash
set -euo pipefail
URDF="${1:-generated_robot/robot.urdf}"
source /opt/ros/jazzy/setup.bash
test -f "$URDF" || { echo "URDF not found: $URDF"; exit 2; }
check_urdf "$URDF"
python3 - "$URDF" <<'PY'
import os, sys, xml.etree.ElementTree as ET
p=sys.argv[1]
root=ET.parse(p).getroot()
links={x.attrib["name"] for x in root.findall("link")}
joints=root.findall("joint")
bad=[]
for j in joints:
    pa=j.find("parent").attrib["link"]
    ch=j.find("child").attrib["link"]
    if pa not in links or ch not in links:
        bad.append((j.attrib.get("name"), pa, ch))
if bad:
    print("Invalid parent/child references:", bad)
    sys.exit(1)
print(f"Basic XML checks passed: {len(links)} links, {len(joints)} joints.")
PY
