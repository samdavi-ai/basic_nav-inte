#!/usr/bin/env bash
set -e
out=${1:?supply a new evidence directory}
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
if [[ -n "${VIRA_RTABMAP_PREFIX:-}" ]]; then
  p=$VIRA_RTABMAP_PREFIX
  export AMENT_PREFIX_PATH="$p:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$p/lib:$p/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
  export PYTHONPATH="$p/lib/python3.12/site-packages:$PYTHONPATH"
fi
export VIRA_RECORD_REGISTRATION=1 VIRA_RTABMAP_DATABASE="$(realpath -m "$out")/map.db" VIRA_TEST_DEPTH_10HZ=1 VIRA_DETECTION_HZ=10
exec python3 scripts/validation/launcher_recovery.py "$out" --vision --visual-goal
