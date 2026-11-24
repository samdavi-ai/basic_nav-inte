#!/usr/bin/env bash
set -e
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
if [[ -n "${VIRA_RTABMAP_PREFIX:-}" ]]; then
  p=$VIRA_RTABMAP_PREFIX
  export AMENT_PREFIX_PATH="$p:$AMENT_PREFIX_PATH" LD_LIBRARY_PATH="$p/lib:$p/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH"
  export PYTHONPATH="$p/lib/python3.12/site-packages:$PYTHONPATH"
fi
exec python3 scripts/validation/run_static_rgbd_trial.py "$@"
