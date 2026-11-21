#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi

export PYTHONPATH="$REPO_ROOT/ros2_ws/src/gracemo_rl:$REPO_ROOT/scripts:$PYTHONPATH"

python3 "$REPO_ROOT/scripts/execute_d1_5runs.py"

