#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Step 1 Quick Checks & 3-Run Step Response"
echo "============================================================"

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi

killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 2

ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=true \
    use_static_localization:=false > "$REPO_ROOT/scratch/quick_checks_sim.log" 2>&1 &
SIM_PID=$!

echo "Waiting for simulation and autonomy stack..."
for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack ready (elapsed: ${i}s)"
        sleep 3
        break
    fi
    sleep 1
done

"$REPO_ROOT/.venv-rl/bin/python" "$REPO_ROOT/scripts/step1_quick_checks.py"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 2
echo "Quick checks complete."
