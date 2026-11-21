#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: 20s RL Trace on AT-02 (use_rl:=true)"
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
    use_static_localization:=false > "$REPO_ROOT/scratch/rl_trace_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active with RL enabled!"
        sleep 3
        break
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/trace_rl_at02.py"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3
echo "RL trace complete."

