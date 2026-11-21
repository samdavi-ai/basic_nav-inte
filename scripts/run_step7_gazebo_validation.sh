#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Step 7 Live Gazebo RL Validation Suite"
echo "============================================================"

# GPU Acceleration & EGL Driver Initialization
if command -v nvidia-smi > /dev/null 2>&1 && [ -f "/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

# Source ROS 2 Jazzy and workspace
source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi

export PYTHONPATH="$REPO_ROOT/ros2_ws/src/gracemo_rl:$REPO_ROOT/scripts:$PYTHONPATH"

# Cleanup any stale simulation instances
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
ros2 daemon stop 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 2

SIM_PID=""
cleanup() {
    if [ -n "$SIM_PID" ]; then
        echo "Shutting down simulation process group (PGID $SIM_PID)..."
        kill -9 -$SIM_PID 2>/dev/null || true
        kill -9 $SIM_PID 2>/dev/null || true
        killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
        ros2 daemon stop 2>/dev/null || true
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
        sleep 2
    fi
}
trap cleanup EXIT INT TERM

echo "--- Launching Gazebo & Nav2 Stack with RL Local Planner (use_rl:=true) ---"
setsid ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=true \
    use_static_localization:=false > "$REPO_ROOT/scratch/rl_validation_sim.log" 2>&1 &
SIM_PID=$!

echo "Simulation PGID: $SIM_PID. Waiting for full lifecycle activation..."
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
        if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
            if ros2 topic list 2>/dev/null | grep -q "/cmd_vel_rl"; then
                echo "Simulation, Autonomy Coordinator, and RL Local Planner active!"
                sleep 5
                break
            fi
        fi
    fi
    sleep 1
done

echo "--- Running Live RL Evaluation Suite ---"
$REPO_ROOT/.venv-rl/bin/python "$REPO_ROOT/scripts/run_rl_evaluation_suite.py"

echo "Step 7 validation run complete."

