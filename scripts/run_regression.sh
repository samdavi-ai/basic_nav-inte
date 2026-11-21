#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa Automated Regression Harness (Phase 7)"
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

# 1. Run unit tests first (pytest, no Gazebo required)
echo "--- [STEP 1] Running Unit Tests (no Gazebo required) ---"
$REPO_ROOT/.venv-rl/bin/pytest -v \
    "$REPO_ROOT/ros2_ws/src/gracemo_rl/test/test_parity_and_consolidation.py" \
    "$REPO_ROOT/ros2_ws/src/gracemo_rl/gracemo_rl/tests/"

is_sim_active() {
    ros2 node list 2>/dev/null | grep -q "/bt_navigator"
}

# 2. Check if Gazebo + Nav2 is running; if not, launch headless
SIM_PID=""
cleanup() {
    if [ -n "$SIM_PID" ]; then
        echo "Shutting down simulation process group (PGID $SIM_PID)..."
        kill -9 -$SIM_PID 2>/dev/null || true
        kill -9 $SIM_PID 2>/dev/null || true
        ros2 daemon stop 2>/dev/null || true
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
        sleep 2
    fi
}
trap cleanup EXIT INT TERM

if ! is_sim_active; then
    echo "--- [STEP 2] Launching Headless Gazebo & Nav2 Stack ---"
    ros2 daemon stop 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    sleep 1
    setsid ros2 launch gracemo_gazebo vira_complete.launch.py \
        use_rviz:=false \
        use_rl:=false \
        use_static_localization:=false > "$REPO_ROOT/scratch/regression_sim.log" 2>&1 &
    SIM_PID=$!
    echo "Launched simulation process group (PGID $SIM_PID). Waiting for Nav2 lifecycle activation..."
    for i in $(seq 1 45); do
        if is_sim_active; then
            if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
                echo "Simulation and Autonomy Coordinator active!"
                sleep 3
                break
            fi
        fi
        sleep 1
    done
fi

# 3. Run the regression evaluation
echo "--- [STEP 3] Running Acceptance Suite & Baseline Verification ---"
python3 "$REPO_ROOT/scripts/regression_harness.py"
