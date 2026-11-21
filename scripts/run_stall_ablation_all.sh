#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Stall Hypothesis Direct Test Suite"
echo "============================================================"

# GPU Acceleration
if command -v nvidia-smi > /dev/null 2>&1 && [ -f "/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi
export PYTHONPATH="$REPO_ROOT/ros2_ws/src/gracemo_rl:$REPO_ROOT/scripts:$PYTHONPATH"

cleanup() {
    echo "Cleaning up simulation processes..."
    killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
    ros2 daemon stop 2>/dev/null || true
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
    sleep 2
}
trap cleanup EXIT INT TERM

# --- PART 1: Launch with RL (use_rl:=true) ---
cleanup
echo "--- 1. Launching Gazebo with RL (use_rl:=true) ---"
setsid ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=true \
    use_static_localization:=false > "$REPO_ROOT/scratch/stall_test_rl_sim.log" 2>&1 &
RL_SIM_PID=$!

echo "Waiting for simulation lifecycle..."
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
        if ros2 topic list 2>/dev/null | grep -q "/cmd_vel_rl"; then
            echo "RL simulation active!"
            sleep 4
            break
        fi
    fi
    sleep 1
done

# Run Condition (a): RL alone
echo "--- Running Condition (a): RL alone (10 runs) ---"
$REPO_ROOT/.venv-rl/bin/python "$REPO_ROOT/scripts/test_stall_hypothesis_online.py" "RL_ALONE" "$REPO_ROOT/scratch/stall_rl_alone_results.json"

# Run Condition (b): RL + Recovery
echo "--- Running Condition (b): RL + Recovery (10 runs) ---"
$REPO_ROOT/.venv-rl/bin/python "$REPO_ROOT/scripts/test_stall_hypothesis_online.py" "RL_RECOVERY" "$REPO_ROOT/scratch/stall_rl_recovery_results.json"

cleanup

# --- PART 2: Launch with DWB (use_rl:=false) ---
echo "--- 2. Launching Gazebo with DWB (use_rl:=false) ---"
setsid ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/stall_test_dwb_sim.log" 2>&1 &
DWB_SIM_PID=$!

echo "Waiting for DWB simulation lifecycle..."
for i in $(seq 1 60); do
    if ros2 node list 2>/dev/null | grep -q "/bt_navigator"; then
        if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
            echo "DWB simulation active!"
            sleep 4
            break
        fi
    fi
    sleep 1
done

# Run Condition (c): DWB
echo "--- Running Condition (c): DWB (10 runs) ---"
$REPO_ROOT/.venv-rl/bin/python "$REPO_ROOT/scripts/test_stall_hypothesis_online.py" "DWB" "$REPO_ROOT/scratch/stall_dwb_results.json"

# Consolidate results into a single report
$REPO_ROOT/.venv-rl/bin/python -c "
import json
with open('$REPO_ROOT/scratch/stall_rl_alone_results.json') as f:
    rl_alone = json.load(f)
with open('$REPO_ROOT/scratch/stall_rl_recovery_results.json') as f:
    rl_rec = json.load(f)
with open('$REPO_ROOT/scratch/stall_dwb_results.json') as f:
    dwb = json.load(f)

summary = {
    'route': 'Bedroom Doorway (-4.567, 1.446, -1.54) -> Bedroom (-5.0, 2.2)',
    'conditions': {
        'RL_alone': rl_alone,
        'RL_recovery': rl_rec,
        'DWB': dwb
    }
}
with open('$REPO_ROOT/scratch/stall_hypothesis_ablation_results.json', 'w') as f:
    json.dump(summary, f, indent=2)
print('✓ Consolidated results saved to: $REPO_ROOT/scratch/stall_hypothesis_ablation_results.json')
"

echo "Stall hypothesis test suite complete."

