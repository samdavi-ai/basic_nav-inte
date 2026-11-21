#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Alphas Ablation (0.005 vs 0.02, 10 transits each)"
echo "============================================================"

# GPU Acceleration
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi

killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 2

# 1. First run with alphas = 0.005 (10 transits)
echo "--- [1/2] Running 10 hallway transits with alphas = 0.005 ---"
ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/alpha_0005_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active!"
        sleep 3
        break
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/measure_amcl_hallway_transits.py" 10 "ablation_alphas_0005_10transits.json"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3

# 2. Modify nav2_params.yaml to alpha = 0.02
echo "--- [2/2] Running 10 hallway transits with alphas = 0.02 ---"
sed -i 's/alpha1: 0.005/alpha1: 0.02/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha2: 0.005/alpha2: 0.02/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha3: 0.005/alpha3: 0.02/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha4: 0.005/alpha4: 0.02/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha5: 0.005/alpha5: 0.02/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"

cd "$REPO_ROOT/ros2_ws"
colcon build --packages-select gracemo_nav2 --symlink-install
cd "$REPO_ROOT"

ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/alpha_002_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active!"
        sleep 3
        break
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/measure_amcl_hallway_transits.py" 10 "ablation_alphas_002_10transits.json"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3

# Revert back to alpha = 0.005
echo "Reverting alphas back to 0.005..."
sed -i 's/alpha1: 0.02/alpha1: 0.005/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha2: 0.02/alpha2: 0.005/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha3: 0.02/alpha3: 0.005/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha4: 0.02/alpha4: 0.005/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"
sed -i 's/alpha5: 0.02/alpha5: 0.005/' "$REPO_ROOT/ros2_ws/src/gracemo_nav2/config/nav2_params.yaml"

cd "$REPO_ROOT/ros2_ws"
colcon build --packages-select gracemo_nav2 --symlink-install
cd "$REPO_ROOT"

echo "Alpha ablation complete. Reverted to 0.005."

