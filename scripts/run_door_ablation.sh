#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Door Opening Ablation (1.2m vs 1.5m)"
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

# 1. First run with Door Opening = 1.2m (Default)
echo "--- [1/2] Running 5 runs AT-02 & AT-03 with Door Opening = 1.2m (Baseline) ---"
ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/door_12m_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active!"
        sleep 3
        break
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/execute_at02_at03_5runs.py" "ablation_door_12m_results.json"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3

# 2. Modify apartment_floor.world to Door Opening = 1.5m
echo "--- [2/2] Running 5 runs AT-02 & AT-03 with Door Opening = 1.5m ---"
# Backup if not already backed up
if [ ! -f "$REPO_ROOT/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world.bak_door_ablation" ]; then
    cp "$REPO_ROOT/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world" "$REPO_ROOT/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world.bak_door_ablation"
fi

# Expand doorways to 1.5m:
# hall_north_west: size 3.1, pose -7.45
# hall_north_middle: size 8.5, pose 0.15
# hall_south_west: size 3.1, pose -7.45
# hall_south_middle: size 8.5, pose 0.15
python3 -c '
w_path = "ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world"
with open(w_path, "r") as f:
    c = f.read()

# Replace hallway walls for 1.5m doors
c = c.replace("<link name=\"l\"><pose>-7.3 1.0 1.3 0 0 0</pose>\n        <collision name=\"c\"><geometry><box><size>3.4 0.2 2.6</size></box></geometry></collision>",
              "<link name=\"l\"><pose>-7.45 1.0 1.3 0 0 0</pose>\n        <collision name=\"c\"><geometry><box><size>3.1 0.2 2.6</size></box></geometry></collision>")

c = c.replace("<link name=\"l\"><pose>-7.3 -1.0 1.3 0 0 0</pose>\n        <collision name=\"c\"><geometry><box><size>3.4 0.2 2.6</size></box></geometry></collision>",
              "<link name=\"l\"><pose>-7.45 -1.0 1.3 0 0 0</pose>\n        <collision name=\"c\"><geometry><box><size>3.1 0.2 2.6</size></box></geometry></collision>")

with open(w_path, "w") as f:
    f.write(c)
print("Updated apartment_floor.world for 1.5m door opening.")
'

ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/door_15m_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active!"
        sleep 3
        break
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/execute_at02_at03_5runs.py" "ablation_door_15m_results.json"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3

# Revert apartment_floor.world
echo "Reverting apartment_floor.world back to 1.2m..."
cp "$REPO_ROOT/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world.bak_door_ablation" "$REPO_ROOT/ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world"

echo "Door ablation complete. Reverted to 1.2m baseline."

