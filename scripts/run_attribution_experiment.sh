#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Map Attribution Experiment"
echo "    Testing OLD map with CURRENT calibrated parameters"
echo "============================================================"

# GPU Acceleration
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source /opt/ros/jazzy/setup.bash

# Ensure clean slate
echo "Cleaning up any stale processes..."
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 2

# 1. Swap in OLD map
echo "Installing OLD map (apartment_map.pgm.bak_pre_regen)..."
cp "$REPO_ROOT/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm.bak_pre_regen" "$REPO_ROOT/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm"
cat << 'EOF' > "$REPO_ROOT/ros2_ws/src/gracemo_nav2/maps/apartment_map.yaml"
image: apartment_map.pgm
mode: trinary
resolution: 0.05
origin: [-9.0, -6.5, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
EOF

# Build package
echo "Building gracemo_nav2 with old map..."
cd "$REPO_ROOT/ros2_ws"
colcon build --packages-select gracemo_nav2 --symlink-install
source "$REPO_ROOT/ros2_ws/install/setup.bash"
cd "$REPO_ROOT"

# Launch headless simulation
echo "Launching headless simulation..."
ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/attribution_sim.log" 2>&1 &
SIM_PID=$!
echo "Simulation launched with PID $SIM_PID. Waiting for /autonomy/state..."

ACTIVE=0
for i in $(seq 1 45); do
    if ros2 topic list 2>/dev/null | grep -q "/autonomy/state"; then
        echo "Autonomy stack active after ${i}s!"
        ACTIVE=1
        sleep 3
        break
    fi
    sleep 1
done

if [ $ACTIVE -eq 0 ]; then
    echo "ERROR: Simulation failed to activate in 45s!"
    kill -9 $SIM_PID 2>/dev/null || true
    killall -9 gz sim gz-sim-server gz-sim-gui 2>/dev/null || true
    exit 1
fi

# Run AT-02 and AT-03 5 runs
echo "Running AT-02 and AT-03 (5 runs each) on OLD map..."
python3 "$REPO_ROOT/scripts/execute_at02_at03_5runs.py" "attribution_old_map_results.json"

# Stop simulation
echo "Stopping simulation..."
kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node 2>/dev/null || true
sleep 3

# Restore NEW full-envelope map
echo "Restoring NEW full-envelope map..."
cp "$REPO_ROOT/docs/apartment_map_regenerated.pgm" "$REPO_ROOT/ros2_ws/src/gracemo_nav2/maps/apartment_map.pgm"
cat << 'EOF' > "$REPO_ROOT/ros2_ws/src/gracemo_nav2/maps/apartment_map.yaml"
image: apartment_map.pgm
mode: trinary
resolution: 0.05
origin: [-9.20, -7.20, 0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
EOF

cd "$REPO_ROOT/ros2_ws"
colcon build --packages-select gracemo_nav2 --symlink-install
echo "Attribution experiment complete. Map restored to new full envelope."

