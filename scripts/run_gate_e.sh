#!/usr/bin/env bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

echo "============================================================"
echo "    GRaCEmo ViRa: Gate E 50 Random Multi-Room Missions"
echo "    (AMCL ON, Static OFF, Zero Manual Reset)"
echo "============================================================"

export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json

source /opt/ros/jazzy/setup.bash
if [ -f "$REPO_ROOT/ros2_ws/install/setup.bash" ]; then
    source "$REPO_ROOT/ros2_ws/install/setup.bash"
fi

killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node robot_state_publisher sensor_bridge interaction_bridge terrain_monitor_node cmd_vel_mux_node.py map_server amcl lifecycle_manager controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother collision_monitor 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 2

ros2 launch gracemo_gazebo vira_complete.launch.py \
    use_rviz:=false \
    use_rl:=false \
    use_static_localization:=false > "$REPO_ROOT/scratch/gate_e_sim.log" 2>&1 &
SIM_PID=$!

for i in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q "/navigate_to_pose"; then
        if ros2 action info /navigate_to_pose 2>/dev/null | grep -q "Action servers: 1"; then
            echo "Autonomy stack and /navigate_to_pose action server active!"
            sleep 3
            break
        fi
    fi
    sleep 1
done

python3 "$REPO_ROOT/scripts/run_gate_e_50_missions.py" "$@"

kill -9 $SIM_PID 2>/dev/null || true
killall -9 gz sim gz-sim-server gz-sim-gui nav2_bringup coordinator_node robot_state_publisher sensor_bridge interaction_bridge terrain_monitor_node cmd_vel_mux_node.py map_server amcl lifecycle_manager controller_server smoother_server planner_server behavior_server bt_navigator waypoint_follower velocity_smoother collision_monitor 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null || true
sleep 3
echo "Gate E campaign complete."
