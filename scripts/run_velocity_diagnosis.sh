#!/usr/bin/env bash
set -eo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$DIR/ros2_ws"

echo "================================================================================"
echo "  🔍 GRaCEmo ViRa — Velocity Pipeline Diagnostic Runner"
echo "================================================================================"

# Verify safety locks
if grep -q "MOTOR_OUTPUT_ENABLED 1" "$DIR/firmware/config.h"; then
    echo "❌ FATAL: MOTOR_OUTPUT_ENABLED is 1 in firmware/config.h!"
    exit 1
fi
if grep -qE "^\s*PROTOCOL_CONFIRMED\s*(:\s*bool)?\s*=\s*True" "$WS_DIR/src/gracemo_hardware/gracemo_hardware/mcu_transport.py"; then
    echo "❌ FATAL: PROTOCOL_CONFIRMED is True in mcu_transport.py!"
    exit 1
fi

pkill -9 -f "gz sim|vira_complete|gracemo_sensor_bridge|gracemo_command_coordinator|gracemo-kernel" 2>/dev/null || true
sleep 1

source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=gracemo
export GZ_IP=127.0.0.1
export DISPLAY="${DISPLAY:-:0}"
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/gracemo_gazebo/share/gracemo_gazebo/models:$WS_DIR/install/gracemo_description/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/opt/ros/jazzy/share"
export PYTHONPATH="$DIR:$WS_DIR/src/gracemo_bridge:$DIR/vira_interaction:$PYTHONPATH"

cleanup() {
    echo ""
    echo "🛑 Shutting down diagnostic simulation..."
    pkill -INT -f "vira_complete.launch.py" 2>/dev/null || true
    pkill -INT -f "gracemo-kernel" 2>/dev/null || true
    sleep 2
    pkill -9 -f "gz sim.*apartment_floor|gz.*apartment_floor" 2>/dev/null || true
    pkill -9 -f "gracemo_sensor_bridge|gracemo_command_coordinator|gracemo_terrain_monitor" 2>/dev/null || true
    pkill -9 -f "nav2_.*|controller_server|planner_server|bt_navigator|amcl|map_server" 2>/dev/null || true
    pkill -9 -f "gracemo-kernel" 2>/dev/null || true
    echo "✅ Teardown complete."
}
trap cleanup EXIT INT TERM

# Start Kernel
KERNEL_BIN="$DIR/kernel/target/debug/gracemo-kernel"
if [ -x "$KERNEL_BIN" ]; then
    echo "🧠 Launching Cognitive Kernel daemon..."
    "$KERNEL_BIN" > /tmp/gracemo_kernel.log 2>&1 &
    for i in {1..20}; do
        if curl -s http://127.0.0.1:7780/health > /dev/null 2>&1; then
            echo "   ✓ Cognitive Kernel online at http://127.0.0.1:7780."
            break
        fi
        sleep 0.5
    done
fi

# Launch Simulation & Autonomy Substrate
echo "🚀 Launching Gazebo Harmonic and Nav2 autonomy substrate..."
ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization use_rviz:=false > /tmp/gracemo_sim_diag.log 2>&1 &
SIM_PID=$!
echo "   Autonomy substrate PID: $SIM_PID (Logs: /tmp/gracemo_sim_diag.log)"

# Await readiness
echo "⏳ Awaiting autonomous simulation readiness..."
READY=false
for attempt in {1..15}; do
    sleep 4
    if python3 "$DIR/scripts/verify_system_readiness.py" > /tmp/gracemo_readiness_diag.log 2>&1; then
        READY=true
        echo "   🎉 All 11 Autonomous Readiness Stages Confirmed (State: AUTONOMY_READY)."
        break
    else
        echo "   [Attempt $attempt/15] Waiting for Nav2 lifecycle activation and localization..."
    fi
done

if [ "$READY" != true ]; then
    echo "❌ System readiness failed. Log summary:"
    cat /tmp/gracemo_readiness_diag.log
    exit 1
fi

echo ""
echo "================================================================================"
echo "🎯 RUNNING VELOCITY PIPELINE DIAGNOSTIC PROBE"
echo "================================================================================"
python3 "$DIR/scripts/diagnose_velocity_pipeline.py" 1.5 0.0 20.0

