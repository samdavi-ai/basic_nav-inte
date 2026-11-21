#!/usr/bin/env bash
set -e

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$DIR/ros2_ws"
PIDS_LIST="/tmp/gracemo_voice_test_pids.txt"
rm -f "$PIDS_LIST"
touch "$PIDS_LIST"

echo "================================================================================"
echo "  🤖 GRaCEmo ViRa — End-to-End Live Voice Autonomous Navigation Test Harness"
echo "================================================================================"

# Safety Check
echo "🔒 [Safety Check] Verifying simulation hardware locks..."
grep "MOTOR_OUTPUT_ENABLED 0" "$DIR/firmware/config.h" > /dev/null
grep "PROTOCOL_CONFIRMED: bool = False" "$WS_DIR/src/gracemo_hardware/gracemo_hardware/mcu_transport.py" > /dev/null
echo "   ✅ Verified: MOTOR_OUTPUT_ENABLED = 0 and PROTOCOL_CONFIRMED = False."

# Concurrency check: Reuse active session if available
REUSING_SIM=false
if pgrep -f "gz sim" > /dev/null && pgrep -f "coordinator_node" > /dev/null; then
    echo "ℹ️ Active simulation detected. Reusing existing running session..."
    REUSING_SIM=true
else
    echo "🧹 Cleaning any orphaned simulation or bridge instances..."
    pkill -INT -f "vira_complete.launch.py" 2>/dev/null || true
    pkill -INT -f "gracemo-kernel" 2>/dev/null || true
    sleep 1
    pkill -9 -f "gz sim.*apartment_floor|gz.*apartment_floor" 2>/dev/null || true
fi

cleanup() {
    echo ""
    echo "================================================================================"
    echo "🛑 Completing test harness..."
    echo "================================================================================"
    if [ "$REUSING_SIM" = false ]; then
        if [ -f "$PIDS_LIST" ]; then
            while read -r pid; do
                if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                    kill -INT "$pid" 2>/dev/null || true
                fi
            done < "$PIDS_LIST"
        fi
        pkill -INT -f "vira_complete.launch.py" 2>/dev/null || true
        pkill -INT -f "gracemo-kernel" 2>/dev/null || true
        sleep 2
        pkill -9 -f "gz sim.*apartment_floor|gz.*apartment_floor" 2>/dev/null || true
        pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
        pkill -9 -f "gracemo_command_coordinator" 2>/dev/null || true
        pkill -9 -f "gracemo_interaction_bridge" 2>/dev/null || true
        pkill -9 -f "gracemo-kernel" 2>/dev/null || true
    fi
    rm -f "$PIDS_LIST"
    echo "✅ Done."
}
trap cleanup EXIT INT TERM

# Source environment
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"

if command -v nvidia-smi > /dev/null 2>&1 && [ -f "/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=gracemo
export GZ_IP=127.0.0.1
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/gracemo_gazebo/share/gracemo_gazebo/models:$WS_DIR/install/gracemo_description/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/opt/ros/jazzy/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/materials/textures:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/meshes"
export PYTHONPATH="$DIR:$WS_DIR/src/gracemo_bridge:$DIR/vira_interaction:$PYTHONPATH"

# 1. Start Kernel
echo "🧠 [Stage 1/4] Launching Cognitive Kernel Daemon..."
if curl -s http://127.0.0.1:7780/health > /dev/null 2>&1; then
    echo "   ✓ Cognitive Kernel already running."
else
    "$DIR/kernel/target/debug/gracemo-kernel" > /tmp/gracemo_kernel_test.log 2>&1 &
    KERNEL_PID=$!
    echo $KERNEL_PID >> "$PIDS_LIST"

    for i in {1..20}; do
        if curl -s http://127.0.0.1:7780/health > /dev/null 2>&1; then
            echo "   ✓ Cognitive Kernel online (PID $KERNEL_PID)."
            break
        fi
        sleep 0.5
    done
fi

# 2. Launch Simulation Substrate
if [ "$REUSING_SIM" = true ]; then
    echo "🚀 [Stage 2/4] Reusing existing simulation substrate..."
else
    echo "🚀 [Stage 2/4] Launching Simulation Substrate (Headless)..."
    ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization use_rviz:=false headless:=true > /tmp/gracemo_sim_test.log 2>&1 &
    SIM_PID=$!
    echo $SIM_PID >> "$PIDS_LIST"
    echo "   Simulation launched with PID $SIM_PID (Logs: /tmp/gracemo_sim_test.log)"
fi

# 3. Autonomous Readiness Gate
echo "⏳ [Stage 3/4] Awaiting Autonomous Readiness Gate..."
READY=false
for attempt in {1..20}; do
    sleep 3
    if python3 "$DIR/scripts/verify_system_readiness.py" > /tmp/gracemo_readiness_test.log 2>&1; then
        cat /tmp/gracemo_readiness_test.log
        READY=true
        echo "   🎉 All Autonomous Readiness Stages Confirmed (State: AUTONOMY_READY)."
        break
    else
        echo "   [Attempt $attempt/20] Waiting for localization convergence and 10/10 Nav2 lifecycle nodes..."
    fi
done

if [ "$READY" != true ]; then
    echo "❌ Error: System readiness gate timed out! Last diagnostics:"
    cat /tmp/gracemo_readiness_test.log
    exit 1
fi

# 4. Execute Voice Autonomous Navigation Live Validation
echo "🎯 [Stage 4/4] Executing Live Voice Autonomous Navigation Verification..."
python3 "$DIR/scripts/verify_voice_navigation_live_system.py"
echo "🎉 In-Silico and Live Simulation Verification Completed Successfully!"

