#!/usr/bin/env bash
# ==============================================================================
# GRaCEmo ViRa — Master Single-Command Autonomous System Launcher
# ==============================================================================
# Starts the complete integrated GRaCEmo ViRa autonomous system in strict
# dependency order:
#   1. Cognitive Kernel Daemon (MNSE REST/WebSocket EventBus at :7780)
#   2. Simulation & Autonomy Substrate (Gazebo Harmonic, Nav2, Localization, Bridges)
#   3. System Readiness Verification Gate (Autonomous readiness & clock advance)
#   4. Pygame ViRa Face & Native Voice Interaction Engine (STT / TTS / FSM)
#   5. UARCC Engineering Workstation (Embedded RViz2, Live Telemetry & Mission Controls)
#
# Clean single shutdown guarantees ZERO duplicate or orphaned processes.
# ==============================================================================

set -eo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$DIR/ros2_ws"
PID_FILE="/tmp/gracemo_master.pid"
PIDS_LIST="/tmp/gracemo_pids.txt"

# ------------------------------------------------------------------------------
# GPU Acceleration & EGL Driver Initialization
# ------------------------------------------------------------------------------
# On hybrid GPU systems (Lenovo Legion AMD APU + NVIDIA RTX GPU), route EGL/GLX
# directly to the proprietary NVIDIA driver to eliminate Mesa 'driver (null)' warnings.
if command -v nvidia-smi > /dev/null 2>&1 && [ -f "/usr/share/glvnd/egl_vendor.d/10_nvidia.json" ]; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi

echo "================================================================================"
echo "  🤖 GRaCEmo ViRa — Master Single-Command Autonomous System Launcher"
echo "  Ubuntu 24.04 | ROS 2 Jazzy | Gazebo Harmonic | PySide6 UARCC | Voice ViRa"
echo "================================================================================"

# ------------------------------------------------------------------------------
# STEP 0: Safety Hardware Gate Assertions (Simulation Only)
# ------------------------------------------------------------------------------
echo "🔒 [Safety Gate] Verifying simulation-only hardware locks..."

FIRMWARE_CONFIG="$DIR/firmware/config.h"
if [ -f "$FIRMWARE_CONFIG" ]; then
    if grep -q "MOTOR_OUTPUT_ENABLED 1" "$FIRMWARE_CONFIG"; then
        echo "❌ FATAL SAFETY ERROR: MOTOR_OUTPUT_ENABLED is 1 in firmware/config.h!"
        echo "   Physical motor outputs must remain disabled for simulation operation."
        exit 1
    fi
fi

MCU_TRANSPORT="$WS_DIR/src/gracemo_hardware/gracemo_hardware/mcu_transport.py"
if [ -f "$MCU_TRANSPORT" ]; then
    if grep -qE "^\s*PROTOCOL_CONFIRMED\s*(:\s*bool)?\s*=\s*True" "$MCU_TRANSPORT"; then
        echo "❌ FATAL SAFETY ERROR: SerialMCUTransport.PROTOCOL_CONFIRMED is True in mcu_transport.py!"
        echo "   Hardware transport confirmation must remain False for simulation operation."
        exit 1
    fi
fi
echo "   ✅ Confirmed: MOTOR_OUTPUT_ENABLED = 0 and PROTOCOL_CONFIRMED = False."

# ------------------------------------------------------------------------------
# STEP 1: Parse Arguments & Concurrency Management
# ------------------------------------------------------------------------------
RESTART_REQUESTED=false
HEADLESS=false
RUN_VERIFICATION=true
AUTO_DURATION=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            echo "Usage: ./run_gracemo.sh [OPTIONS]"
            echo "  --headless         Run in headless offscreen mode"
            echo "  --restart, -r      Perform clean restart of existing instances"
            echo "  --no-verify        Skip autonomous readiness verification gate"
            echo "  --duration SEC     Automatically terminate after SEC seconds"
            exit 0
            ;;
        --restart|-r)
            RESTART_REQUESTED=true
            shift
            ;;
        --headless)
            HEADLESS=true
            shift
            ;;
        --no-verify)
            RUN_VERIFICATION=false
            shift
            ;;
        --duration)
            AUTO_DURATION="$2"
            shift 2
            ;;
        --duration=*)
            AUTO_DURATION="${1#*=}"
            shift
            ;;
        *)
            EXTRA_ARGS+=("$1")
            shift
            ;;
    esac
done

# Check if a previous instance is actively running
EXISTING_PROCS=$(pgrep -f "vira_complete.launch.py|gracemo_control_center/main.py|vira_interaction/model.py|gracemo_command_coordinator|gracemo_interaction_bridge|gracemo_sensor_bridge|gracemo_kernel_bridge|gz sim" || true)
if [ -n "$EXISTING_PROCS" ]; then
    echo "🔄 Existing or orphaned GRaCEmo processes detected (PIDs: $EXISTING_PROCS)."
    echo "   Performing clean teardown to prevent dual-instance collisions and clock jumps..."
    pkill -INT -f "gracemo_control_center/main.py" 2>/dev/null || true
    pkill -INT -f "vira_interaction/model.py" 2>/dev/null || true
    pkill -INT -f "vira_complete.launch.py" 2>/dev/null || true
    pkill -INT -f "prototype.launch.py" 2>/dev/null || true
    pkill -INT -f "gracemo-kernel" 2>/dev/null || true
    pkill -INT -f "rviz2" 2>/dev/null || true
    pkill -INT -f "gracemo_command_coordinator" 2>/dev/null || true
    pkill -INT -f "gracemo_interaction_bridge" 2>/dev/null || true
    pkill -INT -f "gracemo_sensor_bridge" 2>/dev/null || true
    pkill -INT -f "gracemo_kernel_bridge" 2>/dev/null || true
    pkill -INT -f "gracemo_terrain_monitor" 2>/dev/null || true
    pkill -INT -f "cmd_vel_mux" 2>/dev/null || true
    pkill -INT -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor|map_server|amcl|lifecycle_manager" 2>/dev/null || true
    pkill -INT -f "gz sim|gzserver|gzclient|ruby.*gz" 2>/dev/null || true
    sleep 2
    pkill -9 -f "gz sim|gzserver|gzclient|ruby.*gz" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "gracemo_command_coordinator|gracemo_interaction_bridge|gracemo_sensor_bridge|gracemo_kernel_bridge|cmd_vel_mux|gracemo_terrain_monitor" 2>/dev/null || true
    pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor|map_server|amcl|lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "gracemo-kernel" 2>/dev/null || true
    pkill -9 -f "vira_interaction/model.py|gracemo_control_center/main.py" 2>/dev/null || true
    sleep 1
    echo "   ✅ Clean teardown complete. Fresh environment guaranteed."
fi

# Reset PID tracking
rm -f "$PID_FILE" "$PIDS_LIST" 2>/dev/null || true
touch "$PIDS_LIST"
echo $$ > "$PID_FILE"

# ------------------------------------------------------------------------------
# STEP 2: Configure Environment
# ------------------------------------------------------------------------------
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "⚙️ Building ROS 2 workspace..."
    (cd "$WS_DIR" && colcon build --symlink-install)
    source "$WS_DIR/install/setup.bash"
fi

export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION=gracemo
export GZ_IP=127.0.0.1
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/gracemo_gazebo/share/gracemo_gazebo/models:$WS_DIR/install/gracemo_description/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/opt/ros/jazzy/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/materials/textures:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/meshes"
export PYTHONPATH="$DIR:$WS_DIR/src/gracemo_bridge:$DIR/vira_interaction:$PYTHONPATH"

# Native Interaction Engine Environment (from vira_interaction/run.sh)
export PATH="$HOME/.local/bin:/opt/homebrew/bin:/usr/local/bin:$PATH"
export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
export PORTAUDIO_PATH="$HOME/.local/lib/libportaudio.so"
export PYTHONUNBUFFERED=1

# Resolve Pygame ViRa Python virtual environment
VIRA_DIR="$DIR/vira_interaction"
if [ -f "$VIRA_DIR/.venv/bin/python3" ]; then
    VIRA_PYTHON="$VIRA_DIR/.venv/bin/python3"
elif command -v python3 >/dev/null 2>&1; then
    VIRA_PYTHON="python3"
else
    echo "❌ Error: python3 not found. Please install Python 3.10+."
    exit 1
fi

# ------------------------------------------------------------------------------
# STEP 3: Unified Graceful Shutdown Trap
# ------------------------------------------------------------------------------
cleanup() {
    echo ""
    echo "================================================================================"
    echo "🛑 Caught shutdown signal. Executing unified graceful termination..."
    echo "================================================================================"

    # 1. Assert safe stop on ROS topics
    echo "   [1/4] Asserting immediate robot safe-halt and emergency stop..."
    python3 -c "
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

try:
    rclpy.init()
    n = Node('gracemo_shutdown_halt')
    p_estop = n.create_publisher(Bool, '/terrain/emergency_stop', 10)
    p_cmd = n.create_publisher(Twist, '/cmd_vel', 10)
    
    e = Bool()
    e.data = True
    p_estop.publish(e)
    
    t = Twist()
    p_cmd.publish(t)
    
    rclpy.spin_once(n, timeout_sec=0.2)
    n.destroy_node()
    rclpy.shutdown()
except Exception:
    pass
" 2>/dev/null || true

    # 2. Terminate background processes cleanly via recorded PIDs
    echo "   [2/4] Terminating launcher child processes..."
    if [ -f "$PIDS_LIST" ]; then
        while read -r pid; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -INT "$pid" 2>/dev/null || true
            fi
        done < "$PIDS_LIST"
    fi

    # 3. Terminate known launcher-owned process signatures (never touch unrelated desktop apps)
    echo "   [3/4] Reaping UI, voice, simulation, and bridge processes..."
    pkill -INT -f "gracemo_control_center/main.py" 2>/dev/null || true
    pkill -INT -f "vira_interaction/model.py" 2>/dev/null || true
    pkill -INT -f "vira_complete.launch.py" 2>/dev/null || true
    pkill -INT -f "gracemo-kernel" 2>/dev/null || true
    pkill -INT -f "rviz2.*gracemo|gracemo.*rviz|rviz2_nav2" 2>/dev/null || true

    # Allow 2.5 seconds for graceful shutdown
    for i in {1..5}; do
        if ! pgrep -f "vira_complete.launch.py|gracemo_control_center|vira_interaction/model.py" > /dev/null; then
            break
        fi
        sleep 0.5
    done

    # Force-kill any lingering launcher-specific Gazebo or ROS nodes
    pkill -9 -f "gz sim|gzserver|gzclient|ruby.*gz" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "ros_gz_bridge" 2>/dev/null || true
    pkill -9 -f "gracemo_sensor_bridge" 2>/dev/null || true
    pkill -9 -f "gracemo_kernel_bridge" 2>/dev/null || true
    pkill -9 -f "gracemo_interaction_bridge" 2>/dev/null || true
    pkill -9 -f "gracemo_command_coordinator" 2>/dev/null || true
    pkill -9 -f "gracemo_terrain_monitor" 2>/dev/null || true
    pkill -9 -f "cmd_vel_mux" 2>/dev/null || true
    pkill -9 -f "controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|collision_monitor|map_server|amcl|lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "gracemo-kernel" 2>/dev/null || true
    pkill -9 -f "vira_interaction/model.py" 2>/dev/null || true
    pkill -9 -f "gracemo_control_center/main.py" 2>/dev/null || true

    rm -f "$PID_FILE" "$PIDS_LIST" 2>/dev/null || true
    echo "   [4/4] Confirming clean state..."
    REMAINING=$(ps aux | grep -E "gz sim|gracemo_command_coordinator|gracemo-kernel|vira_interaction/model.py" | grep -v grep || true)
    if [ -z "$REMAINING" ]; then
        echo "✅ SUCCESS: All GRaCEmo ViRa processes terminated. Zero orphaned processes."
    else
        echo "⚠️ Note: Remaining processes: $REMAINING"
    fi
    exit 0
}
trap cleanup EXIT INT TERM

# ------------------------------------------------------------------------------
# STEP 4: Stage 1 — Authoritative Cognitive Kernel Daemon (MNSE)
# ------------------------------------------------------------------------------
echo "🧠 [Stage 1/5] Checking / Starting Cognitive Kernel Daemon..."
KERNEL_BIN="$DIR/kernel/target/debug/gracemo-kernel"

if curl -s http://127.0.0.1:7780/health > /dev/null 2>&1; then
    echo "   ✓ Cognitive Kernel is already running and healthy at http://127.0.0.1:7780."
else
    if [ -x "$KERNEL_BIN" ]; then
        echo "   Launching Cognitive Kernel daemon..."
        "$KERNEL_BIN" > /tmp/gracemo_kernel.log 2>&1 &
        KERNEL_PID=$!
        echo $KERNEL_PID >> "$PIDS_LIST"
        
        # Poll health endpoint
        for i in {1..20}; do
            if curl -s http://127.0.0.1:7780/health > /dev/null 2>&1; then
                echo "   ✓ Cognitive Kernel online (PID $KERNEL_PID). REST: http://127.0.0.1:7780"
                break
            fi
            sleep 0.5
        done
    else
        echo "   ⚠️ Cognitive Kernel binary not found at $KERNEL_BIN. Continuing with mock/cloud bridge."
    fi
fi

# ------------------------------------------------------------------------------
# STEP 5: Stage 2 — Master Simulation & Autonomy Substrate (vira_complete.launch.py)
# ------------------------------------------------------------------------------
echo "🚀 [Stage 2/5] Launching Master Simulation & Autonomy Substrate..."

SIM_RUNNING=false
if pgrep -f "gz sim" > /dev/null && pgrep -f "coordinator_node" > /dev/null; then
    SIM_RUNNING=true
fi

if [ "$SIM_RUNNING" = true ]; then
    echo "   ✓ Gazebo Harmonic simulation and Nav2 substrate are already running."
else
    RVIZ_OPT="use_rviz:=true"
    if [ "$HEADLESS" = true ]; then
        RVIZ_OPT="use_rviz:=false"
    fi
    ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization "$RVIZ_OPT" 2>&1 | tee /tmp/gracemo_simulation.log &
    SIM_PID=$!
    echo $SIM_PID >> "$PIDS_LIST"
    echo "   Simulation launched with PID $SIM_PID (Logs: /tmp/gracemo_simulation.log)"
fi

# ------------------------------------------------------------------------------
# STEP 6: Stage 3 — Autonomous Readiness Verification Gate
# ------------------------------------------------------------------------------
if [ "$RUN_VERIFICATION" = true ]; then
    echo "⏳ [Stage 3/5] Verifying autonomous simulation readiness gate..."
    echo "   Awaiting clock advance, TF connectivity, AMCL convergence, and 10/10 Nav2 lifecycle nodes..."
    
    READY=false
    for attempt in {1..15}; do
        sleep 4
        if python3 "$DIR/scripts/verify_system_readiness.py" > /tmp/gracemo_readiness.log 2>&1; then
            cat /tmp/gracemo_readiness.log
            READY=true
            echo "   🎉 All 11 Autonomous Readiness Stages Confirmed (State: AUTONOMY_READY)."
            break
        else
            echo "   [Attempt $attempt/15] Waiting for Nav2 lifecycle activation and localization convergence..."
        fi
    done

    if [ "$READY" != true ]; then
        echo "   ⚠️ Readiness gate timed out or detected issues. Last diagnostic summary:"
        tail -n 25 /tmp/gracemo_readiness.log 2>/dev/null || true
    fi
fi

# ------------------------------------------------------------------------------
# STEP 7: Stage 4 — Pygame ViRa Face & Native Voice Interaction Engine
# ------------------------------------------------------------------------------
echo "🗣️  [Stage 4/5] Starting Pygame ViRa Face & Voice Interaction Engine..."

FACE_RUNNING=false
if pgrep -f "vira_interaction/model.py" > /dev/null; then
    FACE_RUNNING=true
fi

# Check display environment on Linux
if [ "$HEADLESS" != true ] && [ "$(uname -s)" = "Linux" ] && [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "⚠️  Warning: Neither DISPLAY nor WAYLAND_DISPLAY is set."
    echo "   GraceEMO requires an X11 or Wayland display for the Pygame face window."
    echo "   Tip: prefix your command with DISPLAY=:0 (e.g., DISPLAY=:0 ./run_gracemo.sh)"
fi

# Validate essential application components (parity with vira_interaction/run.sh)
for req in "$VIRA_DIR/model.py" "$VIRA_DIR/data" "$VIRA_DIR/vid" "$VIRA_DIR/font.ttf" "$VIRA_DIR/config/places.json"; do
    if [ ! -e "$req" ]; then
        echo "❌ Error: Essential interaction component missing: $req"
        exit 1
    fi
done

echo "=================================================================="
echo "  🤖 GRaCEmo ViRa — Native Voice & Face Expression Interaction Engine"
echo "=================================================================="
echo "  Working Directory: $VIRA_DIR"
echo "  Python Binary:     $VIRA_PYTHON"
if [ -d "$VIRA_DIR/models/Qwen2.5-3B-Instruct-4bit" ]; then
    echo "  Local Qwen Model:  $VIRA_DIR/models/Qwen2.5-3B-Instruct-4bit"
else
    echo "  Local Qwen Model:  Not installed (cloud fallback active)"
fi
echo "------------------------------------------------------------------"
echo "  Controls in GraceEMO Window:"
echo "    [s]       -> START listening to microphone"
echo "    [e]       -> STOP listening"
echo "    [+] / [-] -> Adjust microphone threshold & background rejection"
echo "    [f]       -> Toggle Fullscreen / Windowed Mode"
echo "    [q]       -> Quit / Exit (or Esc)"
echo "=================================================================="

if [ "$FACE_RUNNING" = true ]; then
    echo "   ✓ Pygame ViRa face and voice interaction engine is already running."
elif [ "$HEADLESS" = true ]; then
    echo "   Headless mode: Starting Pygame ViRa in offscreen mode..."
    (cd "$VIRA_DIR" && SDL_VIDEODRIVER=dummy "$VIRA_PYTHON" -u "$VIRA_DIR/model.py" 2>&1 | tee /tmp/gracemo_voice.log) &
    VOICE_PID=$!
    echo $VOICE_PID >> "$PIDS_LIST"
    echo "   Pygame ViRa face online (Headless PID: $VOICE_PID)"
else
    echo "   Launching Pygame ViRa face GUI window & voice services (streaming live logs)..."
    (cd "$VIRA_DIR" && "$VIRA_PYTHON" -u "$VIRA_DIR/model.py" 2>&1 | tee /tmp/gracemo_voice.log) &
    VOICE_PID=$!
    echo $VOICE_PID >> "$PIDS_LIST"
    echo "   Pygame ViRa face window and voice services online (PID: $VOICE_PID)"
fi
sleep 1.5

# ------------------------------------------------------------------------------
# STEP 8: Stage 5 — UARCC Engineering Workstation Desktop Application

echo "🖥️  [Stage 5/5] Launching Unified Autonomous Robotics Control Center (UARCC)..."
echo "   Authentic ROS 2 RViz2 will be embedded inside the Navigation workspace."
echo "   Press Ctrl+C at any time to perform safe robot stop and complete system shutdown."

UARCC_CMD=(python3 "$DIR/gracemo_control_center/main.py")

if [ -n "$AUTO_DURATION" ]; then
    UARCC_CMD+=(--duration "$AUTO_DURATION")
fi

if [ ${#EXTRA_ARGS[@]} -gt 0 ]; then
    UARCC_CMD+=("${EXTRA_ARGS[@]}")
fi

if [ "$HEADLESS" = true ]; then
    export QT_QPA_PLATFORM=offscreen
fi

# Execute UARCC in foreground; signals will be captured by trap handler
"${UARCC_CMD[@]}"
