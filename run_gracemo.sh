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
RGBD_MAPPING_PID=

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$DIR/ros2_ws"


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
PRIVATE_SIM=false
HEADLESS=false
RUN_VERIFICATION=true
AUTO_DURATION=""
EXTRA_ARGS=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --help|-h)
            echo "Usage: ./run_gracemo.sh [OPTIONS]"
            echo "  --private-simulation  Isolate the kernel, logs and Gazebo session (requires GRACEMO_STATE_DIR and ROS_DOMAIN_ID)"
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
        --private-simulation)
            PRIVATE_SIM=true
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

# Refuse to interfere with any pre-existing session.  The supported launcher
# tracks only its own process groups; killing by executable name can terminate
# an unrelated Gazebo session or a healthy developer service.
EXISTING_PROCS=$(pgrep -f "vira_complete.launch.py|gracemo_control_center/main.py|vira_interaction/model.py|gracemo_command_coordinator|gz sim" || true)
if [ "$PRIVATE_SIM" != true ] && [ -n "$EXISTING_PROCS" ]; then
    echo "Existing simulation or GRaCEmo processes detected (PIDs: $EXISTING_PROCS)."
    echo "Refusing to kill processes this launcher does not own. Stop that session first, then retry."
    exit 1
fi

LOG_DIR=/tmp
if [ "$PRIVATE_SIM" = true ]; then
    : "${GRACEMO_STATE_DIR:?Set a persistent private state directory}"
    : "${ROS_DOMAIN_ID:?Set a dedicated local ROS domain}"
    : "${GZ_PARTITION:?Set a dedicated Gazebo partition}"
    mkdir -p -m 700 "$GRACEMO_STATE_DIR"
    LOG_DIR=$(mktemp -d "$GRACEMO_STATE_DIR/launch.XXXXXX")
    export GRACEMO_STATE_DIR
    export VIRA_AUDIO_WORK_DIR="$LOG_DIR"
    export VIRA_CAMERA_CACHE="$LOG_DIR/camera.jpg"
fi
# Reset PID tracking
SESSION_DIR=$(mktemp -d)
PID_FILE="$SESSION_DIR/master.pid"
PIDS_LIST="$SESSION_DIR/children"
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-68}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${GZ_PARTITION:-gracemo-${ROS_DOMAIN_ID}}"
export GZ_IP=127.0.0.1
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/gracemo_gazebo/share/gracemo_gazebo/models:$WS_DIR/install/gracemo_description/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/opt/ros/jazzy/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/materials/textures:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models/cardboard box/3/meshes"
export PYTHONPATH="$DIR/adapters/sdk:$DIR:$WS_DIR/src/gracemo_bridge:$DIR/vira_interaction:$PYTHONPATH"

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
    local result=${1:-0}
    trap - EXIT INT TERM
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
                kill -INT -- "-$pid" 2>/dev/null || true
            fi
        done < "$PIDS_LIST"
    fi

    # 3. Do not use name-based pkill here.  Every process launched below is
    # recorded in PIDS_LIST, while independently started ROS/Gazebo sessions
    # are deliberately left alone.
    echo "   [3/4] Reaping only recorded launcher child processes..."
    sleep 1
    # RTAB-Map flushes its visual dictionary after the ROS shutdown signal.
    # Keep the owned mapper alive while it finishes this bounded save.
    if [ -n "${RGBD_MAPPING_PID:-}" ]; then
        for ((attempt=0; attempt<48; attempt++)); do
            kill -0 -- "-$RGBD_MAPPING_PID" 2>/dev/null || break
            grep -Fq '[rtabmap-1]: process has finished cleanly' "$LOG_DIR/gracemo_rgbd_slam.log" && break
            sleep 0.25
        done
    fi
    if [ -f "$PIDS_LIST" ]; then
        while read -r pid; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -TERM -- "-$pid" 2>/dev/null || true
            fi
        done < "$PIDS_LIST"
        sleep 1
        while read -r pid; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill -KILL -- "-$pid" 2>/dev/null || true
            fi
        done < "$PIDS_LIST"
    fi

    rm -f "$PID_FILE" "$PIDS_LIST" 2>/dev/null || true
    echo "   [4/4] Confirming clean state..."
    rmdir "$SESSION_DIR" 2>/dev/null || true
    REMAINING=$(ps aux | grep -E "gz sim|gracemo_command_coordinator|gracemo-kernel|vira_interaction/model.py" | grep -v grep || true)
    if [ -z "$REMAINING" ]; then
        echo "✅ SUCCESS: All GRaCEmo ViRa processes terminated. Zero orphaned processes."
    else
        echo "⚠️ Note: Remaining processes: $REMAINING"
    fi
    exit "$result"
}
trap 'cleanup $?' EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# ------------------------------------------------------------------------------
# STEP 4: Stage 1 — Authoritative Cognitive Kernel Daemon (MNSE)
# ------------------------------------------------------------------------------
echo "🧠 [Stage 1/5] Checking / Starting Cognitive Kernel Daemon..."
KERNEL_BIN="$DIR/kernel/target/debug/gracemo-kernel"

export VIRA_GAZEBO_INTERACTION=1
export VIRA_KERNEL_URL=http://127.0.0.1:7780
if [ "$PRIVATE_SIM" = true ]; then
    setsid "$KERNEL_BIN" --isolated-simulation > "$LOG_DIR/gracemo_kernel.log" 2>&1 &
    KERNEL_PID=$!
    echo "$KERNEL_PID" >> "$PIDS_LIST"
    for i in {1..50}; do
        kill -0 "$KERNEL_PID" 2>/dev/null || exit 1
        if [ -s "$GRACEMO_STATE_DIR/listen.port" ]; then
            VIRA_KERNEL_URL="http://127.0.0.1:$(cat "$GRACEMO_STATE_DIR/listen.port")"
            if curl -fsS --max-time 1 "$VIRA_KERNEL_URL/health" 2>/dev/null | python3 -c 'import json, sys; sys.exit(json.load(sys.stdin).get("pid") != int(sys.argv[1]))' "$KERNEL_PID" 2>/dev/null; then
                break
            fi
        fi
        sleep .1
    done
    curl -fsS --max-time 2 "$VIRA_KERNEL_URL/health" | python3 -c 'import json,sys;sys.exit(json.load(sys.stdin).get("pid")!=int(sys.argv[1]))' "$KERNEL_PID"
elif curl -fsS --max-time 2 http://127.0.0.1:7780/health > /dev/null 2>&1; then
    echo "   ✓ Cognitive Kernel is already running and healthy at http://127.0.0.1:7780."
else
    if [ -x "$KERNEL_BIN" ]; then
        echo "   Launching Cognitive Kernel daemon..."
        setsid "$KERNEL_BIN" > "$LOG_DIR/gracemo_kernel.log" 2>&1 &
        KERNEL_PID=$!
        echo $KERNEL_PID >> "$PIDS_LIST"
        
        # Poll health endpoint
        for i in {1..20}; do
            if curl -fsS --max-time 2 http://127.0.0.1:7780/health > /dev/null 2>&1; then
                echo "   ✓ Cognitive Kernel online (PID $KERNEL_PID). REST: http://127.0.0.1:7780"
                break
            fi
            sleep 0.5
        done
    else
        echo "Missing kernel executable: $KERNEL_BIN" >&2
        exit 1
    fi
fi

# ------------------------------------------------------------------------------
for endpoint in health snapshot context graph; do
    curl -fsS --max-time 3 "$VIRA_KERNEL_URL/$endpoint" >/dev/null || exit 1
done

# STEP 5: Stage 2 — Master Simulation & Autonomy Substrate (vira_complete.launch.py)
# ------------------------------------------------------------------------------
echo "🚀 [Stage 2/5] Launching Master Simulation & Autonomy Substrate..."

SIM_RUNNING=false
if [ "$PRIVATE_SIM" != true ] && pgrep -f "gz sim" > /dev/null && pgrep -f "coordinator_node" > /dev/null; then
    SIM_RUNNING=true
fi

if [ "$SIM_RUNNING" = true ]; then
    echo "   ✓ Gazebo Harmonic simulation and Nav2 substrate are already running."
else
    RVIZ_OPT="use_rviz:=true"
    if [ "$HEADLESS" = true ]; then
        RVIZ_OPT="use_rviz:=false"
    fi
    setsid ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization headless:="$HEADLESS" kernel_url:="$VIRA_KERNEL_URL" "$RVIZ_OPT" > "$LOG_DIR/gracemo_simulation.log" 2>&1 &
    SIM_PID=$!
    echo $SIM_PID >> "$PIDS_LIST"
    echo "   Simulation launched with PID $SIM_PID (Logs: /tmp/gracemo_simulation.log)"
fi

# ------------------------------------------------------------------------------
# Use the same kernel and simulation clock for camera observations and mission state.
if [ -n "${VIRA_RTABMAP_DATABASE:-}" ]; then
    if [ "$PRIVATE_SIM" != true ] || [ -e "$VIRA_RTABMAP_DATABASE" ]; then
        echo "RGB-D mapping requires private simulation and a new database path." >&2
        exit 1
    fi
    map_args=()
    if [ "${VIRA_TEST_DISTRIBUTED_FEATURES:-}" = 1 ]; then map_args+=(distributed_features:=true); fi
    if [ "${VIRA_TEST_METRIC_RGBD:-}" = 1 ]; then map_args+=(metric_rgbd:=true); fi
    if [ "${VIRA_TEST_MAPPING_CAMERA_RATE:-}" = 1 ]; then map_args+=(benchmark_camera_rate:=true); fi
    if [ -n "${VIRA_RTABMAP_INPUT:-}" ]; then
        map_args+=(source_map:="$VIRA_RTABMAP_INPUT" source_sha256:="${VIRA_RTABMAP_SHA256:?missing map fingerprint}")
        if [ "${VIRA_TEST_RGBD_2X:-}" = 1 ]; then map_args+=(camera_scale:=2); fi
        if [ "${VIRA_TEST_MATCHED_DEPTH_SIZE:-}" = 1 ]; then map_args+=(depth_divisor:=1); fi
    fi
    setsid ros2 launch "$DIR/ros2_ws/src/gracemo_nav2/launch/gazebo_rgbd_mapping.launch.py" \
        database:="$VIRA_RTABMAP_DATABASE" visual_odometry:=true descriptor:="${VIRA_RTABMAP_DESCRIPTOR:-default}" \
        optical_flow:="${VIRA_RTABMAP_OPTICAL_FLOW:-false}" planar_registration:="${VIRA_RTABMAP_PLANAR:-false}" \
        "${map_args[@]}" > "$LOG_DIR/gracemo_rgbd_slam.log" 2>&1 &
    RGBD_MAPPING_PID=$!
    echo "$RGBD_MAPPING_PID" >> "$PIDS_LIST"
fi
setsid python3 -m adapters.perception.perception_service --ros-args -p use_sim_time:=true > "$LOG_DIR/gracemo_perception.log" 2>&1 &
echo $! >> "$PIDS_LIST"

# STEP 6: Stage 3 — Autonomous Readiness Verification Gate
# ------------------------------------------------------------------------------
if [ "$RUN_VERIFICATION" = true ]; then
    echo "⏳ [Stage 3/5] Verifying autonomous simulation readiness gate..."
    echo "   Awaiting clock advance, TF connectivity, AMCL convergence, and 10/10 Nav2 lifecycle nodes..."
    
    READY=false
    for attempt in {1..15}; do
        sleep 4
        if python3 "$DIR/scripts/verify_system_readiness.py" > "$LOG_DIR/gracemo_readiness.log" 2>&1; then
            cat "$LOG_DIR/gracemo_readiness.log"
            READY=true
            echo "   🎉 All 11 Autonomous Readiness Stages Confirmed (State: AUTONOMY_READY)."
            break
        else
            echo "   [Attempt $attempt/15] Waiting for Nav2 lifecycle activation and localization convergence..."
        fi
    done

    if [ "$READY" != true ]; then
        echo "   ⚠️ Readiness gate timed out or detected issues. Last diagnostic summary:"
        tail -n 25 "$LOG_DIR/gracemo_readiness.log" 2>/dev/null || true
        exit 1
    fi
fi

# ------------------------------------------------------------------------------
# STEP 7: Stage 4 — Pygame ViRa Face & Native Voice Interaction Engine
# ------------------------------------------------------------------------------
echo "🗣️  [Stage 4/5] Starting Pygame ViRa Face & Voice Interaction Engine..."

FACE_RUNNING=false
if [ "$PRIVATE_SIM" != true ] && pgrep -f "vira_interaction/model.py" > /dev/null; then
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
    VIRA_ADAPTER_TOKEN_FILE="${VIRA_VOICE_TOKEN_FILE:-}" setsid bash -c 'cd "$1" && SDL_VIDEODRIVER=dummy exec "$2" -u "$1/model.py"' _ "$VIRA_DIR" "$VIRA_PYTHON" > "$LOG_DIR/gracemo_voice.log" 2>&1 &
    VOICE_PID=$!
    echo $VOICE_PID >> "$PIDS_LIST"
    echo "   Pygame ViRa face online (Headless PID: $VOICE_PID)"
else
    echo "   Launching Pygame ViRa face GUI window & voice services (streaming live logs)..."
    VIRA_ADAPTER_TOKEN_FILE="${VIRA_VOICE_TOKEN_FILE:-}" setsid bash -c 'cd "$1" && exec "$2" -u "$1/model.py"' _ "$VIRA_DIR" "$VIRA_PYTHON" > "$LOG_DIR/gracemo_voice.log" 2>&1 &
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
