#!/usr/bin/env bash
# Start the local ROS 2 Jazzy and Gazebo Harmonic simulation stack.
# This launcher only stops process groups that it started itself.

set -eo pipefail

RUN_PATROL=false
SHOW_GUI=true
PATROL_TIMEOUT=300
PATROL_ROOMS=()
PATROL_ROOMS_SPECIFIED=false
while (($#)); do
    case "$1" in
        --patrol)
            RUN_PATROL=true
            shift
            ;;
        --timeout)
            if (($# < 2)); then
                printf '%s\n' '--timeout requires seconds' >&2
                exit 2
            fi
            PATROL_TIMEOUT="$2"
            shift 2
            ;;
        --no-gui)
            SHOW_GUI=false
            shift
            ;;
        --rooms)
            PATROL_ROOMS_SPECIFIED=true
            shift
            while (($#)) && [[ "$1" != --* ]]; do
                PATROL_ROOMS+=("$1")
                shift
            done
            if ((${#PATROL_ROOMS[@]} == 0)); then
                printf '%s\n' '--rooms requires at least one room name' >&2
                exit 2
            fi
            ;;
        --help|-h)
            cat <<'EOF'
Usage: launch_full_simulation.sh [--patrol] [--no-gui] [--timeout SECONDS] [--rooms ROOM ...]

Starts the local ROS 2 Jazzy / Gazebo Harmonic simulation. With --patrol,
waits for autonomy readiness and dispatches a real mission through the kernel.
The default patrol covers all configured rooms. Use --rooms to select a subset.
Press Ctrl+C to stop only processes started by this launcher.
EOF
            exit 0
            ;;
        *)
            printf 'Unknown option: %s\n' "$1" >&2
            exit 2
            ;;
    esac
done

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WS_DIR="$DIR/ros2_ws"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_DIR/install/setup.bash"
KERNEL_BIN="$DIR/kernel/target/debug/gracemo-kernel"
SIM_LOG="/tmp/gracemo_simulation.log"
KERNEL_LOG="/tmp/gracemo_kernel.log"
GUI_LOG="/tmp/gracemo_gazebo_gui.log"
OWNED_GROUPS=()
SHUTTING_DOWN=false

if [[ ! -r "$ROS_SETUP" ]]; then
    printf 'ROS 2 Jazzy setup not found: %s\n' "$ROS_SETUP" >&2
    exit 1
fi
if ! command -v setsid >/dev/null 2>&1; then
    printf 'setsid is required so this launcher can stop only its own child processes.\n' >&2
    exit 1
fi

source "$ROS_SETUP"
if [[ ! -r "$WS_SETUP" ]]; then
    if ! command -v colcon >/dev/null 2>&1; then
        printf 'ROS workspace is not built and colcon is unavailable: %s\n' "$WS_DIR" >&2
        exit 1
    fi
    printf 'ROS workspace is not built; building it now...\n'
    (cd "$WS_DIR" && colcon build --symlink-install)
fi
if [[ ! -r "$WS_SETUP" ]]; then
    printf 'ROS workspace build did not create the expected setup file: %s\n' "$WS_SETUP" >&2
    exit 1
fi

if [[ "$RUN_PATROL" == true && ! -x "$KERNEL_BIN" ]]; then
    printf 'Building the GRaCEmo kernel for the standalone patrol...\n'
    cargo build --manifest-path "$DIR/kernel/Cargo.toml" --bin gracemo-kernel
fi

source "$WS_SETUP"
set -u

export DISPLAY="${DISPLAY:-:0}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-68}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/gracemo_ros_logs}"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="${GZ_PARTITION:-gracemo-${ROS_DOMAIN_ID}}"
export GZ_IP=127.0.0.1
if [[ -z "${GZ_CONFIG_PATH:-}" && -d /usr/share/gz ]]; then
    export GZ_CONFIG_PATH=/usr/share/gz
fi
export QT_X11_NO_MITSHM=1
export GZ_SIM_RESOURCE_PATH="$WS_DIR/install/gracemo_gazebo/share/gracemo_gazebo/models:$WS_DIR/install/gracemo_description/share:$HOME/.gz/fuel/fuel.gazebosim.org/openrobotics/models:/opt/ros/jazzy/share"

start_group() {
    local label="$1"
    local log_file="$2"
    shift 2

    setsid "$@" >"$log_file" 2>&1 &
    local pid=$!
    OWNED_GROUPS+=("$pid")
    printf 'Started %s in process group %s (log: %s)\n' "$label" "$pid" "$log_file"
}

stop_owned_groups() {
    local signal="$1"
    local pgid
    for pgid in "${OWNED_GROUPS[@]}"; do
        if [[ "$pgid" =~ ^[0-9]+$ ]]; then
            kill -s "$signal" -- "-$pgid" 2>/dev/null || true
        fi
    done
}

cleanup() {
    local pgid
    if [[ "$SHUTTING_DOWN" == true ]]; then
        return
    fi
    SHUTTING_DOWN=true
    trap - EXIT INT TERM
    printf '\nStopping simulation processes started by this launcher...\n'

    # ros2 launch forwards SIGINT to its children. Signaling the entire group
    # here delivers it twice and can interrupt node cleanup mid-callback.
    for pgid in "${OWNED_GROUPS[@]}"; do
        kill -INT "$pgid" 2>/dev/null || true
    done
    for _ in {1..30}; do
        local any_running=false
        for pgid in "${OWNED_GROUPS[@]}"; do
            if kill -0 -- "-$pgid" 2>/dev/null; then
                any_running=true
                break
            fi
        done
        [[ "$any_running" == false ]] && break
        sleep 0.1
    done

    stop_owned_groups TERM
    sleep 0.5
    stop_owned_groups KILL
    for pgid in "${OWNED_GROUPS[@]}"; do
        wait "$pgid" 2>/dev/null || true
    done
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

printf 'GRaCEmo ViRa — ROS 2 Jazzy / Gazebo Harmonic simulation\n'

if [[ -x "$KERNEL_BIN" ]]; then
    if command -v curl >/dev/null 2>&1 && curl --fail --silent --max-time 1 http://127.0.0.1:7780/health >/dev/null; then
        printf 'Kernel already responds on 127.0.0.1:7780; leaving that process untouched.\n'
    else
        printf 'Starting GRaCEmo kernel...\n'
        start_group 'kernel' "$KERNEL_LOG" "$KERNEL_BIN"
    fi
else
    printf 'Kernel binary is absent; simulation will continue without kernel telemetry.\n'
fi

printf 'Starting Gazebo, Nav2, and bridge nodes...\n'
start_group 'ROS simulation' "$SIM_LOG" ros2 launch gracemo_gazebo vira_complete.launch.py mode:=localization headless:=true use_rviz:=false

if [[ "$SHOW_GUI" == true ]]; then
    printf 'Opening Gazebo GUI client...\n'
    start_group 'Gazebo GUI' "$GUI_LOG" gz sim -g
fi

if [[ "$RUN_PATROL" == true ]]; then
    mission_args=(--timeout "$PATROL_TIMEOUT" --out "$DIR/docs/validation/standalone_patrol_latest.json")
    if [[ "$PATROL_ROOMS_SPECIFIED" == true ]]; then
        mission_args+=(--rooms "${PATROL_ROOMS[@]}")
    fi
    printf 'Waiting for autonomy readiness, then running the standalone patrol...\n'
    python3 "$DIR/scripts/run_autonomous_mission.py" "${mission_args[@]}"
    printf 'Standalone patrol completed. Simulation remains active until Ctrl+C.\n'
fi

printf 'Simulation is running. Press Ctrl+C to stop only this launch session.\n'
while :; do
    sleep 1
done
