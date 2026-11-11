#!/usr/bin/env bash
# Local SLAM + RViz; Ctrl+C saves the map while SLAM is still running.
set -u
set -o pipefail
source "$(dirname -- "${BASH_SOURCE[0]}")/mode3_real.sh" || exit 1

map_cleanup() {
    local status=$1
    trap - EXIT
    trap '' INT TERM HUP
    if [[ -n "$MAP_CHECK_PID" ]]; then
        kill -TERM "$MAP_CHECK_PID" 2>/dev/null || true
        wait "$MAP_CHECK_PID" 2>/dev/null || true
    fi
    if [[ -n "$M3_NAV_PID" ]] && kill -0 "$M3_NAV_PID" 2>/dev/null; then
        printf 'Saving before stopping SLAM. Keep this terminal open...\n'
        timeout --signal=TERM --kill-after=3s 50s python3 "$M3_ROOT/scripts/save_mapping.py" "$MAP_OUTPUT" \
            > "$MAP_OUTPUT/save.log" 2>&1
        if [[ $? == 0 ]]; then
            cat "$MAP_OUTPUT/save.log"
            printf 'Saved navigation map: %s/map.yaml\n' "$MAP_OUTPUT"
        else
            cat "$MAP_OUTPUT/save.log"
            printf 'FAIL map was not confirmed saved; inspect %s\n' "$MAP_OUTPUT" >&2
            status=1
        fi
    fi
    if mode3_stop_group "$M3_NAV_PID" mapping; then
        [[ -z "$M3_NAV_PID" ]] || rm -f "$M3_NAV_PID_FILE"
    else
        status=1
    fi
    exit "$status"
}

M3_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd) || exit 1
cd "$M3_ROOT" || exit 1
MAP_OUTPUT='' MAP_FUSED=false
while (( $# )); do
    case "$1" in
        --output) (( $# >= 2 )) || mode3_fail '--output requires a new directory'; MAP_OUTPUT=$2; shift ;;
        --fused) MAP_FUSED=true ;;
        -h|--help) printf 'Usage: scripts/map_real.sh [--output NEW_DIRECTORY] [--fused]\nCtrl+C saves map.yaml/map.pgm, then stops SLAM. Default is lidar-only mapping.\n'; exit 0 ;;
        *) mode3_fail "unknown option $1" ;;
    esac
    shift
done
[[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] || mode3_fail 'use a desktop terminal so RViz can open'
[[ -f install/setup.bash ]] || mode3_fail 'build the workspace first: colcon build --base-paths src'
export PATH="/usr/bin:$PATH"
set +u
source setup.bash --skip > /dev/null
status=$?
set -u
(( status == 0 )) || mode3_fail 'workspace setup failed'
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" HF_HUB_OFFLINE=1
mkdir -p logs || exit 1
exec 9> logs/mode3.lock || exit 1
flock -n 9 || mode3_fail 'another mapping/navigation launcher is running; stop it first'
for file in logs/mode{2,3}_{real,sim}_{nav,voice}.pid logs/map_real_nav.pid; do
    [[ -f "$file" ]] || continue
    read -r pid < "$file" || mode3_fail "cannot read $file"
    [[ "$pid" =~ ^[1-9][0-9]*$ ]] || mode3_fail "invalid PID in $file"
    ! kill -0 -- "-$pid" 2>/dev/null || mode3_fail "previous session is still running: $file"
done
stamp=$(date +%Y%m%d_%H%M%S)_$$
[[ -n "$MAP_OUTPUT" ]] || MAP_OUTPUT="$M3_ROOT/maps/local_session_$stamp"
[[ "$MAP_OUTPUT" == /* ]] || MAP_OUTPUT="$M3_ROOT/$MAP_OUTPUT"
[[ ! -e "$MAP_OUTPUT" ]] || mode3_fail 'output directory already exists; choose a new one'
mkdir -p "$MAP_OUTPUT" || mode3_fail 'cannot create output directory'
M3_NAV_LOG="$MAP_OUTPUT/mapping.log" M3_NAV_PID_FILE="$M3_ROOT/logs/map_real_nav.pid"
M3_CAMERA_LOG="$MAP_OUTPUT/cameras.log" M3_NAV_PID='' MAP_CHECK_PID=''
for tool in ros2 python3 rs-enumerate-devices timeout setsid env flock rg; do
    command -v "$tool" >/dev/null || mode3_fail "required tool missing: $tool"
done
printf 'Keep the power cut within reach; FlySky stick override is off. Power Arduino first.\nKeep the chair still through startup. Front D455 must be directly on USB 3.\n'
mode3_camera_check || mode3_fail 'camera enumeration failed'
mode3_serial_check lidar /dev/rplidar
mode3_serial_check Arduino /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_9513731393235170B2F1-if00
mode3_memory_check || mode3_fail 'insufficient free memory'
trap 'map_cleanup $?' EXIT
trap 'exit 0' INT
trap 'exit 143' TERM
trap 'exit 129' HUP
env --default-signal=INT,TERM setsid ros2 launch wheelchair_bringup wheelchair_slam_mapping.launch.py \
    rviz:=true use_sim_time:=false "use_fused_slam:=$MAP_FUSED" enable_session_manager:=false \
    dataset_mode:=false enable_plotting:=false lidar_port:=/dev/rplidar \
    port:=/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_9513731393235170B2F1-if00 \
    > "$M3_NAV_LOG" 2>&1 < /dev/null 9>&- &
M3_NAV_PID=$!
printf '%s\n' "$M3_NAV_PID" > "$M3_NAV_PID_FILE" || mode3_fail 'cannot write mapping PID'
printf 'Starting SLAM + RViz; log: %s\nKeep still for the full two-minute scan check.\n' "$M3_NAV_LOG"
timeout --signal=TERM --kill-after=3s 310s python3 "$M3_ROOT/scripts/map_ready_check.py" \
    > "$MAP_OUTPUT/checks.log" 2>&1 9>&- &
MAP_CHECK_PID=$!
while kill -0 "$MAP_CHECK_PID" 2>/dev/null; do
    mode3_nav_alive
    sleep 1
done
wait "$MAP_CHECK_PID"; status=$?
MAP_CHECK_PID=''
cat "$MAP_OUTPUT/checks.log"
(( status == 0 )) || mode3_fail 'mapping startup checks failed'
printf 'Mapping is ready. Drive manually and watch the map in RViz.\nStop the chair and manual control first, then Ctrl+C HERE to save.\nOutput: %s\nROS_DOMAIN_ID=%s\n' "$MAP_OUTPUT" "$ROS_DOMAIN_ID"
while true; do mode3_nav_alive; sleep 1; done
