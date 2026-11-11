#!/usr/bin/env bash
# Usage: scripts/mode3_real.sh [--text] [--no-checks]
set -u
set -o pipefail

mode3_pass() { printf 'PASS %s\n' "$*"; }
mode3_fail() { printf 'FAIL %s\n' "$*" >&2; exit 1; }

mode3_stop_group() {
    local pid=$1 label=$2 deadline signal
    [[ -n "$pid" ]] || return 0
    for signal in INT TERM KILL; do
        kill -0 -- "-$pid" 2>/dev/null || break
        if [[ "$signal" != INT ]]; then
            printf 'FAIL %s did not stop after the previous signal; sending SIG%s to group %s\n' \
                "$label" "$signal" "$pid" >&2
        fi
        kill -s "$signal" -- "-$pid" 2>/dev/null || true
        deadline=$((SECONDS + 10))
        while kill -0 -- "-$pid" 2>/dev/null && (( SECONDS < deadline )); do sleep 0.2; done
    done
    if kill -0 -- "-$pid" 2>/dev/null; then
        printf 'FAIL %s process group %s still exists; inspect its PID file\n' "$label" "$pid" >&2
        return 1
    fi
    wait "$pid" 2>/dev/null || true
    mode3_pass "$label process group $pid stopped"
}

mode3_cleanup() {
    local status=$1
    trap - EXIT
    trap '' INT TERM HUP
    # Nav2 must still answer while vlm_nav processes cancellation on SIGINT.
    if mode3_stop_group "$M3_VOICE_PID" voice; then
        [[ -z "$M3_VOICE_PID" ]] || rm -f -- "$M3_VOICE_PID_FILE"
    else
        status=1
    fi
    [[ -z "$M3_VOICE_FD" ]] || exec {M3_VOICE_FD}>&-
    if mode3_stop_group "$M3_NAV_PID" navigation; then
        [[ -z "$M3_NAV_PID" ]] || rm -f -- "$M3_NAV_PID_FILE"
    else
        status=1
    fi
    if [[ -n "$M3_TEE_PID" ]]; then
        kill -TERM "$M3_TEE_PID" 2>/dev/null || true
        wait "$M3_TEE_PID" 2>/dev/null || true
    fi
    printf 'VLM server left running; health: http://127.0.0.1:8091/health\n'
    exit "$status"
}

mode3_memory_check() {
    python3 - <<'PY_MEMORY'
from pathlib import Path

def free_percent(text):
    values = {line.split()[0].rstrip(":"): int(line.split()[1])
              for line in text.splitlines() if line.strip()}
    total = values["MemTotal"] + values["SwapTotal"]
    return 100 * (values["MemAvailable"] + values["SwapFree"]) / total

try:
    percent = free_percent(Path("/proc/meminfo").read_text())
    ok = percent >= 20
    print(f"{'PASS' if ok else 'FAIL'} RAM+swap free {percent:.2f}%; requires >= 20%")
    raise SystemExit(0 if ok else 1)
except (OSError, KeyError, ValueError, ZeroDivisionError) as exc:
    print(f"FAIL cannot measure free RAM+swap: {exc}")
    raise SystemExit(1)
PY_MEMORY
}

mode3_camera_check() {
    local status
    timeout 30s rs-enumerate-devices > "$M3_CAMERA_LOG" 2>&1
    status=$?
    (( status == 0 )) || mode3_fail "rs-enumerate-devices exited $status (124 means timeout); see $M3_CAMERA_LOG"
    python3 - "$M3_CAMERA_LOG" <<'PY_CAMERAS'
from pathlib import Path
import re
import sys

def cameras(text):
    found, serial = {}, None
    for line in text.splitlines():
        if line.strip() == "Device info:":
            serial = None
        match = re.match(r"\s*Serial Number\s*:\s*(\S+)", line)
        if match:
            serial = match[1]
            found[serial] = None
        match = re.match(r"\s*Usb Type Descriptor\s*:\s*(\S+)", line, re.I)
        if match and serial is not None:
            found[serial] = match[1]
    return found

found = cameras(Path(sys.argv[1]).read_text())
for serial, name in (("337122300107", "front D455"), ("146222253403", "left D455"),
                     ("207522077542", "right D435i")):
    if serial not in found:
        print(f"FAIL {name} serial {serial} missing from rs-enumerate-devices")
        raise SystemExit(1)
    print(f"PASS {name} serial {serial} present")
usb = found["337122300107"]
if usb is None or re.fullmatch(r"3\.\d+", usb) is None:
    print(f"FAIL front D455 337122300107 USB descriptor is {usb!r}; requires 3.x; "
          "connect its USB 3 cable directly to the laptop")
    raise SystemExit(1)
print(f"PASS front D455 337122300107 is on USB {usb}")
PY_CAMERAS
}

mode3_serial_check() {
    local label=$1 device=$2
    [[ -c "$device" ]] || mode3_fail "$label serial device $device is missing or is not a character device"
    [[ -r "$device" && -w "$device" ]] || mode3_fail "$label serial device $device is not readable and writable by this user"
    mode3_pass "$label serial device $device present and accessible"
}

mode3_health() {
    curl --noproxy '*' --fail --silent --show-error --max-time 2 \
        http://127.0.0.1:8091/health >/dev/null 2>&1
}

mode3_server() {
    local pid='' deadline status
    if mode3_health; then
        mode3_pass 'VLM server /health is OK on port 8091; reusing it'
        return
    fi
    if [[ -f "$M3_ROOT/logs/mode3_vlm.pid" ]]; then
        read -r pid < "$M3_ROOT/logs/mode3_vlm.pid" || pid=''
    fi
    if [[ "$pid" =~ ^[1-9][0-9]*$ ]] && kill -0 "$pid" 2>/dev/null; then
        printf 'Waiting for tracked VLM server PID %s to become healthy\n' "$pid"
    else
        # Detach only the server: it should survive this terminal and release the run lock.
        nohup env --default-signal=INT,TERM setsid bash \
            "$M3_ROOT/src/wheelchair_description/scripts/vlm_server.sh" \
            >> "$M3_ROOT/logs/mode3_vlm.log" 2>&1 < /dev/null 9>&- &
        pid=$!
        printf '%s\n' "$pid" > "$M3_ROOT/logs/mode3_vlm.pid" || mode3_fail 'cannot write VLM PID file'
        printf 'Starting VLM server PID %s; log: %s/logs/mode3_vlm.log\n' "$pid" "$M3_ROOT"
    fi
    deadline=$((SECONDS + 180))
    while (( SECONDS < deadline )); do
        if mode3_health; then
            mode3_pass 'VLM server /health is OK on port 8091'
            return
        fi
        if ! kill -0 "$pid" 2>/dev/null; then
            wait "$pid" 2>/dev/null; status=$?
            mode3_fail "VLM server PID $pid exited (wait status $status); see logs/mode3_vlm.log"
        fi
        sleep 1
    done
    mode3_fail "VLM /health did not become OK within 180 s; PID $pid left running; see logs/mode3_vlm.log"
}

mode3_nav_alive() {
    local reason status
    if ! kill -0 "$M3_NAV_PID" 2>/dev/null; then
        wait "$M3_NAV_PID" 2>/dev/null; status=$?
        mode3_fail "navigation launch PID $M3_NAV_PID exited with status $status; see $M3_NAV_LOG"
    fi
    reason=$(rg -m 1 'Aborting bringup|RPLidar health status *: *2|RPLidar internal error|write cycle resulted in an error:.*WheelchairSystem' "$M3_NAV_LOG")
    [[ -z "$reason" ]] || mode3_fail "navigation log: $reason; stop and correct hardware before relaunching"
}

mode3_wait_nav() {
    local deadline manager ready
    local -a managers=(nav2_lifecycle_manager)
    if [[ "$M3_MODE" == real ]]; then
        managers=(lifecycle_manager_localization lifecycle_manager_navigation lifecycle_manager_collision_monitor)
    fi
    deadline=$((SECONDS + 180))
    while (( SECONDS < deadline )); do
        mode3_nav_alive
        ready=1
        for manager in "${managers[@]}"; do
            rg -q "\[$manager\].*Managed nodes are active" "$M3_NAV_LOG" || ready=0
        done
        if (( ready )); then
            mode3_pass "Managed nodes are active from all required managers: ${managers[*]}"
            return
        fi
        sleep 1
    done
    for manager in "${managers[@]}"; do
        rg -q "\[$manager\].*Managed nodes are active" "$M3_NAV_LOG" || \
            printf 'FAIL missing Managed nodes are active from %s\n' "$manager" >&2
    done
    mode3_fail "navigation readiness timed out after 180 s; see $M3_NAV_LOG"
}

mode3_ros_check() {
    local timeout_seconds=40
    [[ "$M3_MODE" != real || "$M3_CHECKS" != 1 ]] || timeout_seconds=140
    timeout --signal=TERM --kill-after=3s "${timeout_seconds}s" python3 - "$M3_MODE" "$M3_CHECKS" <<'PY_ROS'
import sys
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sim, rates = sys.argv[1] == "sim", sys.argv[2] == "1"
measurement_seconds = 20 if sim else 120
rclpy.init()
node = rclpy.create_node("mode3_startup_check", parameter_overrides=[Parameter("use_sim_time", value=sim)])
counts = {"/scan": 0} if sim else {"/scan": 0, "/scan_fused": 0}

def count(msg, topic):
    counts[topic] += 1

try:
    if rates:
        for topic in counts:
            node.create_subscription(LaserScan, topic, lambda msg, t=topic: count(msg, t), qos_profile_sensor_data)
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (not rates or all(counts.values())) and "amcl_settle" in node.get_node_names() and (
                not sim or node.get_clock().now().nanoseconds > 0):
            break
    if rates:
        counts = dict.fromkeys(counts, 0)
        start = time.monotonic()
        clock_start = node.get_clock().now().nanoseconds / 1e9
        while time.monotonic() - start < measurement_seconds:
            rclpy.spin_once(node, timeout_sec=0.1)
        wall = time.monotonic() - start
        duration = node.get_clock().now().nanoseconds / 1e9 - clock_start if sim else wall
        if duration <= 0:
            print(f"FAIL simulation clock did not advance during the {measurement_seconds} s scan check")
            raise SystemExit(1)
        for topic, count_ in counts.items():
            hz = count_ / duration
            ok = 9 <= hz <= 11 if topic == "/scan" else hz >= 8
            required = "9–11 Hz (about 10)" if topic == "/scan" else ">= 8 Hz"
            print(f"{'PASS' if ok else 'FAIL'} {topic}: {hz:.2f} Hz, {count_} messages / "
                  f"{duration:.2f} {'sim' if sim else 'wall'} s ({wall:.2f} wall s observed); requires {required}")
            if not ok:
                raise SystemExit(1)
    if "amcl_settle" not in node.get_node_names():
        print("FAIL /amcl_settle is absent from the live ROS graph; rebuild the workspace and inspect the navigation log")
        raise SystemExit(1)
    print("PASS /amcl_settle is present in the live ROS graph")
finally:
    node.destroy_node()
    rclpy.try_shutdown()
PY_ROS
}

mode3_text_command() {
    printf 'In another terminal:\n  cd %q && source setup.bash --skip\n' "$M3_ROOT"
    if [[ "$M3_MODE" == sim ]]; then
        printf '  export FASTRTPS_DEFAULT_PROFILES_FILE=%q\n' "$FASTRTPS_DEFAULT_PROFILES_FILE"
    fi
    printf '  ROS_DOMAIN_ID=%q ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: '\''go to the bed'\''}"\n' "$ROS_DOMAIN_ID"
}

mode3_main() {
    M3_MODE=$1
    shift
    M3_CHECKS=1
    M3_MIC=true
    [[ "$M3_MODE" != sim ]] || M3_MIC=false
    while (( $# )); do
        case "$1" in
            --text) M3_MIC=false ;;
            --no-checks) M3_CHECKS=0 ;;
            -h|--help)
                printf 'Usage: scripts/mode3_%s.sh [--text] [--no-checks]\n' "$M3_MODE"
                printf 'Sim always uses text. --no-checks skips hardware, memory and scan rates; readiness and amcl_settle remain required.\n'
                return 0 ;;
            *) mode3_fail "unknown option: $1" ;;
        esac
        shift
    done
    if [[ "$M3_MODE" == real ]]; then
        printf 'SAFETY: Keep the power cut within reach. FlySky is off; there is no stick override.\n'
    fi
    M3_ROOT=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd) || mode3_fail 'cannot locate workspace'
    cd -- "$M3_ROOT" || mode3_fail "cannot enter $M3_ROOT"
    mkdir -p logs || mode3_fail 'cannot create logs/'
    local stamp tool status pid file
    stamp=$(date +%Y%m%d_%H%M%S)_$$
    M3_NAV_LOG="$M3_ROOT/logs/mode3_${M3_MODE}_nav_${stamp}.log"
    M3_VOICE_LOG="$M3_ROOT/logs/mode3_${M3_MODE}_voice_${stamp}.log"
    M3_CAMERA_LOG="$M3_ROOT/logs/mode3_real_cameras_${stamp}.log"
    M3_NAV_PID_FILE="$M3_ROOT/logs/mode3_${M3_MODE}_nav.pid"
    M3_VOICE_PID_FILE="$M3_ROOT/logs/mode3_${M3_MODE}_voice.pid"
    M3_NAV_PID='' M3_VOICE_PID='' M3_TEE_PID='' M3_VOICE_FD=''
    export PATH="/usr/bin:$PATH"
    for tool in python3 curl setsid nohup env flock timeout rg tee; do
        command -v "$tool" >/dev/null || mode3_fail "required command $tool is missing"
    done
    [[ -x .venv-voice/bin/python ]] || mode3_fail '.venv-voice/bin/python is missing or not executable'
    [[ -f install/setup.bash ]] || mode3_fail 'workspace is not built; run colcon build --base-paths src first'
    # ROS-generated setup files reference unset variables; restore nounset immediately afterwards.
    set +u
    # shellcheck disable=SC1091
    source setup.bash --skip > "logs/mode3_${M3_MODE}_setup_${stamp}.log" 2>&1
    status=$?
    set -u
    (( status == 0 )) || mode3_fail "setup.bash --skip failed ($status); see logs/mode3_${M3_MODE}_setup_${stamp}.log"
    command -v ros2 >/dev/null || mode3_fail 'ros2 is missing after sourcing setup.bash'
    export HF_HUB_OFFLINE=1
    unset HTTP_PROXY HTTPS_PROXY ALL_PROXY NO_PROXY http_proxy https_proxy all_proxy no_proxy
    if [[ "$M3_MODE" == sim ]]; then
        export ROS_DOMAIN_ID=42 FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_udp.xml"
        export GZ_SIM_SERVER_CONFIG_PATH="$M3_ROOT/src/wheelchair_description/config/sim_contact_server.config"
        [[ -r "$FASTRTPS_DEFAULT_PROFILES_FILE" ]] || mode3_fail "DDS profile missing: $FASTRTPS_DEFAULT_PROFILES_FILE"
    else
        export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
    fi
    mode3_pass "workspace sourced; ROS_DOMAIN_ID=$ROS_DOMAIN_ID; voice Python: $M3_ROOT/.venv-voice/bin/python"
    exec 9> "$M3_ROOT/logs/mode3.lock" || mode3_fail 'cannot open Mode 3 run lock'
    flock -n 9 || mode3_fail 'another Mode 3 launcher holds logs/mode3.lock; stop it first'
    # Refuse orphaned groups from a prior launcher; never signal a PID read from an old file.
    for file in "$M3_ROOT"/logs/mode3_{real,sim}_{nav,voice}.pid; do
        [[ -f "$file" ]] || continue
        read -r pid < "$file" || mode3_fail "cannot read PID file $file"
        [[ "$pid" =~ ^[1-9][0-9]*$ ]] || mode3_fail "invalid PID file $file; inspect it before restarting"
        if kill -0 -- "-$pid" 2>/dev/null; then
            mode3_fail "tracked process group $pid still exists ($file); stop the previous run first"
        fi
    done
    mode3_pass 'no other launcher or tracked navigation/voice process group is running'
    set +m
    trap 'mode3_cleanup $?' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
    trap 'exit 129' HUP
    if (( M3_CHECKS )); then
        if [[ "$M3_MODE" == real ]]; then
            command -v rs-enumerate-devices >/dev/null || mode3_fail 'rs-enumerate-devices is missing'
            mode3_camera_check || mode3_fail "camera check failed; see $M3_CAMERA_LOG"
            mode3_serial_check lidar /dev/rplidar
            mode3_serial_check Arduino /dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_9513731393235170B2F1-if00
        fi
        mode3_memory_check || mode3_fail 'memory check failed; free memory before relaunching'
    else
        printf 'SKIP hardware, memory and scan-rate checks: --no-checks was requested\n'
    fi
    mode3_server
    if [[ "$M3_MODE" == real ]]; then
        env --default-signal=INT,TERM setsid ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py \
            record_bag:=false front_color:=true axle_to_base:=0.31984 > "$M3_NAV_LOG" 2>&1 < /dev/null 9>&- &
    else
        env --default-signal=INT,TERM setsid ros2 launch wheelchair_description gazebo_sim.launch.py \
            world_name:=small_house use_rviz:=false nav2:=true bridge_camera:=true \
            "map:=$M3_ROOT/maps/small_house_clearance.yaml" \
            "nav2_params_file:=$M3_ROOT/src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml" \
            > "$M3_NAV_LOG" 2>&1 < /dev/null 9>&- &
    fi
    M3_NAV_PID=$!
    printf '%s\n' "$M3_NAV_PID" > "$M3_NAV_PID_FILE" || mode3_fail 'cannot write navigation PID file'
    printf 'Starting navigation group %s; log: %s\n' "$M3_NAV_PID" "$M3_NAV_LOG"
    mode3_wait_nav
    if (( M3_CHECKS )); then
        if [[ "$M3_MODE" == real ]]; then
            printf 'Checking ROS; keep the chair still during the 120 s scan measurement.\n'
        else
            printf 'Checking ROS; measuring the sim scan rate for 20 wall seconds.\n'
        fi
    else
        printf 'Checking ROS node readiness; scan measurement was skipped.\n'
    fi
    mode3_ros_check
    status=$?
    (( status == 0 )) || mode3_fail "ROS check exited $status (124 means timeout); see the reason above and $M3_NAV_LOG"
    if [[ "$M3_MODE" == real ]]; then
        python3 "$M3_ROOT/scripts/mode3_localization_check.py" || \
            mode3_fail 'Mode 3 localization settings do not match the tested base reference'
        python3 "$M3_ROOT/scripts/mode3_camera_check.py" || \
            mode3_fail 'front RGB/depth stream is not ready for Mode 3'
    else
        python3 "$M3_ROOT/scripts/mode3_camera_check.py" --sim || \
            mode3_fail 'sim RGB/depth stream is not ready for Mode 3'
    fi
    mode3_nav_alive
    if [[ "$M3_MODE" == real ]]; then
        printf 'In RViz, set 2D Pose Estimate on open floor so the scan matches the walls. Press Enter when aligned.\n'
        read -r || mode3_fail 'no Enter received on stdin; use an interactive terminal for the pose check'
        mode3_pass 'operator confirmed the 2D Pose Estimate'
    else
        mode3_pass 'small_house uses the configured initial pose; no RViz pose input required'
    fi
    mode3_nav_alive
    mode3_health || mode3_fail 'VLM /health stopped answering before voice startup'
    mode3_text_command
    printf 'Starting voice (mic=%s); wait for "Voice navigation ready.". Ctrl+C stops voice, then navigation.\nLog: %s\n' \
        "$M3_MIC" "$M3_VOICE_LOG"
    # Keep terminal output live, but isolate signals so the launcher controls shutdown order.
    exec {M3_VOICE_FD}> >(tee "$M3_VOICE_LOG" 9>&-)
    M3_TEE_PID=$!
    env --default-signal=INT,TERM setsid "$M3_ROOT/.venv-voice/bin/python" -u \
        "$M3_ROOT/src/wheelchair_description/scripts/vlm_nav.py" --ros-args \
        -p "use_sim_time:=$([[ "$M3_MODE" == sim ]] && printf true || printf false)" \
        -p "mic:=$M3_MIC" >&"$M3_VOICE_FD" 2>&1 < /dev/null 9>&- &
    M3_VOICE_PID=$!
    exec {M3_VOICE_FD}>&-
    printf '%s\n' "$M3_VOICE_PID" > "$M3_VOICE_PID_FILE" || mode3_fail 'cannot write voice PID file'
    while kill -0 "$M3_VOICE_PID" 2>/dev/null; do
        mode3_nav_alive
        kill -0 "$M3_TEE_PID" 2>/dev/null || mode3_fail "voice log writer exited; check $M3_VOICE_LOG and available disk space"
        sleep 1
    done
    wait "$M3_VOICE_PID"; status=$?
    (( status == 0 )) || mode3_fail "voice node exited with status $status; see $M3_VOICE_LOG"
    mode3_pass 'voice node exited normally'
}

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
    mode3_main real "$@"
fi
