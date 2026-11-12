# Local wheelchair launchers: v31, v30 and mapping

These additions are local and have not been committed or pushed. Required Qwen3 4B, Qwen3-VL 4B Instruct and Whisper `base.en` files are already downloaded on this laptop. Launchers use local inference; they never pull a model during a wheelchair run. The Python voice environment and ROS packages must remain installed.

## Before connecting and launching

Use a persistent terminal on the laptop desktop so RViz can open. Power the Arduino first, connect the lidar, put the front D455 directly on the laptop's USB 3 cable, and connect the other cameras. Keep the chair still through initialization and the two-minute scan-rate measurement. Keep the physical power cut within reach: FlySky stick override is off.

Stop an earlier navigation/mapping session with Ctrl+C before starting another. The scripts share a session lock and refuse tracked active process groups. Manually started ROS sessions must also be stopped. No destination is submitted by a launcher itself.

Build if the source/install has changed:

```bash
cd ~/wheelchair_nav
colcon build --base-paths src
```

## v31: voice navigation to saved places

```bash
cd ~/wheelchair_nav
./scripts/v31_real.sh
```

The launcher starts or reuses local Ollama, loads **Qwen3 4B**, and exercises downloaded **Whisper base.en on CUDA/float16** before starting hardware. It explicitly selects the v31 Nav2 params and behavior tree v3, then starts sensors, odometry, AMCL, Nav2, collision monitor and RViz. It retains the previously used zero axle-offset setting for v31.

Wait through the 120-second scan check. In RViz use **2D Pose Estimate** and align the scans with map walls. Press Enter in the launch terminal when aligned. The voice node starts only after this confirmation. Wait for `Voice navigation ready.`.

The default saved places currently are **dp lab** and **optical lab**, on `maps/real_chair_indoor_2026-11-05_v2.yaml`. Say “go to the optical lab”, “take me to the dp lab”, or “stop”. Replies appear in the terminal and on `/voice/status`; there is no speaker.

To use typed commands without loading Whisper or opening the microphone:

```bash
./scripts/v31_real.sh --text
```

For a different map and its matching saved places:

```bash
./scripts/v31_real.sh --map /absolute/path/new_map.yaml --locations new_locations.yaml
```

The locations argument can be an absolute YAML path or a filename installed under `wheelchair_description/config`. Record new saved poses for a new map; the old lab coordinates cannot be reused automatically on a new map. Each entry needs finite `x`, `y`, `yaw`, and may include a `desc` for purpose-based requests.

Ctrl+C stops the voice group first, allowing goal cancellation while Nav2 is still alive, then stops navigation and RViz. The local model service remains available for reuse. Navigation/voice logs use `logs/mode2_real_*`.

## v30: voice navigation to visible objects

```bash
cd ~/wheelchair_nav
./scripts/v30_real.sh
```

This starts or reuses the local **Qwen3-VL 4B Instruct** server and exercises downloaded **Whisper base.en on CPU/int8**. It explicitly selects v30 Nav2 params and behavior tree v5, enables front color, and supplies the tested `axle_to_base:=0.31984` correction. RViz, AMCL, Nav2, collision monitor and sensors start together.

The launcher checks the two-minute scan rates, actual front RGB/aligned depth/intrinsics/TF and ZUPT/AMCL parameters. Align the initial pose in RViz and press Enter before voice starts. Begin with a visible chair or door on open lab floor. Say “take me to the chair”, “go to the door” or “stop”. A door request approaches and faces the panel; it does not open it or drive through it.

```bash
./scripts/v30_real.sh --text
./scripts/v30_real.sh --map /absolute/path/new_map.yaml
```

The current chair/door simulator smoke run passed 2/2. A box request is supported but not simulator-validated. This prepares a limited hardware trial; full Mode 3 and real-chair validation remain open. Speed stays 0.40 m/s. Ctrl+C cancels voice before shutting down navigation/RViz. Logs use `logs/mode3_real_*`.

## Mapping: save when stopped

```bash
cd ~/wheelchair_nav
./scripts/map_real.sh
```

This starts the real sensors, mapping odometry, **SLAM Toolbox** and mapping RViz. It selects lidar-only SLAM by default, matching the usual lidar AMCL navigation map. Depth fusion still runs and is checked. No language model or internet connection is needed.

Wait for `PASS live SLAM map, RViz and map-to-base TF; manual mapping can begin` after the full two-minute scan check. Then drive manually through the area and watch the map in RViz. Revisit previously mapped parts for loop closure. SLAM establishes its own map frame; there is no AMCL 2D Pose Estimate step during mapping.

If using keyboard manual control, start it in another terminal after the checks pass:

```bash
cd ~/wheelchair_nav
source setup.bash --skip
# Use the same domain printed by map_real.sh (normally 0).
export ROS_DOMAIN_ID=0
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
  -p speed:=0.15 -p turn:=0.35 -p stamped:=true -p frame_id:=base_link \
  -r cmd_vel:=/wc_control/cmd_vel
```

Use the displayed teleop keys. Keep speed conservative; do not increase real-chair speed limits. This keyboard command is a separate manual controller, not a FlySky emergency override.

**To finish: stop the chair and the manual-control terminal first, then press Ctrl+C in the mapping terminal.** The script keeps SLAM running while saving, verifies the map files, then stops the mapping process group and RViz. Keep the terminal open until it prints the saved path. A failed save prints `FAIL` and points to the log; it is never reported as successful.

Default output is a new local directory:

```text
maps/local_session_<date>_<time>_<pid>/
    map.yaml
    map.pgm
    map.posegraph        # if SLAM serialization succeeds
    map.data             # if SLAM serialization succeeds
    save_result.json
    save.log
    mapping.log
    checks.log
    cameras.log
```

The YAML/image are the navigation map; the posegraph/data support later continuation in SLAM. If only serialization fails, the script clearly reports that the navigation map is still saved. It disables the old simultaneous-shutdown session manager and does not start a rosbag or dataset recorder. Default session outputs are excluded locally from Git.

Choose a new output directory or optional fused SLAM:

```bash
./scripts/map_real.sh --output maps/my_new_lab_session
./scripts/map_real.sh --fused
```

An existing output directory is refused to protect previous maps. A normal Ctrl+C attempts saving even before readiness completes, provided SLAM is still running; saving cannot succeed if there is no map yet. SIGKILL, power loss or loss of the SLAM process cannot trigger a successful save.

## Offline checks performed

Model warm-up is checked without hardware using `scripts/prepare_voice_models.py`. Map saving is checked against a synthetic ROS map and serialization service on an isolated domain. Launch parameter selection, script syntax and installed launch arguments are checked without launching hardware. These checks do not certify sensor startup or a real route; follow the gates in each launcher.
