# Voice navigation: progress and how to run

Voice navigation works in the Gazebo simulation with typed transcripts. You say "take me to the kitchen", the chair transcribes it, picks the place from `locations.yaml`, sends a Nav2 goal and drives there. The live microphone path has not been tested yet.

## Summary of changes

This is what changed on the `voice-sim` branch, oldest first. Details for each item are in the sections below and in the files named here.

**2026-10-04: Nav2 in the Gazebo sim, with named places** (commit `46de1e9`)

- `launch/gazebo_sim.launch.py` gained a `nav2:=true` mode that starts the full Nav2 stack (AMCL, planner, controller, behaviour server, velocity smoother) on a saved map, using the real chair's `nav2_params_3cam_v29.yaml` with `config/nav2_sim.yaml` layered on top. New launch arguments: `nav2`, `map`, `nav2_params_file`, `bt_xml`, `use_collision_monitor`.
- The EKF in the sim now fuses only forward speed from the wheels and takes heading from the IMU. `scripts/odom_axle_to_base.py` shifts the axle odometry to `base_link` for it. Without this, wheel heading drifted about 45° after one trip and AMCL ended more than 1 m off.
- `scripts/world_to_map.py` builds a map from the world file; the generated maps are in `maps/`.
- `scripts/go_to_location.py` and `config/locations.yaml` send a named place (bedroom, kitchen, living room) to Nav2.

**2026-10-04: voice navigation and RGB-D cameras** (commits `695022f`, `0d8c67f`)

- `scripts/voice_nav.py` is the voice node: microphone, faster-whisper, place matching, Nav2 goal, spoken replies. `CMakeLists.txt` installs it with the other two scripts.
- `urdf/wc_gazebo.xacro` now has three simulated RGB-D cameras (front, left, right) on the RealSense topic names, bridged with `bridge_camera:=true`. It also has sim-only fixes: lower caster friction, a lidar raised 0.25 m so pitching does not draw phantom walls, and a lidar scan arc that skips the chair's own frame.
- `scripts/vlm_nav.py` and `scripts/vlm_server.sh` are the phase-2 camera/VLM files. They are not installed and not part of this run.
- `.gitignore` now covers `.venv-voice/`.
- Docs: `docs/architecture.md`, `AUTONOMOUS_NAV.md` (section 2.7), `README.md` and `README.dp.md` describe the voice phase.

**2026-10-05: this file** (commit `8165155`) records how to build and run voice navigation in the sim.

**2026-10-06: forward-first navigation** (commits `0d41650`, `d0391dc`, `2adde49`, `58997c0`, `be108ac`)

- `nav2_params_3cam_v30.yaml` stops the chair driving backwards along its path and switches the planner to Hybrid-A*.
- `wheelchair_robust_nav_v4.xml` and `v5.xml` try forward recoveries first and reverse only as the last step.
- `voice_nav.py` asks the passenger for help when a goal fails.
- `README.md` notes the new config is opt-in, because it is tested in the sim only. The launch defaults are still v29 and the v3 tree.
- `build_src/` and `install_src/` are ignored by git (commit `6c391c0`).

## Status

| Item | State |
|---|---|
| Voice to named places in the sim | Works with typed text (bedroom, kitchen, living room, unknown place, stop, "already moving") |
| Live microphone | Not tested |
| Real chair | Not tested with voice |
| Saving places from a hand-driven map (`save_location.py`) | Not written |

Sim results from typed commands: the chair arrived at the bedroom, kitchen and living room. The gap between Gazebo ground truth and AMCL was about 5 cm at the bedroom and about 30 cm at the kitchen and living room. After arriving, the chair turns to the saved heading using Nav2's Spin. Next to furniture, Spin can refuse with "Collision Ahead", and the chair then keeps the heading it arrived with.

## Models and software

| Part | What we use |
|---|---|
| Speech to text | faster-whisper, model `base.en`, CPU, `compute_type="int8"`, `beam_size=1`, `language="en"` |
| Model cache | `~/.cache/huggingface/hub/models--Systran--faster-whisper-base.en` |
| Place matching | rapidfuzz `WRatio`, score cutoff 85, against the names in `locations.yaml` |
| Microphone capture | `arecord` (16 kHz, mono, S16_LE) from the default ALSA device, with an energy gate: speech starts above max(3 x running noise level, 0.01 RMS) and ends after 1.2 s of silence (9 s maximum) |
| Spoken replies | `spd-say`; the mic is muted while the chair speaks |
| Navigation | Nav2 `NavigateToPose` for the trip, then `Spin` to the saved heading |
| Localization | AMCL on a map generated from the world file, plus an EKF |
| Simulator | Gazebo (`gz sim`) through `ros_gz_sim` and `ros_gz_bridge`, ROS 2 Jazzy |
| Python environment | `.venv-voice`, created with `--system-site-packages` from `/usr/bin/python3`, with faster-whisper and rapidfuzz installed |

The VLM files for phase 2 (`scripts/vlm_nav.py`, `scripts/vlm_server.sh`, model `qwen2.5vl:3b` through Ollama) exist but are not installed and not part of this run.

## Files

All paths are under `~/wheelchair_nav/src/wheelchair_description/` unless noted.

- `scripts/voice_nav.py`: the voice node. Listens on the mic, transcribes, handles "go to / take me to <place>" and "stop", sends Nav2 goals, speaks replies. Also accepts text on `/voice/transcript` and publishes replies on `/voice/status`.
- `scripts/go_to_location.py`: sends a named place to Nav2 from the command line. `voice_nav.py` imports `load_locations` from it.
- `config/locations.yaml`: the named places in the map frame (x, y in metres, yaw in radians): bedroom, kitchen, living_room.
- `launch/gazebo_sim.launch.py`: starts the sim, Nav2 (`nav2:=true`) and the EKF. The EKF takes forward speed from the wheels and heading from the IMU, because the wheel heading drifted about 45 degrees after one trip in the sim.
- `worlds/small_house.world`: the simulated house.
- `config/nav2_sim.yaml`: Nav2 parameters for the sim.
- `scripts/world_to_map.py`: generates the map from the world file.
- `~/wheelchair_nav/maps/small_house_world.yaml` (and `.pgm`): the map the sim uses.
- `~/fastdds_udp.xml`: a UDP-only Fast DDS profile. Without it, Nav2 nodes can report "Node not found" in the sim.
- `~/wheelchair_nav/AUTONOMOUS_NAV.md`: the full guide, with voice navigation in section 2.7.

## Build

The branch `voice-sim` (from commit `0d8c67f`) has these files. A stray `CMakeLists.txt` at the repo root makes `source setup.bash` build the wrong package and fail with a CMake cache mismatch. Build into separate directories, with conda off `PATH`, because conda's Python has no `catkin_pkg`:

```bash
cd ~/wheelchair_nav
conda deactivate; conda deactivate
export PATH=$(echo $PATH | tr ':' '\n' | grep -v miniconda3 | paste -sd:)
source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --build-base build_src --install-base install_src \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

One-time voice environment:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv-voice
.venv-voice/bin/pip install faster-whisper rapidfuzz
```

## Run

Open every terminal without conda, and start each one with:

```bash
cd ~/wheelchair_nav
source /opt/ros/jazzy/setup.bash && source install_src/setup.bash
source offline_env.sh
```

`offline_env.sh` makes the sim work with no internet. Gazebo finds the ROS bridge by multicast, and with Wi-Fi down no interface carries multicast, so the bridge logs `Exception sending a multicast message: Network is unreachable` and the chair never spawns. The script turns multicast on for the loopback interface (asks for sudo once per boot), pins Gazebo to `127.0.0.1`, sets `HF_HUB_OFFLINE=1`, drops the proxy variables, and sets the Fast DDS profile.

Terminal 1 starts the sim and Nav2 with the forward-first config (see "Forward-first navigation" below):

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
  world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml \
  nav2_params_file:=$HOME/wheelchair_nav/src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml \
  bt_xml:=$HOME/wheelchair_nav/src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v5.xml
```

Without the last two arguments the launch falls back to the old defaults (v29 params, which reverse along the path).

Terminal 2, after about 30 s, checks that all eight Nav2 nodes print `active [3]`, then sets the start pose at the origin:

```bash
for n in map_server amcl controller_server planner_server behavior_server bt_navigator velocity_smoother waypoint_follower; do printf '%s: ' $n; ros2 lifecycle get /$n; done
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

Terminal 3 starts the voice node. `HF_HUB_OFFLINE=1` skips the Hugging Face network check, because the model is already cached. The `unset` removes `NO_PROXY` entries such as `::1` that `httpx` cannot parse and that crashed the first run with `httpx.InvalidURL: Invalid port: ':'`.

```bash
source .venv-voice/bin/activate
ros2 run wheelchair_description voice_nav.py
```

Say "take me to the bedroom", "kitchen" or "living room". A bare place name ("kitchen") also works, because a pause often splits the sentence. The recording ends after 1.2 s of silence. Say "stop", "halt" or "cancel" during a trip.

## Test without a microphone

From terminal 2:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'take me to the kitchen'}"
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'stop'}"
```

Expected behavior:

| Input | Reply |
|---|---|
| "take me to the kitchen" | "Going to the kitchen. Say stop to cancel.", drives, then "Arrived at the kitchen." |
| "go to the garage" | "I don't know garage. I know bedroom, kitchen, living room." |
| "stop" | Cancels the goal, "Stopping." |
| New place while moving | "I am already moving. Say stop first." |
| Speech that is not a command | Ignored |

## Stop the sim

Press Ctrl+C in terminal 1, then check for leftovers, because a stray `gz sim` or `parameter_bridge` breaks the next launch:

```bash
ps -eo pid,cmd | grep -E "gz sim|parameter_bridge|nav2_|ekf_node" | grep -v grep
```

Do not run `run_nav`, `run_slam` or `run_localization` while the sim is up. They run `pkill -9 -f ros2` and kill every ROS 2 process.

## Forward-first navigation (2026-10-06)

The chair now drives forward to every place and reverses only as a last-resort recovery. Before this, it often drove backwards, which the passenger cannot see. In the sim it reached the bedroom, kitchen and living room from the spawn pose next to the sofa, with zero reverse commands on `/cmd_vel`. It has not been tested on the real chair.

The reversing had two sources. RPP in `nav2_params_3cam_v29.yaml` had `allow_reversing: true`, so it drove backwards whenever the path started behind the chair. The v3 behaviour tree also backed up first in every recovery cycle, three times (30 + 40 + 50 cm).

### What changed

All new files are versioned copies. The only existing file edited is `voice_nav.py`.

`src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml` is a copy of v29 with these changes, each explained in its header:

1. RPP `allow_reversing: true` → `false`.
2. RPP `use_rotate_to_heading: true`, `rotate_to_heading_min_angle: 0.785`. If the path points more than 45° away, the chair turns in place first.
3. RPP `use_collision_detection: true`. RPP stops if the footprint would hit something in the next 1 s.
4. Progress checker `SimpleProgressChecker` → `PoseProgressChecker` (`required_movement_angle: 0.5`), so a turn in place counts as progress. `movement_time_allowance` 4.0 → 10.0.
5. Velocity smoother `min_velocity` x −0.15 → −0.10 m/s, the backup speed.
6. RPP `max_angular_accel` 0.8 → 3.2. At 0.8, RPP ramps each command from the measured spin, so a turn from rest crept at about 0.05 rad/s and the progress checker aborted it.
7. A second controller, `FollowPathReverse`: RPP with `allow_reversing: true` at 0.10 m/s, used only by the v5 tree's last recovery.
8. Planner `SmacPlanner2D` → `SmacPlannerHybrid` with `motion_model_for_search: DUBIN` (forward only) and `minimum_turning_radius: 0.40`. This was the fix that made the spawn pose work. The 2D planner treats the chair as a point with no heading, so its bedroom path started toward the sofa. The chair's corners sweep 0.61 m when it pivots, the sofa was about 0.5 m away, and every turn and backup was refused with "Collision Ahead". Hybrid-A* starts from the chair's heading and checks the footprint along the path, so it plans a forward curve away from the sofa.

`src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v5.xml` replaces v3's recoveries with a forward-first order, tried once each per cycle: clear costmaps, wait 3 s, spin 60° left, spin 60° right, back up 30 cm at 0.10 m/s, and finally follow the path in reverse with `FollowPathReverse` for at most 4 s. If the chair still cannot get through, the tree gives up after about 17 s of recoveries. `wheelchair_robust_nav_v4.xml` is the same tree without the last step.

`src/wheelchair_description/scripts/voice_nav.py` now says "I could not reach the <place>. I may be stuck. Please help me." when a goal fails.

One change was tried and reverted: setting the local costmap's `track_unknown_space` to `false`. The evidence for it came from a costmap snapshot that was probably stale (`always_send_full_costmap: false`), so it stays `true` as in v29.

### How it was tested

All runs were in the sim, from the spawn pose at the origin, with typed transcripts:

| Run | Config | Result |
|---|---|---|
| Bedroom | v30 changes 1–5, v4 tree | Turned at about 0.05 rad/s, aborted with "Failed to make progress" |
| Bedroom | + change 6 | Turned to −50° in under 10 s, then stopped by the sofa; all recoveries refused |
| Bedroom | + change 7, v5 tree | Reversed 0.33 m along the path at 0.10 m/s, still boxed in, voice asked for help |
| Bedroom | + change 8 | Arrived in about 25 s, forward only |
| Kitchen | same | Arrived in 51 s, forward only |
| Living room | same | Arrived in 37 s, forward only |

Checks used: `ros2 param get /controller_server FollowPath.allow_reversing` prints `False`, and `ros2 topic echo /cmd_vel --field linear.x | awk '$1+0 < 0'` stays silent unless a recovery runs. The controller, behaviour server and planner logs are under `~/.ros/log/`.

### Risks

- Hybrid-A* uses more CPU than the 2D planner. It replans at 0.5 Hz in the tree, which was fine in the sim, but it has not been measured on the Jetson.
- If the chair is boxed in with no forward exit, the Dubin planner finds no path. The v5 recoveries then run, including up to 0.4 m of path-reverse, and the voice asks for help.
- On the real chair the cameras face front, left and right, so any reverse relies on the lidar alone to see behind.
- RPP collision detection was turned off in earlier configs because of false stops from phantom STVL obstacles. STVL is gone now, but if false stops return on the real chair, set `use_collision_detection: false` in a new version.
- The planner logs that `inflation_radius: 0.40` is too small for its footprint checks. That affects planning speed, not safety. The sim overrides it to 0.65 in `nav2_sim.yaml`.
- AMCL drift is still there: at the living room, AMCL and Gazebo ground truth differed by about 60 cm, and at the bedroom by about 26 cm.

The launch defaults still point to v29 and v3. To make v30 and v5 the default, change two lines each in `gazebo_sim.launch.py` and `wheelchair_fusion_nav.launch.py`, then rebuild.

## Known issues and next steps

- The root `CMakeLists.txt` should be removed, along with the stale `build/` and `install/`. This needs explicit approval, so it has not been done.
- The background-noise threshold of the energy gate is untested in a real room. If it misfires, the gate is `max(3.0 * noise, 0.01)` in `listen()` in `voice_nav.py`.
- Still to do: update `docs/architecture.md` so voice-only is the current phase, write `save_location.py`, and merge `voice-sim` into `voice-pipeline` so GitHub has the files again.
- Still to do: make v30 and v5 the launch defaults (see "Forward-first navigation"), test the live microphone, and test voice and forward-first navigation on the real chair. The last two need the hardware.

## Which simulation to use

Use this simulation (`gazebo_sim.launch.py` with `world_name:=small_house`) as the base. It already has Nav2, AMCL, the EKF fix, a generated map and named places, and the places are already hardcoded in `config/locations.yaml` as x, y (metres) and yaw (radians) in the map frame. Finishing a second simulation would repeat that setup.

A separate half-finished simulation is only worth keeping if it has something this one lacks, such as a different world or layout. In that case, copy its world file into `src/wheelchair_description/worlds/`, generate its map with `scripts/world_to_map.py`, and add its places to `config/locations.yaml`.
