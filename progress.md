# Voice navigation: progress and how to run

Voice navigation works in the Gazebo simulation with typed transcripts. You say "take me to the kitchen", the chair transcribes it, picks the place from `locations.yaml`, sends a Nav2 goal and drives there. The live microphone path has not been tested yet.

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
| Microphone capture | `arecord` (16 kHz, mono, S16_LE) from the default ALSA device, with an energy gate: speech starts above max(3 x running noise level, 0.01 RMS) and ends after 0.75 s of silence (9 s maximum) |
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
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml
```

Terminal 1 starts the sim and Nav2:

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
  world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml
```

Terminal 2, after about 30 s, checks that all eight Nav2 nodes print `active [3]`, then sets the start pose at the origin:

```bash
for n in map_server amcl controller_server planner_server behavior_server bt_navigator velocity_smoother waypoint_follower; do printf '%s: ' $n; ros2 lifecycle get /$n; done
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

Terminal 3 starts the voice node. `HF_HUB_OFFLINE=1` skips the Hugging Face network check, because the model is already cached. The `unset` removes `NO_PROXY` entries such as `::1` that `httpx` cannot parse and that crashed the first run with `httpx.InvalidURL: Invalid port: ':'`.

```bash
source .venv-voice/bin/activate
export HF_HUB_OFFLINE=1
unset NO_PROXY no_proxy
ros2 run wheelchair_description voice_nav.py
```

Say "take me to the bedroom", "kitchen" or "living room". Say "stop", "halt" or "cancel" during a trip.

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

## Known issues and next steps

- The root `CMakeLists.txt` should be removed, along with the stale `build/` and `install/`. This needs explicit approval, so it has not been done.
- `build_src/` and `install_src/` are not in `.gitignore`.
- The background-noise threshold of the energy gate is untested in a real room. If it misfires, the gate is `max(3.0 * noise, 0.01)` in `listen()` in `voice_nav.py`.
- Still to do: update `docs/architecture.md` so voice-only is the current phase, write `save_location.py`, and merge `voice-sim` into `voice-pipeline` so GitHub has the files again.
