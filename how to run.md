# How to run: voice mode and voice + VLM mode (simulation)

Both modes run in the Gazebo sim on the saved `small_house` map. Voice mode sends the chair to the
named places in `src/wheelchair_description/config/locations.yaml` (bedroom, kitchen, living_room).
Voice + VLM mode also takes objects ("take me to the bed"): it looks for them with the three
simulated RGB-D cameras and Qwen2.5-VL 3B, then drives to a free spot next to the object.

The sim needs 4-6 GB of free RAM, so close browsers first. Run every command from
`~/wheelchair_nav`.

## One-time setup

The UDP-only Fast DDS profile. Without it Nav2 nodes can report "Node not found" in the sim.

```bash
cat > ~/fastdds_udp.xml <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <transport_descriptors>
    <transport_descriptor><transport_id>udp_only</transport_id><type>UDPv4</type></transport_descriptor>
  </transport_descriptors>
  <participant profile_name="udp_only_participant" is_default_profile="true">
    <rtps>
      <useBuiltinTransports>false</useBuiltinTransports>
      <userTransports><transport_id>udp_only</transport_id></userTransports>
    </rtps>
  </participant>
</profiles>
EOF
```

The voice venv (the first voice run also downloads the whisper `base.en` model, so do it once with
internet):

```bash
cd ~/wheelchair_nav
/usr/bin/python3 -m venv --system-site-packages .venv-voice
.venv-voice/bin/pip install faster-whisper rapidfuzz
```

For voice + VLM only, the model:

```bash
ollama pull qwen2.5vl:3b
```

Build once (and after any change to `CMakeLists.txt` or a new script):

```bash
cd ~/wheelchair_nav
export PATH=/usr/bin:$PATH   # only if conda (base) is active
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

## Common terminal setup

Every terminal below starts with these lines. Call this the "env block".

```bash
cd ~/wheelchair_nav
export PATH=/usr/bin:$PATH   # only if conda (base) is active
source setup.bash --skip
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml HF_HUB_OFFLINE=1
unset HTTP_PROXY HTTPS_PROXY NO_PROXY no_proxy   # NO_PROXY "::1" crashes httpx in faster-whisper
```

With no internet (Wi-Fi off), replace the last two lines with `source ~/wheelchair_nav/offline_env.sh`.
It enables multicast on loopback (asks for sudo once per boot) so Gazebo and Fast DDS can find each
other, and sets the same exports.

---

## Mode 1: voice only

Three terminals.

**Terminal 1: sim + Nav2.** Run the env block, then:

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house use_rviz:=false nav2:=true map:=$HOME/wheelchair_nav/maps/small_house_world.yaml
```

**Terminal 2: check Nav2, set the start pose.** Wait about 45 s after terminal 1. Run the env
block, then check that all eight nodes print `active [3]`:

```bash
for n in map_server amcl controller_server planner_server behavior_server bt_navigator velocity_smoother waypoint_follower; do printf '%s: ' $n; ros2 lifecycle get /$n; done
```

Set the start pose:

```bash
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

**Terminal 3: voice node.** Run the env block, then:

```bash
source .venv-voice/bin/activate
ros2 run wheelchair_description voice_nav.py
```

When it prints `Voice navigation ready.`, say "take me to the kitchen" (or bedroom, living room).
Say "stop" to cancel. To test without a microphone, type the command in terminal 2:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'take me to the kitchen'}"
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'stop'}"
```

Replies are printed in terminal 3. To watch them from terminal 2: `ros2 topic echo /voice/status`.

---

## Mode 2: voice + VLM

Four terminals. Two differences from mode 1: the sim must bridge the cameras
(`bridge_camera:=true`), and the VLM server must be running. `vlm_nav.py` is not installed by
CMake, so it runs with `python3` from the source tree instead of `ros2 run`. Unlike `voice_nav.py`,
it speaks its replies aloud with `spd-say` and also prints them.

**Terminal 1: VLM server.** No env block needed:

```bash
cd ~/wheelchair_nav
bash src/wheelchair_description/scripts/vlm_server.sh
```

Wait until it prints `server is listening on http://127.0.0.1:8091`. Check it from any terminal:

```bash
curl -s http://127.0.0.1:8091/health
```

**Terminal 2: sim + Nav2 with cameras.** Run the env block, then:

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house use_rviz:=false nav2:=true bridge_camera:=true map:=$HOME/wheelchair_nav/maps/small_house_world.yaml
```

**Terminal 3: check Nav2, set the start pose.** Same as mode 1, terminal 2 (the lifecycle loop,
then the `/initialpose` publish). Also check that the cameras publish:

```bash
ros2 topic hz /camera/color/image_raw
ros2 topic hz /camera/aligned_depth_to_color/image_raw
```

**Terminal 4: voice + VLM node.** Run the env block, then:

```bash
source .venv-voice/bin/activate
python3 src/wheelchair_description/scripts/vlm_nav.py
```

When it says `Voice navigation ready.`, say "take me to the bed" (any object), or a named place
like "take me to the kitchen" (named places skip the VLM). Say "stop" to cancel. To test without a
microphone, type in terminal 3:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'go to the bed'}"
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'stop'}"
```

Terminal 4 logs each step: `VLM box [...] for 'bed', verify: yes`, then the object's map position
and the goal.

---

## Shutting down

Ctrl+C in each terminal, voice node first, sim last. Then kill any leftover Gazebo before the next
launch:

```bash
pkill -9 -f "gz sim"
```

## If something fails

| Symptom | Fix |
|---|---|
| `No module named 'faster_whisper'` | `source .venv-voice/bin/activate` before starting the node. |
| Nav2 nodes not `active [3]`, or "Node not found" | Check `FASTRTPS_DEFAULT_PROFILES_FILE` is exported in every terminal; close browsers to free RAM. |
| Chair never spawns with Wi-Fi off | Use `source ~/wheelchair_nav/offline_env.sh` instead of the exports. |
| Command heard but nothing happens | It needs "go to", "take me to", "drive to" or similar plus a target. Check the `heard:` log lines. |
| `VLM request failed` | VLM server (mode 2, terminal 1) is not running or still loading. |
| `I can't find the <object>` | Cameras not bridged (`bridge_camera:=true`) or the object is not in view; the chair turns around once and looks again. |
