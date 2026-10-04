# Autonomous navigation: simulation and real chair

The chair drives itself to a named place ("kitchen") or to any point on a saved map, from the
command line or from a voice command. Nav2 and the safety layer are the only things that command
the motors. The design, phases and test results are in `docs/architecture.md`.

| Part | State |
|---|---|
| Simulation: saved map, named goals | Works. 6/6 goals succeeded (bedroom, living_room, kitchen, in mixed order), 20-60 s per room-to-room trip. |
| Simulation: voice to named places | Works with typed commands (section 2.7). Microphone not tested yet. |
| Real chair: build a map, navigate on it | Existing pipeline (`run_slam`, `run_nav`). Not re-tested in this work. |
| Real chair: named places | Manual for now: read the pose, edit `locations.yaml` (section 3.4). |

Run every command from `~/wheelchair_nav`. Section 1 is one-time setup, section 2 is the
simulation, section 3 is the real chair.

---

## 1. Setup

### Build

Build after every pull and after adding a file:

```bash
cd ~/wheelchair_nav
export PATH=/usr/bin:$PATH   # only if conda (base) is active
source setup.bash            # builds, sources ROS 2 + the workspace, defines run_nav / run_slam
```

Every other terminal only needs to source, which skips the build:

```bash
cd ~/wheelchair_nav && source setup.bash --skip
```

The build uses `--symlink-install`, so edits to YAML, launch files and scripts under `src/` apply
on the next launch without a rebuild. A new script is the exception: make it executable
(`chmod +x`), add it to `install(PROGRAMS ...)` in `src/wheelchair_description/CMakeLists.txt`,
and rebuild.

### USB rules (real chair, once per laptop)

```bash
sudo cp src/scripts/99-wheelchair-usb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The rules give the Arduino and the lidar fixed names (`/dev/wheelchair_arduino`, `/dev/rplidar`)
and let a normal user open them and the RealSense cameras.

### DDS profile (simulation)

In the sim, Fast DDS shared memory can fail: Nav2 nodes report "Node not found" and service calls
time out. A UDP-only profile avoids this. Save it as `~/fastdds_udp.xml`:

```xml
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
```

and export it in every terminal you use for the sim:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml
```

---

## 2. Simulation

The sim needs 4-6 GB of free RAM. Close browsers first: once swap fills up, the sim slows down
and Nav2 loses nodes. Each sim terminal needs `source setup.bash --skip` and the DDS export from
section 1.

### 2.1 Start the sim with Nav2

Terminal 1:

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
  world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml
```

After about 30 s, all eight Nav2 nodes should report `active [3]`:

```bash
for n in map_server amcl controller_server planner_server behavior_server bt_navigator velocity_smoother waypoint_follower; do
  printf '%s: ' $n; ros2 lifecycle get /$n; done
```

### 2.2 Tell AMCL where the chair is

AMCL needs a start pose before the first goal. The chair spawns at the world origin, so publish
(0, 0) from terminal 2:

```bash
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

### 2.3 Send a goal

To a named place:

```bash
ros2 run wheelchair_description go_to_location.py kitchen      # or bedroom, living_room
```

It prints `kitchen: SUCCEEDED` or `kitchen: FAILED` and exits with 0 or 1.

To any point (x, y in metres, map frame):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
 "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.6, y: -2.0}, orientation: {w: 1.0}}}}"
```

With `use_rviz:=true` you can also set the start pose with **2D Pose Estimate** and the goal with
**2D Goal Pose**. This has not been tried in the sim yet.

By voice: see section 2.7.

### 2.4 Stop the sim

Press Ctrl+C in terminal 1, then make sure nothing is left over. A stray `gz sim` or
`parameter_bridge` breaks the next launch.

```bash
ps -eo pid,cmd | grep -E "gz sim|parameter_bridge|nav2_|ekf_node" | grep -v grep
kill <pid> ...        # kill -9 only if it survives
```

Do not run `run_nav`, `run_slam` or `run_localization` while the sim is up. They start with
`pkill -9 -f ros2` and `pkill -9 -f rviz`, which kill every ROS 2 process on the machine, the sim
included.

### 2.5 Named places

The places live in `src/wheelchair_description/config/locations.yaml`, in the map frame with
`yaw` in radians:

```yaml
bedroom:     {x: -4.54, y: -0.14, yaw: 2.22}
kitchen:     {x: 4.64, y: -1.98, yaw: 0.99}
living_room: {x: 0.20, y: 1.48, yaw: -1.53}
```

Pick spots with at least 1.3 m clearance from furniture. Spots 0.7 m from furniture failed with
"Start occupied", because the chair footprint (0.9 x 0.7 m) plus inflation covered the goal.

### 2.6 Make a sim map

**From the world file** (what the sim uses; exact, no driving):

```bash
python3 src/wheelchair_description/scripts/world_to_map.py \
  src/wheelchair_description/worlds/small_house.world \
  src/wheelchair_description/models \
  maps/small_house_world
```

This writes `maps/small_house_world.pgm` and `.yaml` (0.02 m/pixel, origin -10.50, -6.70). It
rasterizes what the lidar sees: collision shapes between 0.05 and 1.0 m height plus visuals at
lidar height (`LIDAR_Z = 0.29` in the script). Everything outside the house is unknown. Open the
PGM in an image viewer to check it and to read room coordinates off it:
`x = origin_x + col * 0.02`, `y = origin_y + (height - row) * 0.02`.

**By driving (SLAM):**

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
  world_name:=small_house slam:=true teleop:=true teleop_keyboard:=true
```

Drive slowly from the keyboard window, cover every room and close loops, then save from a second
terminal:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/wheelchair_nav/maps/my_map
```

Check the map before you use it. Walls should be thin and straight, with no ghost walls inside
rooms; fuzzy or doubled walls make AMCL and the planner fail. If it looks wrong, drive it again,
slower.

### 2.7 Voice navigation

`voice_nav.py` listens on the laptop microphone, transcribes with faster-whisper (`base.en`, on
the CPU), matches "go to / take me to <place>" against `locations.yaml`, and sends the place to
Nav2. It speaks its replies with `spd-say` and ignores speech that is not a command.

One-time setup (the first run also downloads the whisper model):

```bash
cd ~/wheelchair_nav
/usr/bin/python3 -m venv --system-site-packages .venv-voice
.venv-voice/bin/pip install faster-whisper rapidfuzz
```

With the sim and Nav2 up (2.1) and the start pose set (2.2), in a new sim terminal:

```bash
source .venv-voice/bin/activate
ros2 run wheelchair_description voice_nav.py
```

Then say "take me to the kitchen". To test without a microphone, type the command instead:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'take me to the kitchen'}"
```

| You say | The chair |
|---|---|
| "take me to the kitchen" | Says "Going to the kitchen. Say stop to cancel.", waits 2 s, drives, then turns to the saved heading and says "Arrived at the kitchen." |
| "stop", "halt" or "cancel" | Cancels the goal at any point and says "Stopping." |
| An unknown place | Says "I don't know <place>." and lists the known places |
| A new place while driving | Says "I am already moving. Say stop first." |

Typed tests on 2026-10-04: all four behaviours worked, and the chair reached bedroom, kitchen and
living room. The microphone path has not been tested. The turn to the saved heading uses Nav2's
Spin, which can refuse next to furniture ("Collision Ahead"); the chair then keeps the heading it
arrived with.

---

## 3. Real chair

### 3.1 Connect the laptop

The laptop runs the whole stack. Plug in:

| Device | Role | Shows up as |
|---|---|---|
| Arduino Mega | motors, encoders | `/dev/ttyACM0`, alias `/dev/wheelchair_arduino` |
| RPLidar S3 | `/scan` | `/dev/ttyUSB0`, alias `/dev/rplidar` |
| RealSense D455, front | depth, the only IMU | serial 337122300107 |
| RealSense D455, left | depth | serial 146222253403 |
| RealSense D435i, right | depth | serial 207522077542 |

Put the cameras on USB 3 ports (blue, or marked SS); a powered USB 3 hub helps if the laptop is
short of ports. The launch files open each camera by its serial number. Check that everything
shows up:

```bash
ls -l /dev/wheelchair_arduino /dev/rplidar   # both should point at a ttyACM* / ttyUSB* device
rs-enumerate-devices -s                      # should list all three serial numbers
```

The launch files expect `/dev/ttyACM0` and `/dev/ttyUSB0`. If the aliases point at other numbers
(for example because another USB serial device was plugged in first), pass the aliases instead.
`run_nav` takes the same two arguments. This override has not been tried on the chair yet.

```bash
run_slam port:=/dev/wheelchair_arduino lidar_port:=/dev/rplidar
```

At start, the launch also runs `sudo chmod 666` on both ports using `sudo_password` (default
`12345`). With the USB rules from section 1 installed the ports are already open, so a
`[WARN] Failed to set permissions` line is harmless.

### 3.2 Build a map

1. Power up the chair, put it in the room, stand clear.
2. Start mapping:

   ```bash
   cd ~/wheelchair_nav && source setup.bash --skip
   run_slam                        # lidar + camera map (default)
   run_slam use_fused_slam:=false  # lidar only
   run_slam hospital_mode:=true    # long corridors
   ```

3. Keep the chair still for the first 3 s while the IMU measures its gyro bias. The nodes start
   in stages, and the session manager that saves the map comes up last, at about 42 s. Wait for
   it before you drive; Ctrl+C before then saves nothing.
4. Drive every area slowly with the joystick and come back to the start (loop closure). Watch
   `/map` in RViz.
5. Press Ctrl+C. The session manager saves the pose graph, the map (`.pgm` + `.yaml`) and a rosbag
   to `~/wheelchair_nav/maps/session_YYYYMMDD_HHMMSS/`.
6. Check the PGM the same way as a sim map (section 2.6). If it is bad, map again.

### 3.3 Navigate on the map

```bash
cd ~/wheelchair_nav && source setup.bash --skip
run_nav map_name:=$HOME/wheelchair_nav/maps/session_YYYYMMDD_HHMMSS/<name>.yaml
```

`run_nav` needs the lidar and all three cameras: AMCL and the costmaps read `/scan_fused`, the
lidar scan fused with the camera depth.

1. Localize. AMCL starts at the map origin, which is where the chair stood when mapping began.
   Put the chair back there, or set the pose with RViz **2D Pose Estimate** or with the
   `/initialpose` command from section 2.2 using the real x, y. Before the first goal, the scan
   outline must sit on the map walls.
2. Send a goal with RViz **2D Goal Pose**, the `ros2 action send_goal` command from section 2.3, or
   `go_to_location.py <name>` once the real places are in `locations.yaml` (section 3.4).

### 3.4 Named places

`locations.yaml` still holds the sim coordinates. On a real map those point at arbitrary spots, so
replace them before you use `go_to_location.py` on the chair. Until a save script exists, do it by
hand:

1. With `run_nav` running and the chair localized, drive it to the spot with the joystick.
2. Read its pose in the map frame:

   ```bash
   ros2 topic echo --once /amcl_pose --field pose.pose
   ```

   `x` and `y` are `position.x` and `position.y`. The yaw comes from the quaternion:
   `yaw = 2 * atan2(z, w)`.
3. Add the entry to `src/wheelchair_description/config/locations.yaml`. The script reads only this
   file, so keep one set of places per map and swap them along with the map.
4. Run `ros2 run wheelchair_description go_to_location.py <name>`.

Planned, waiting for a go-ahead: `save_location.py <name>` to do steps 2 and 3, and a `--file`
option on `go_to_location.py` so each map gets its own places file.

---

## 4. What changed for the sim

Every change is in `wheelchair_description`. The real-chair packages (`wheelchair_bringup`,
`wc_control`, `wheelchair_localization`, `wheelchair_navigation`) are untouched. The URDF edits sit
inside `<gazebo>` blocks, which only Gazebo reads, so the real chair's TF tree is unchanged.

| Change | File | Why |
|---|---|---|
| `nav2:=true map:=...` starts map_server, AMCL and the Nav2 servers with the real `nav2_params_3cam_v29.yaml`, with `nav2_sim.yaml` layered on top (sim time, raw `/scan` in place of `/scan_fused`) | `launch/gazebo_sim.launch.py`, `config/nav2_sim.yaml` | The sim runs the real chair's navigation config. |
| Castor friction 0.1 to 0.01 | `urdf/wc_gazebo.xacro` | The castors dragged, so the chair did not rotate as commanded and the odometry yaw did not match. |
| Lidar lifted 0.25 m | `urdf/wc_gazebo.xacro` | At 4 cm height, the chair's pitch made the beams hit the floor: phantom walls in the costmap. |
| Scan arc -1.72 to 2.80 rad, 260 samples | `urdf/wc_gazebo.xacro` | The chair's own frame blocked part of the scan (returns at 0.3-1.1 m), leaving blocked sectors around the chair. |
| `odom_axle_to_base.py`, EKF input `/wc_control/odom_base` | `scripts/`, `launch/gazebo_sim.launch.py` | diff_drive odometry is at the wheel-axle midpoint and `base_link` is 0.31984 m ahead. Without the shift, odom drifted while turning. |
| Map generated from the world file | `scripts/world_to_map.py`, `maps/small_house_world.*` | Exact map, no SLAM errors. |
| Named places with 1.3 m clearance | `config/locations.yaml` | See section 2.5. |
| `go_to_location.py` | `scripts/` | Named goals through `NavigateToPose`. It only uses the `navigate_to_pose` action, which `run_nav` also provides. |
| Install the two new scripts | `CMakeLists.txt` | So `ros2 run` finds them. |
| EKF fuses only forward speed from the wheels; heading comes from the IMU | `launch/gazebo_sim.launch.py` | Gazebo's wheel odometry drifted about 45° in heading per trip (castor slip), and fusing the wheels' x, y pushed AMCL more than 1 m off. After the change AMCL stayed within 0.06-0.34 m of the true pose. `ekf.yaml` is unchanged. |
| Three RGB-D cameras (front, left, right) on the RealSense topic names, bridged with `bridge_camera:=true` (off by default) | `urdf/wc_gazebo.xacro`, `launch/gazebo_sim.launch.py` | For the camera/VLM phase. Voice navigation does not use them. |
| `voice_nav.py` | `scripts/`, `CMakeLists.txt` | Voice to named places (section 2.7). |

Known gaps: the sim's straight-line odometry is 7% off in scale and not calibrated. The real chair
may have the same axle-vs-`base_link` odometry offset; not checked yet.

---

## 5. Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `Node not found`, `Failed to change state`, lifecycle nodes stuck `unconfigured` | Stale processes or a DDS shared-memory problem. Stop everything (section 2.4), check `ps`, use the UDP profile, relaunch. |
| Control loop "missed its desired rate ... 1-7 Hz", goals time out | Out of RAM/swap. Check `free -m`, close browsers and editors. |
| Goal FAILED, "Start occupied" | The chair or the goal is inside the inflated zone. Move the chair to open floor, or pick a goal with more clearance. |
| Goal FAILED, "no valid path found" | Goal in an unknown or occupied cell, or phantom walls. Check the map and the costmap in RViz. |
| `ros2 run ... No executable found` | Script not executable or missing from `install(PROGRAMS)`. `chmod +x`, rebuild, re-source. |
| Build skips `src/` or fails with "The source ... does not match the source ... used to generate cache" | A stray `CMakeLists.txt` at the repository root makes colcon treat the root as the only package. Build with `colcon build --base-paths src --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`. |
| `voice_nav.py`: `No module named 'faster_whisper'` | Activate the venv first: `source .venv-voice/bin/activate` (section 2.7). |
| Voice command heard but nothing happens | It must contain "go to", "take me to", "drive to" or similar plus a known place. Check `/voice/status` and the node's `heard:` log lines. |
| Gazebo segfault at start | Transient; relaunch. |
| Sim goal sent, chair does not move | No start pose, so AMCL has no pose. Publish `/initialpose` (section 2.2). |
| Real chair: `Device busy` | A previous run still holds the port. `pkill -9 -f ros; pkill -9 -f rviz; pkill -9 -f realsense; sleep 2` |
| Real chair: Arduino or lidar does not open | Check `ls -l /dev/wheelchair_arduino /dev/rplidar` and pass `port:=` / `lidar_port:=` (section 3.1). |
| Real chair: a camera never starts | Move it to a USB 3 port and check that `rs-enumerate-devices -s` lists its serial number. |
| Real chair: AMCL does not converge | `ros2 topic echo /map --once`, `ros2 topic hz /scan_fused`, reset the pose in RViz. |

More real-chair checks are in the Notes section of README.md.
