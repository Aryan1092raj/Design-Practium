# Autonomous Wheelchair Navigation

ROS 2 Jazzy navigation stack for a self-driving powered wheelchair. The chair maps a space, localizes on the saved map, and drives itself to a named place or a map coordinate. A Gazebo simulation of a small house lets you run the whole pipeline without hardware.

The chair has a differential-drive base driven by an Arduino, a 2D RPLidar S3, and three Intel RealSense depth cameras (the IMU is inside the front camera). It carries a passenger, so the stack favors safe, predictable motion over speed: velocity is capped at 0.25 m/s and 0.35 rad/s.

## How it works

Data flows up through five layers, and only the velocity command flows back down to the motors.

1. Hardware and drivers: Arduino motors and encoders, RPLidar, RealSense cameras, camera IMU.
2. Sensor conditioning: laser filter chain, IMU pipeline, LiDAR and camera scan fusion.
3. State estimation: a ZUPT/EKF node fuses wheel odometry and IMU into `/odometry/filtered` and the `odom -> base_link` transform.
4. World frame: SLAM Toolbox while mapping, or map_server plus AMCL while navigating, producing `map -> odom`.
5. Behavior: Nav2 planner, controller, behavior tree, and a velocity smoother that outputs `/cmd_vel`.

![System architecture](assets/architecture.png)

| Package | Responsibility |
|---|---|
| `wheelchair_bringup` | Top-level launch files |
| `wheelchair_navigation` | Nav2 parameters and behavior trees |
| `wheelchair_localization` | Scan fusion, AMCL, laser filter and SLAM configs |
| `wheelchair_description` | URDF, meshes, RViz configs, Gazebo simulation, named places |
| `wheelchair_firmware` | `ros2_control` hardware interface for the Arduino |
| `wheelchair_mapping` | SLAM Toolbox launch helpers |
| `wc_control` | Diff-drive config and IMU pipeline |
| `rplidar_ros` | RPLidar driver (vendored) |

## Build

Requires Ubuntu 24.04 and ROS 2 Jazzy with Nav2, SLAM Toolbox, robot_localization, ros2_control and the RealSense packages. The full `apt install` list is in the "Setup" section of `AUTONOMOUS_NAV.md`.

```bash
cd ~/wheelchair_nav
export PATH=/usr/bin:$PATH   # only if conda (base) is active
source setup.bash            # builds, sources ROS 2 and the workspace, defines run_nav / run_slam
```

Every other terminal only needs `cd ~/wheelchair_nav && source setup.bash --skip`.

## Run the simulation

The simulation needs 4-6 GB of free RAM. Close browsers first. When swap fills, Gazebo slows down and Nav2 loses nodes.

**One-time DDS setup.** Fast DDS shared memory can break Nav2 in the sim. Save the UDP-only profile from `AUTONOMOUS_NAV.md` as `~/fastdds_udp.xml`, then export it in every sim terminal:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml
```

**Terminal 1: start Gazebo and Nav2.**

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
  world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml
```

After about 30 seconds, check that Nav2 is up. All eight nodes should report `active [3]`.

```bash
for n in map_server amcl controller_server planner_server behavior_server bt_navigator velocity_smoother waypoint_follower; do
  printf '%s: ' $n; ros2 lifecycle get /$n; done
ros2 control list_controllers      # wc_control must be "active"
```

If `wc_control` shows `unconfigured`, its spawner lost a startup race. Fix it without restarting:

```bash
ros2 control set_controller_state wc_control inactive
ros2 control set_controller_state wc_control active
```

**Terminal 2: tell AMCL where the chair is.** The chair spawns at the world origin.

```bash
ros2 topic pub --once -w 1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
 "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}"
```

## Drive from one point to another

To a named place (`bedroom`, `living_room`, `kitchen`, defined in `src/wheelchair_description/config/locations.yaml`):

```bash
ros2 run wheelchair_description go_to_location.py kitchen
```

It prints `kitchen: SUCCEEDED` or `kitchen: FAILED` and exits 0 or 1. In the simulation, six goals across the three rooms succeeded, at 20-60 seconds per trip.

To any map coordinate (x, y in meters, map frame):

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
 "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 4.6, y: -2.0}, orientation: {w: 1.0}}}}"
```

Stop the simulation with Ctrl+C in terminal 1, then check for leftover `gz sim` or `parameter_bridge` processes before relaunching. Do not run `run_nav`, `run_slam` or `run_localization` while the simulation is up: they start with `pkill -9 -f ros2` and kill every ROS 2 process on the machine.

## Run on the physical chair

The full procedure, including the USB udev rules and port aliases, is in section 3 of `AUTONOMOUS_NAV.md`. In short:

1. Plug in the RPLidar, the Arduino, and the three cameras (USB 3 ports). Stop the simulation first.
2. Build the map: run `run_slam`, keep the chair still for 3 seconds, wait about 42 seconds for the session manager, drive every area slowly with the joystick and return to the start for loop closure, then press Ctrl+C. The map and a rosbag are saved to `maps/session_YYYYMMDD_HHMMSS/`.
3. Navigate: `run_nav map_name:=$HOME/wheelchair_nav/maps/session_YYYYMMDD_HHMMSS/<name>.yaml`. Place the chair at the map origin or set its pose with RViz 2D Pose Estimate, then send a goal with RViz 2D Goal Pose or the action command above.
4. Replace the simulation coordinates in `locations.yaml` with real places before using `go_to_location.py` on the chair.

The real-chair pipeline has not been re-tested since the simulation work. Run the first trips with an empty chair, in open space, with a hand on the emergency stop.

## Voice navigation

Say "take me to the kitchen" and the chair drives there on the preloaded map. `voice_nav.py` transcribes the command on the laptop with faster-whisper, matches it against the named places in `locations.yaml`, and sends the place to Nav2 as a goal. "Stop" cancels the trip at any point. The voice layer never commands the motors itself; Nav2 does.

```bash
source .venv-voice/bin/activate      # one-time setup: AUTONOMOUS_NAV.md, section 2.7
ros2 run wheelchair_description voice_nav.py
```

In the simulation it reached all three named places from typed commands, refused an unknown place and stopped on "stop". The microphone path and the physical chair have not been tested yet.

The work is split into phases, described in `docs/architecture.md`:

- **Phase 1, voice to named places (now).** The map is built once by driving the chair manually with `run_slam`; places are saved on that map and reached by voice. Next: a microphone test, a `save_location.py` script, one places file per map, then trials on the chair.
- **Phase 2, camera and VLM (next).** Go to an object the three RGB-D cameras can see, such as "the sofa". A prototype exists but is parked.
- **Phase 3, Jetson Orin Nano Super.** Move the stack from the laptop to the chair's own computer.

## Further documentation

- `docs/architecture.md`: system design with flowcharts, the voice pipeline, simulation results, and the phase 2 and 3 plans.
- `AUTONOMOUS_NAV.md`: full simulation and real-chair walkthrough, named places, voice navigation, making a simulation map, what changed for the simulation, and troubleshooting.
- `maps/`: saved maps, including the generated `small_house_world` map used by the simulation.
