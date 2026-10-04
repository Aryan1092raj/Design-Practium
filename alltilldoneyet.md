# Autonomous Wheelchair — Full Status Handover

**Read this first if you are a new agent picking up this project.**
Last updated: 2026-10-01. Repo: `/home/aryan-raj/wheelchair_nav` (branch `main`).

---

## 1. What the task is

**Goal:** make the powered wheelchair drive itself, autonomously, in a simulated
indoor environment (Gazebo), using the same navigation stack it uses on the real robot.

Two things were asked for, understood as follows:

1. **"Just commit these"** — commit the pre-existing config-archival cleanup.
   DONE (commit `683bb63`).
2. **"Run the wheelchair autonomously in a simulated indoor environment"** — wire the
   real Nav2 stack (AMCL + planner + controller + behaviour trees) into the Gazebo sim
   so the chair drives to goals with no human input.
   IN PROGRESS — the stack boots and accepts goals, but **one planner bug still
   blocks all successful plans**. See section 6.

**Non-goal for now:** the VLM / voice / semantic-goal work described in
`/home/aryan-raj/College/DP/*.md`. That is a *later* layer. Per the project's own
architecture rules, the language model must **never** command the motors — only
Nav2 plus the safety layer may.

---

## 2. Where everything lives

| What | Path |
|---|---|
| Workspace root | `/home/aryan-raj/wheelchair_nav` |
| ROS 2 distro | Jazzy (`/opt/ros/jazzy`) |
| Build / env | `source install/setup.bash` |
| Main sim launch (main file for this work) | `src/wheelchair_description/launch/gazebo_sim.launch.py` |
| Low-level sim launch | `src/wheelchair_description/launch/gazebo_launch.py` |
| **New** Nav2 sim overrides | `src/wheelchair_description/config/nav2_sim.yaml` |
| Existing SLAM sim overrides | `src/wheelchair_description/config/slam_sim.yaml` |
| Real-robot Nav2 config (DO NOT edit) | `src/wheelchair_navigation/config/nav2_params_3cam_v29.yaml` |
| Behaviour tree (shared sim + robot) | `src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v3.xml` |
| Generated sim map | `maps/small_house_sim.yaml` (+ `.pgm`) |
| Gazebo worlds | `src/wheelchair_description/worlds/{empty,small_house,small_warehouse}.world` |
| Furniture models (git-ignored, 95 MB) | `src/wheelchair_description/models/` |
| Archived/superseded configs | `src/*/archive/` |
| Source ZIPs for models/worlds/photos | `/home/aryan-raj/Dp/Simulations/*.zip` |
| Design docs / papers (context only) | `/home/aryan-raj/College/DP/` |

---

## 3. How the system works (so you can debug it)

### 3.1 Baseline sim (this part already worked)

`gazebo_sim.launch.py` includes `gazebo_launch.py`, which starts Gazebo, publishes the
robot description, spawns the wheelchair and bridges Gazebo <-> ROS.

`gazebo_sim.launch.py` adds what plain `gazebo_launch.py` lacks — without these the
robot spawns but **never moves**, because `diff_drive_controller` inside Gazebo is
never activated:

- `joint_state_broadcaster` + `wc_control` spawners (activate the controllers)
- `imu_out_to_imu` — republishes Gazebo `/imu/out` -> `/imu`, `frame_id=base_link`
- `ekf_node` — fuses `/wc_control/odom` + `/imu`, publishes **`odom -> base_link`**
  (deliberately not published by `wc_control`, so robot and sim share one TF owner)
- LiDAR bridge `/scan` <- Gazebo GPU lidar
- optional `teleop` bridge: `/cmd_vel` (Twist) -> `/wc_control/cmd_vel` (TwistStamped)
- optional `slam` (slam_toolbox) for building a map

**Verified working:** both controllers reach `active`; `/clock`, `/scan`, `/imu`,
`/wc_control/odom`, `/odometry/filtered` all publish; the `odom -> base_link -> lidar`
TF chain resolves; publishing `0.15 m/s` on `/cmd_vel` moved the chair **0.63 m**.

### 3.2 Velocity command chain (critical to understand)

`diff_drive_controller` runs *inside* Gazebo and consumes **`/wc_control/cmd_vel`,
type `TwistStamped`** (see `wheelchair_controllers.yaml`: `use_stamped_vel: true`,
`cmd_vel_topic: /wc_control/cmd_vel`). A plain `Twist` will not move the wheels.

Anything producing plain `Twist` must pass through `scripts/twist_stamped_teleop.py`,
which also clamps to safe limits (0.30 m/s fwd, -0.15 rev, 0.50 rad/s).

Chain used by Nav2 (mirrors the real robot):

```
planner_server / controller_server / behavior_server
        -> /cmd_vel_nav
              -> velocity_smoother --/cmd_vel_smoothed--> /cmd_vel
                    -> [collision_monitor] --/cmd_vel_safe--> twist_stamped_teleop
                          -> /wc_control/cmd_vel (TwistStamped) -> Gazebo diff_drive
```

**Recovery behaviours must be remapped to `/cmd_vel_nav`** — otherwise `BackUp` and
`Spin` publish straight to `/cmd_vel` and bypass the smoother. This bug is called out
in comments in `wheelchair_fusion_nav.launch.py`; do not reintroduce it.

---

## 4. What is DONE

### 4.1 Config archival (committed — `683bb63`)

- Moved **66** superseded configs into `src/wheelchair_navigation/archive/` and
  `src/wheelchair_localization/archive/` via `git mv` (history preserved; all recorded
  as renames). Active sets: 5 Nav2 params, 4 behaviour trees, 12 localization configs.
- **Fixed a real install bug:** `wheelchair_localization/setup.py` installed 26
  now-archived files while **omitting two live ones**
  (`slam_toolbox_hospital_lidar_v4.yaml`, `laser_filter_hospital_v2.yaml`), so
  `hospital_mode:=true` would fail on a fresh build.
- `wheelchair_navigation/CMakeLists.txt` now excludes `archive/` from the install.
- Removed a stale comment block advertising 10 non-existent `slam_toolbox_v14*`
  configs; repointed legacy `scan_fusion_v*` docstrings at live configs.
- Added `archive/README.md` in each package (active list + rollback steps).
- Verified: all 10 packages build; zero dangling references; all 14 launch files parse.

### 4.2 Sim environment (already present, verified)

- `models/`, `worlds/`, `photos/` are **byte-identical** to the ZIPs in
  `/home/aryan-raj/Dp/Simulations/`. No extraction was needed.
- The teammate's `gazebo_launch.py` is **identical** to the repo copy — no change needed.
- All sim deps verified installed (`ros_gz_sim`, `ros_gz_bridge`, `gz_ros2_control`,
  `slam_toolbox`, `robot_localization`, diff-drive, teleop, rviz2).
  GPU: RTX 4050. `DISPLAY=:1`.

### 4.3 Nav2 autonomy layer (written, builds, boots — NOT working end-to-end)

**New `src/wheelchair_description/config/nav2_sim.yaml`** — layered *after*
`nav2_params_3cam_v29.yaml` so the real-robot config is never edited. Overrides only:

1. `use_sim_time: true` (the real config hardcodes `false` in 11 places)
2. laser topic `/scan` instead of `/scan_fused` (the 3-camera fusion pipeline does not
   exist in Gazebo)
3. `inflation_radius` (see section 6)

**Modified `src/wheelchair_description/launch/gazebo_sim.launch.py`** — added a
`nav2:=true` mode launching `map_server`, `amcl`, `controller_server`,
`smoother_server`, `planner_server`, `behavior_server`, `bt_navigator`,
`velocity_smoother`, `waypoint_follower`, `collision_monitor` + lifecycle manager, on
a `TimerAction(period=18.0)` so the map and odometry exist first.

New launch args: `nav2`, `map`, `nav2_params_file`, `bt_xml`, `use_collision_monitor`.

Behaviour:

- `slam` and `nav2` are mutually exclusive — both publish `map -> odom`, so
  `slam_ready` also requires `nav2 != true`.
- `use_collision_monitor` defaults to **`false`**, matching
  `wheelchair_fusion_nav.launch.py`. When off the wheel bridge reads `/cmd_vel`; when
  on it reads `/cmd_vel_safe`. See section 7.1 for why.

**Verified in simulation:**

- `amcl`, `controller_server`, `planner_server`, `bt_navigator`, `velocity_smoother`
  all reach lifecycle state **`active [3]`**.
- All `/global_costmap/*` and `/local_costmap/*` topics exist and publish.
- AMCL localizes (after an explicit `/initialpose` reset).
- A goal is **accepted** by `bt_navigator` ("Goal accepted with ID: ..."), proving the
  behaviour tree loads and executes.

---

## 5. How to run it

### 5.1 Build

```bash
cd /home/aryan-raj/wheelchair_nav
export PATH=/usr/bin:$PATH        # MANDATORY, see section 8
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

### 5.2 Stage 1 — build a map (works)

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
    world_name:=small_house use_rviz:=false slam:=true teleop:=true
```

Drive with teleop, then save:

```bash
ros2 run nav2_map_server map_saver_cli -f /home/aryan-raj/wheelchair_nav/maps/small_house_sim
```

(`maps/small_house_sim.yaml` + `.pgm` already exist — 831x564 @ 0.02 m, built this way.)

### 5.3 Stage 2 — autonomous navigation (blocked, see section 6)

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py \
    world_name:=small_house use_rviz:=false \
    nav2:=true map:=$HOME/wheelchair_nav/maps/small_house_sim.yaml
```

Wait ~30 s for Nav2 to activate, then send a goal:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.5, y: 0.5}, orientation: {w: 1.0}}}}"
```

### 5.4 Kill stale sim processes (do this between runs)

```bash
for p in $(ps -eo pid,cmd | grep -E '[g]z sim|[r]os2 launch' | awk '{print $1}'); do kill -9 $p; done
rm -rf /dev/shm/fastrtps* ~/.gz/transport
```

**Never use `pkill -f 'gz sim'`** — the pattern matches the shell running it and kills
your own command part-way through (this cost real debugging time). Use the loop above.

---

## 6. THE REMAINING BLOCKER — "Start occupied"

> **RESOLVED 2026-10-01.** Two real causes, neither the inflation radius:
> 1. `nav2_sim.yaml` nested costmap params once; the nodes are
>    `/local_costmap/local_costmap` and `/global_costmap/global_costmap`, so the
>    overrides never applied. Now nested twice. (`ros2 param list /controller_server`
>    showing no costmap params was a red herring: costmaps are separate nodes.)
> 2. `maps/small_house_sim.pgm` had 11 occupied pixels within 0.5 m of the spawn point
>    (the chair's own lidar returns from mapping). Inscribed radius is 0.38 m, so the
>    start cell was INSCRIBED. `maps/small_house_sim_clean.yaml` has them cleared; use
>    it as `map:=`. The "0.612 m inscribed radius" analysis below is wrong; the real
>    config's 0.38 m comment is correct.
> Verified: two `NavigateToPose` goals, (2.5, 0.5) and (0.5, -0.5), both `SUCCEEDED`.

**This is the single thing left to fix.**

### Symptom

The behaviour tree accepts the goal and runs, then every plan attempt fails:

```
[planner_server]: GridBased plugin failed to plan from (0.01, 0.00) to (2.50, 0.50): "Start occupied"
[behavior_server]: Collision Ahead - Exiting DriveOnHeading / spin failed
Goal finished with status: ABORTED
```

`ros2 topic echo /amcl_pose` stays at `(0.01, 0.00)` — the chair does not move.
Zero successful plans so far.

### Root cause (diagnosed, high confidence)

`SmacPlanner2D` refuses to plan when `inflation_radius < inscribed_radius`, because
the robot's own centre cell gets inflated to lethal.

From `nav2_params_3cam_v29.yaml` the footprint is
`[[0.45,0.35],[0.45,-0.35],[-0.45,-0.35],[-0.45,0.35]]` — half-extents
**0.45 x 0.35 m** (a 0.9 x 0.7 m full footprint) — with `footprint_padding: 0.03`:

```
inscribed_radius = sqrt(0.48^2 + 0.38^2) = 0.612 m
```

The config sets `inflation_radius: 0.40` and its header comment asserts
*"MUST be >= inscribed_radius (0.38m)"*. **That comment is wrong** — the real value
from that footprint is **0.612 m**, not 0.38 m. So `0.40 < 0.612` => "Start occupied".

> **This is a latent bug in the REAL-ROBOT config too**, not just the sim. It has
> probably never fired on the robot only because the real stack has never been asked
> to plan from a start pose with the current footprint. Worth fixing on the robot as
> well — see section 9.

### What was tried

- Added `inflation_layer.inflation_radius: 0.65` to **both** costmaps in
  `nav2_sim.yaml` (0.65 > 0.612, with margin).
- Restarted, reset AMCL to the true spawn pose via `/initialpose`, re-sent the goal.
- **Still failing.** Because of the following.

### The open question — highest-value next step

`ros2 param list /controller_server | grep local_costmap` returns **0 parameters**,
and the same for `global_costmap` on `planner_server` (only 31 planner-plugin params
are declared). That strongly suggests **the costmap parameters are not being loaded
at all**, so Nav2 falls back to built-in defaults — where the default
`inflation_radius` is **0.55**, still below 0.612, which also yields "Start occupied".

Confirm this first:

```bash
ros2 param list /controller_server | grep local_costmap   # expect many, got 0
ros2 param list /planner_server   | grep global_costmap   # expect many, got 0
```

If confirmed, the fix is **not** another YAML value but getting the costmap config to
actually reach the nodes. Check in this order:

1. Is the costmap section nested under the right node? In `nav2_params_3cam_v29.yaml`
   the sections are top-level `local_costmap:` / `global_costmap:` with
   `ros__parameters:` — correct for Nav2.
2. Are **both** param files reaching the node? `nav2_params_file` + `nav2_sim.yaml`
   are passed as a list; confirm no earlier entry shadows them.
3. **Suspect the clock first** (section 7.2) — a costmap that never gets a valid
   current transform reports `Costmap timed out waiting for update` (seen in the log)
   and can appear unconfigured. Fixing the clock may make the params declare normally.
4. Only then re-check whether `inflation_radius: 0.65` takes effect.

**Fallback if params still will not load:** set footprint/inflation inline in
`nav2_sim.yaml` as a top-level override and re-test; or temporarily shrink the
footprint (e.g. half-extents `0.30 x 0.25`, inscribed ~0.43 m) which both fixes
"Start occupied" and lets the chair through the house's doorways.

### Success criterion for this task

`ros2 topic echo /amcl_pose` converges to the goal (~`x: 2.5, y: 0.5`) and the action
logs report **`SUCCEEDED`** — with no human input beyond sending the goal.

---

## 7. Known secondary issues (not blockers)

1. **`collision_monitor` cannot configure.** Its polygon config in
   `nav2_params_3cam_v29.yaml` fails with
   `parameter 'StopZone.points' has invalid type ... is of type {string}, setting it to
   {double_array} is not allowed`. The YAML points *are* valid floats, so this is a
   latent config bug that never surfaced because the real robot runs
   `use_collision_monitor:=false` by default. It is therefore **off by default in the
   sim too**, and the wheel bridge routes `/cmd_vel` -> wheels instead of
   `/cmd_vel_safe`. Left as-is deliberately: do not put an untested safety layer in
   the critical path. Fix separately if the safety layer is wanted.

2. **Continuous "Detected jump back in time. Clearing TF buffer"** (~10 000 warnings per
   run) from `tf2_buffer` and `robot_state_publisher`, plus transient
   `Extrapolation Error looking up target frame`. `/clock` itself samples as strictly
   monotonic (113 -> 116 -> 119) and only one Gazebo instance runs, so this is **not**
   a dual-clock problem. Likely a sim-time vs. TF-buffer interaction at startup. It is
   noisy, degrades the costmaps, and is a strong suspect for section 6.

3. **`ekf_filter_node: Failed to meet update rate`** — transient during startup under
   Gazebo load; the EKF recovers and publishes correctly.

4. **`imu_out_to_imu` traceback on shutdown** — a double `rcl_shutdown()` during
   teardown. Harmless, only on launch teardown.

5. **The generated map is only ~32 % explored** (146 618 free / 4389 occupied /
   317 677 unknown cells). AMCL drifted ~1.8 m before being reset. A more thorough
   exploration run would give a more reliable map.

---

## 8. Environment gotcha that will waste your time if you miss it

A **Conda Python is active** (`/home/aryan-raj/miniconda3/bin/python3`) and shadows
the system Python. Every `ament_cmake` package then fails with:

```
ModuleNotFoundError: No module named 'catkin_pkg'
CMake Error at .../ament_package_xml.cmake:95
```

This is **pre-existing and unrelated** to this work. Always build with:

```bash
export PATH=/usr/bin:$PATH
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

(Already documented in the repo `README.md` under **Build**.)

Harmless noise you can ignore: `RTPS_TRANSPORT_SHM ... Failed init_port
fastrtps_port7000` lines — stale shared-memory segments from killed processes. They
clear after the section 5.4 cleanup. Do *not* let them convince you the sim is broken;
check `/clock` and the controllers before concluding anything.

---

## 9. Suggested order of work

1. **Prove/disprove that costmap params load** (section 6, the open question).
   Everything else depends on this answer.
2. Fix the clock / time-jump behaviour (section 7.2) if the params look unloaded — the
   two symptoms are plausibly the same root cause.
3. Re-run Stage 2. Target: `SUCCEEDED`, chair at the goal.
4. Once autonomous nav works in sim, **fix the same `inflation_radius` bug in
   `nav2_params_3cam_v29.yaml`** for the real robot (section 6) — the sim override was
   only ever a diagnostic.
5. Only then consider the VLM/voice layer from `/home/aryan-raj/College/DP/`, keeping
   the hard safety boundary: *the model may propose a goal; only Nav2 + the safety
   layer may move the chair.*

---

## 10. Project conventions you must follow

- **Never edit config files in place.** Copy forward to a new version, repoint the
  launch default, keep the old one in `archive/`.
- Use `git mv` when archiving so history is preserved.
- Real-robot Nav2/tuning values live in
  `wheelchair_navigation/config/nav2_params_3cam_v29.yaml`; simulation overrides go in
  `wheelchair_description/config/*_sim.yaml`, layered *after* it. Never edit the robot
  config to make the sim work.
- `position_feedback` must stay `false` in the diff-drive config (Arduino sends at
  20 Hz, controller runs at 100 Hz; differentiating under-reports speed by ~35 %).
- diff-drive `publish_odom_tf` must stay `false` — the EKF owns `odom -> base_link`.

---

## 11. Current git state

- Branch `main`, upstream `origin` = `https://github.com/siddharthtiwari1/wheelchair_nav.git`
- `683bb63` — *Archive superseded configs; add Gazebo simulation harness* (config
  archival, install-bug fix, sim harness, worlds, udev rules).
- **Uncommitted work (the Nav2 autonomy layer — commit once section 6 is fixed):**
  - `M src/wheelchair_description/launch/gazebo_sim.launch.py`
  - `?? src/wheelchair_description/config/nav2_sim.yaml`
  - `?? maps/small_house_sim.yaml`, `?? maps/small_house_sim.pgm`
  - `?? alltilldoneyet.md` (this file)
- `models/` (95 MB) and `photos/` (22 MB) are deliberately `.gitignore`d — they stay
  in the working tree for running the sim but never enter git history.
