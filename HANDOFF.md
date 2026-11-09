## Odometry diagnosis/source fixes — 2026-11-09, chair kept stationary
- Full report: `/tmp/claude-1000/odomfix-report.md`; offline check: `python3 src/wheelchair_zupt/test/odom_selfcheck.py` (passes).
- Tape 3.045 m, wheel endpoint 3.092 m, filtered endpoint 4.021 m. Log reconstruction gives wheel **path** ~4.326 m but **endpoint** ~3.104 m: investigate heading substitution, not a guessed 0.77 distance multiplier. ZUPT's old slip branch changes encoder weight from 0.30 to 0.05.
- Source edits: wheel pose increments/rates and wheel heading by default in `zupt_node.py`; launch `gyro_blend_alpha` 0.30 -> 1.0; gyro bias/slip diagnostic-only by default; robust timestamps/freshness; hardware parser retains `rp-0.00` records. Active launch actually uses `wc_control_safe_v2.yaml` (odom TF disabled); included Madgwick also sets `publish_tf=False`.
- **Not deployed or live-tested.** No goals, initial pose, velocity publications, launch, process termination or colcon. DDS passive subscription was blocked by sandbox sockets, so live TF ownership remains unverified. Next controlled validation needs wheel/gyro heading signs/scale as well as <2% endpoint disagreement; log net wheel yaw is ~-184 degrees and cannot be assumed physically calibrated from tape distance alone. RPP/smoother and scan fusion untouched by this task.

# Handoff — 2026-11-09 (night, third real-chair session)

## 2026-11-09 session
- **`/scan_fused` speed fixed, in place in `scan_fusion_v9.py`:** it went from 4.6 Hz to 9.6 Hz over a 2-minute hold, with `/scan` at 9.9 Hz, so every hardware check now passes. Camera point clouds are now processed on one worker thread that keeps only the newest cloud per camera. Before, they were processed on the executor, where they starved the lidar callback. The node reads every 4th XYZ point straight from the buffer (`downsample` stride) and bins with a sort-based minimum. The fusion rule is unchanged: a camera reading wins only when it is closer than the lidar. An offline check (`/tmp/claude-1000/fusecheck.py`) showed identical bins. We decided to keep editing v9 in place, and the old "DO NOT EDIT" docstring line is gone. Details are in `fixes.md`.
- **Launch failure:** the first launch (`logs/real_nav_224040.log`) failed when the Arduino re-enumerated mid-launch after it was powered again. The log shows `Deactivating following hardware components as their write cycle resulted in an error: [ WheelchairSystem`, then no `odom` TF, then `controller_server` failed and bringup aborted. A clean relaunch (`logs/real_nav_224630.log`) came up fully.
- Voice run started at 23:07 with `voice_nav.py` and the real `locations.yaml` (`logs/voice_real_230705.log`). Results are pending.

# Handoff — 2026-11-08 (late evening, second real-chair session)

## Current goal
Mode 2 (voice + SLM, v31) on the real chair, on `maps/real_chair_indoor_2026-11-05_v2.yaml`. Voice goals report "Arrived at the dp lab" while the chair is physically about 2 m short, because the map pose runs ahead of the real chair. The straight-line test below points at `zupt_node` odometry as the cause. Finish that test (tape distance), fix the odometry scale, then rerun the 5-goal Mode 2 pass.

## Done this session (uncommitted)
- **`voice_nav.py`:** naming a new place while the chair is moving now cancels the current goal and starts the new one ("Changing to the <place>."), instead of "I am already moving. Say stop first." Saying "stop" with no goal running replies "Nothing to stop." Whenever a goal ends, or on an idle stop, the node publishes an empty `nav_msgs/Path` on `/plan`, so RViz no longer shows a stale path that looks like an active goal. A lock keeps `busy` and the pending place consistent across the mic thread, the topic callback and the run thread. This is not yet tested end to end: the sim test was cut short (see the incident below), and no voice command has been run on the real chair with this code.
- **Speed 0.40 m/s in all three params files:** RPP `desired_linear_vel` and the velocity smoother linear `max_velocity`, with accel unchanged. v29 and v31 went from 0.50 to 0.40. v30 on this branch went from 0.25 to 0.40, but this branch's v30 copy is older than the one on `voice-vlm-nav` (which runs 0.50), so carry the change there separately. Each header records the change. The real chair loaded it: `ros2 param get` showed 0.4 and `[0.4, 0.0, 0.8]`.

## What the 16:47–17:10 logs showed (first session)
The rider's four complaints, checked against `logs/voice_164714.log`, `logs/voice_170550.log` and `logs/real_nav_164344.log`:

1. **Short of DP lab.** Nav2 reported SUCCEEDED (`xy_goal_tolerance` 0.15), and RViz showed the chair at the goal with the scan shifted off the walls, so the cause is localization, not the goal checker.
2. **"Stop doesn't cancel."** Stop did cancel when a goal was running (17:04:35: "Goal canceled", "Cancellation was successful. Stopping the robot."). The other three stops came after the goal had already failed, so there was nothing to cancel, and RViz kept the last `/plan`. Fixed by the empty `/plan` above.
3. **"New place after stop ignored."** Every ignored place was spoken during an active goal and got "I am already moving". Fixed by the replace-goal change above.
4. **"Optical lab logged as DP lab."** Both "optical lab" phrases were rejected as "already moving", and the chair finished its DP lab goal, so "Arrived at the dp lab" was true.

6 of 13 goals failed: 1147 RPP "detected collision", 51 "Controller patience exceeded", 39 StopZone stops. The last one failed 6 cm from DP lab on "Transform data too old when converting from map to odom", with 1483 "timestamps differ" lines. That is a separate task once odometry is fixed.

## Localization facts found
- The real chair's AMCL loads `src/wheelchair_localization/config/amcl_fusion.yaml`, not the v31 file, so AMCL tuning in v31 has no effect there. Only one AMCL runs: `nav2_amcl_filtered` on `/scan_filtered` (lidar only), because `use_fused_scan` defaults to false. The `scan_topic: /scan_fused` in that yaml is overridden by the launch, and its header comments are stale.
- The odom→base_link TF comes from `zupt_node` (`/odometry/filtered`), which reads `/wc_control/odom` from `diff_drive_controller`. Wheel radius 0.1524 m and separation 0.565 m are set in `src/wheelchair_firmware/config/wheelchair_controllers.yaml:19-20` and repeated for `zupt_node` in `wheelchair_fusion_localization.launch.py:546-548`. All radius multipliers are 1.0, so the scale has never been calibrated.

## Straight-line test (17:38, unfinished)
`send_straight.py 4.0` sent one Nav2 goal 4 m along the chair's heading, and it SUCCEEDED. Distances moved:

| Source | Moved |
|---|---|
| `/wc_control/odom` (diff_drive wheels) | 3.129 m |
| `/odometry/filtered` (`zupt_node`, feeds the TF) | 4.295 m |
| map pose (AMCL) | 3.917 m |

`zupt_node` reads 37% more than the wheel controller on the same drive, which matches the map pose running ahead of the chair. **Missing: the tape-measured distance between the two floor marks.** With it, decide which source is right. If the tape reads about 3.1 m, `zupt_node` overcounts: read `src/wheelchair_zupt/wheelchair_zupt/zupt_node.py` to see how it integrates (wheel velocity from `odom_velocity_corrector`'s `/wheel_velocity`, IMU yaw), and fix it there. If the tape reads about 4.3 m, the wheel radius is too small. Repeat the test twice after the fix; the target is under 2% error.

The scripts used are in the old session scratchpad and will be lost, so re-create them if needed. `odom_probe.py before|after` prints x, y for both odom topics and the TF map→base_link, and the distance moved. `send_straight.py <m>` reads map→base_link and sends a NavigateToPose goal that many metres ahead with the same heading. `scan_rate.py <s>` counts `/scan`, `/scan_fused` and `/scan_filtered` with `qos_profile_sensor_data`.

## Hardware check results this session
Cameras passed (front D455 337122300107 on 3.2, the other two on 2.1). `/scan` 10.0 Hz and `/scan_filtered` 9.9–10 Hz passed. **`/scan_fused` failed at 6.0 Hz and then 4.2 Hz** (needs 8 or more) with load average about 4. The user chose to run the straight test anyway, since only the distance mattered. The fused-scan speed-up is still open.

## Incident: stray sim script drove the real chair
A sim test script (`run_test.sh`) kept running after the sim was stopped: `pkill -f run_test.sh` inside a compound command killed its own shell first, so the rest of the cleanup never ran. When the real stack came up, the script published `/initialpose` at (0, 0), started `voice_nav` with the sim places, and sent kitchen and gym goals to the real chair (17:36:49–17:37:44) until they were cancelled through `/navigate_to_pose/_action/cancel_goal`. The user then reset the start pose. Rule: before launching the real stack, check that no sim, `voice_nav` or test process is left, and stop processes by PID only, never with `pkill -f`.

## Left, in order
1. Tape distance from the 17:38 drive, then fix the odometry (`zupt_node` or the wheel radius), and repeat the 4 m test twice.
2. Real-chair test of the new `voice_nav.py`: redirect mid-drive, stop mid-drive, stop when idle (RViz path clears), then a new place.
3. ~~Speed up `scan_fusion_v9` until `/scan_fused` holds 8 Hz or more.~~ Done 2026-11-09: it holds 9.6 Hz.
4. Real-chair Mode 2 pass: 5 goals between `dp_lab` and `optical_lab` (13.92, 3.65), 4 of 5 required.
5. Diagnose the remaining goal failures (RPP collision detections, "Transform data too old").
6. Carry 0.40 m/s to v29 on `ros-nav` and v30 on `voice-vlm-nav`, then commit everything on this branch.
7. Earlier open items: bench-test the firmware, refuse weak voice matches ("doctor" → `dp_lab`), IMU "MADGWICK NOT TRACKING GYRO", Mode 3 exploration.
8. Fix the stale lines in AGENTS.md: the speed note says 0.25 m/s, and it says AMCL uses the fused scan.

## Branches
- `voice-slm-nav` (this checkout) is at `baf16fc`. Uncommitted: `voice_nav.py`, `scan_fusion_v9.py`, `fixes.md` and the v29, v30 and v31 params files.
- `ros-nav` is at `fd114ff` and `voice-vlm-nav` at `28adb56`, both pushed, neither has the 0.40 change.

## Real-chair run steps
```
rs-enumerate-devices | rg "Serial Number|Usb Type Descriptor"   # front D455 must show 3.x
cd ~/wheelchair_nav && source setup.bash --skip
ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py record_bag:=false   # defaults: v31, v3 BT, real_chair_indoor_2026-11-05_v2 map
# wait for all Nav2 nodes active [3], then set 2D Pose Estimate in RViz in open floor
source .venv-voice/bin/activate && ros2 run wheelchair_description voice_nav.py
```
`setup.bash` overwrites a shell variable named `S`, so use task-prefixed names in scripts that source it.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run. The firmware's 300 ms serial timeout stops the chair if the controller manager stalls.
