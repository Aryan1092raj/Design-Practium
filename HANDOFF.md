# Handoff — 2026-11-10 (after the fourth real-chair session)

This codebase is yours: what it's for and how it's built are your call, and you'll keep building on it. The user is switching to Fable 5.2 for the next session.

## Current goal
The user wants to test Mode 3 (voice + VLM, zero-shot) next. Repeated real-chair faults have blocked every test so far. The left wheel encoder overcounted by a factor that kept changing (1.54, then 1.28, then 1.34). It is now calibrated with `CPR_L = 13100`, and on 2026-11-10 the chair drove 5 m straight on the ground. Mode 2 on the real chair is unblocked. A rigid mount would stop the factor from drifting again.

Mode 3 does not have to wait for the encoder. AGENTS.md requires every mode to pass in Gazebo `small_house` before the real chair, and the sim has no encoder fault. Mode 3 lives on `voice-vlm-nav` with `nav2_params_3cam_v30.yaml` (the `gazebo_sim.launch.py` default). That branch does not have the 0.40 m/s change yet (see Branches).

## Left encoder: what the 2026-11-10 tests showed
The user re-attached both encoders, then this session ran three tests (full numbers in `fixes.md`, top entry):

- **On the ground, 5 m at 0.4 m/s:** the encoders read L/R 1.04, but the chair veered visibly left (AMCL +36° over 1.6 m) until the collision monitor's StopZone stopped it at an object on its left. L/R ≈ 1.0 is meaningless as a check: the firmware PID forces both encoder speeds to the same target.
- **Wheels lifted, 3 and 5 encoder revolutions:** the chair's right wheel matched its encoder. The chair's left wheel made about 2.4 real turns for 3.07 counted, then about 3.8 for 5.08. That is a factor of about 1.28, then 1.34. Last session it was 1.54.

**Applied and passed on the bench:** at the user's request, the firmware now uses `CPR_L = 13100` for the left wheel (10000 × 1.31), and the right keeps `CPR = 10000`. It is flashed to the Mega. A lifted 5-rev run afterwards read right 5.02 and left 5.06 rev by encoder, and the user counted about 5 real turns on each wheel. **The ground test passed:** `scripts/open_straight.py 5.0 0.4` drove the full 5 m in 16 s with no collision stops. The AMCL yaw change was 3.3°, against +36° before the fix. Odom read 5.09 m and AMCL 5.13 m. The user taped about 5 m with 4–5° of drift at most. That is one run, so repeat it once to confirm.

Because the factor changed between mounts, the CPR value lasts only as long as the mount does. The encoder overcounts, and a slipping coupling would undercount, so the cause is either a drive ratio that depends on the mount (a roller or belt) or noise adding edges on the left encoder lines (Arduino pins 2 and 3). The user was asked how the left encoder is attached and hasn't answered yet.

A permanent fix is hardware work the user has to do:

1. Mount the left encoder rigidly, so its ratio to the wheel cannot change, and secure its cable, connector and ground.
2. Lifted 5-rev run, twice: `python3 scripts/open_straight.py 4.788 0.15 /wc_control/odom`, counting real turns against a tape mark on each tyre. Pass: both wheels make 5.0 ± 0.1 turns.
3. If both runs give the same factor within 1% but it isn't 1.0, the mount has a fixed ratio. Then a per-wheel CPR is the correct permanent calibration. Add `CPR_L` and `CPR_R` in `final_control.ino` and reflash, with the user's go-ahead.
4. On the ground: `python3 scripts/straight_test.py 4.0` twice after a 2D Pose Estimate. Pass: the chair drives straight, AMCL yaw change stays under 5°, and filtered distance is within 2% of the tape.

## Other faults seen 2026-11-10
- **USB hub drops.** The Arduino and the lidar fell off USB together twice (`ttyACM0`→`ttyACM1` and `ttyUSB0`→`ttyUSB1` at the same moment). The first drop hit during bringup, and the second while the chair was being lifted. Each one gives `write cycle resulted in an error: [ WheelchairSystem ]` and needs a relaunch. Both devices share the hub, so suspect hub power. Put the Arduino and the lidar on separate laptop ports, or use a powered hub.
- **Slow scan on the first launch:** `/scan` 3.8 Hz and `/scan_fused` 3.9 Hz, with the lidar reporting health OK at a 10 Hz setting. Unplugging the lidar for 10 s and relaunching restored 9.9 / 9.6 Hz.
- **IMU orientation is unreliable.** During the 36° veer, `imu_wheelchair_republisher` yaw moved about 1°. Its roll jumped to about 40°, then later fell back to −2.9°. The ZUPT `gyro_blend_alpha` choice can't be judged until this and the encoder are fixed.
- **The planner can't plan 5 m from (3.99, 1.36, 0.243 rad) on the real map.** The map has a wall 0.1 m from the goal. `straight_test.py` works there up to about 4.0 m.
- **The lidar is mounted backwards:** the `laser` frame sits at x 0.47, y 0.12 in `base_link` with yaw 180°. Any scan probe has to transform into `base_link` first. Probing laser angle 0 looks behind the chair.
- The latched `/local_costmap/costmap` message is the startup map, and only `costmap_updates` stay current. Don't judge live obstacles from the full message alone.

## Scripts (`scripts/`)
- `straight_test.py N`: Nav2 goal N m ahead, then prints the distance moved per odom source.
- `open_straight.py N SPEED [ODOM_TOPIC]`: drives straight by publishing to `/cmd_vel_nav`, so the velocity smoother and the collision monitor stay in the path. It stops at N m minus braking distance on the given odom topic, or after 40 s. With the wheels lifted, pass `/wc_control/odom`.
- `encoder_ratio.py LOG T0 T1`: per-wheel distance from the `WheelchairInterface` lines of a launch log.
- `scan_rate.py SECS`: `/scan` and `/scan_fused` rates with `qos_profile_sensor_data`, for the 2-minute hardware check.
- Launch logs go to `logs/real_nav_HHMMSS.log` (tee in the launch command below). Today's logs are `real_nav_224256.log` (ground runs) and `real_nav_231329.log` (lifted runs).

## Next session, in order
1. Mode 3 in sim, which doesn't need the chair: on `voice-vlm-nav`, carry the 0.40 m/s change into v30, then run the VLM zero-shot path in Gazebo `small_house` (`scripts/vlm_server.sh`, `vlm_nav.py`). Pass: 4 of 5 objects reached.
2. The encoder is now calibrated and drives straight. Repeat the 5 m ground run once (`scripts/open_straight.py 5.0 0.4`), and re-measure the factor if the user ever remounts the encoder. Then do the Mode 2 real-chair steps from the previous handoff. Those are voice tests, then 5 goals between `dp_lab` and `optical_lab` (13.92, 3.65), 4 of 5 required.
3. Leftover items: RPP `detected collision ahead!` near walls, `SlowdownZone` toggling, "Transform data too old", the IMU fault, refusing weak voice matches ("doctor" → `dp_lab`), and stale AGENTS.md lines (the speed note says 0.25 m/s, and it says AMCL uses the fused scan).

## Done 2026-11-09 (all committed on `voice-slm-nav`)
- `/scan_fused` went from 4.6 to 9.6–9.9 Hz (`scan_fusion_v9.py`). Camera clouds now go to one worker thread that keeps only the newest cloud per camera.
- ZUPT odometry (`zupt_node.py`, `gyro_blend_alpha` 1.0) integrates wheel pose increments. Offline check: `python3 src/wheelchair_zupt/test/odom_selfcheck.py`.
- Encoder parser (`wheelchair_interface.cpp`) accepts frames like `rp-0.00,lp0.04,`.
- `voice_nav.py`: naming a new place mid-drive replaces the goal, and stop clears the RViz path.
- Speed 0.40 m/s in v29, v30 and v31 on `voice-slm-nav`.

## Real-chair problems seen 2026-11-09 (details in `fixes.md`)
- Arduino powered again mid-launch → `write cycle resulted in an error: [ WheelchairSystem` → no `odom` TF → bringup aborts. Relaunch.
- Motors without power → `Encoder stall fault - stopping both wheels; send zero to rearm`. Turn on motor power; the next goal rearms the fault.
- Lidar `Error, RPLidar internal error detected.` → `rplidar_node` exits 255 → no `map` TF. Power-cycle the lidar for 10 s.

## Localization facts
- The real chair's AMCL loads `src/wheelchair_localization/config/amcl_fusion.yaml`, not the v31 file. Only one AMCL runs: `nav2_amcl_filtered` on `/scan_filtered` (lidar only), because `use_fused_scan` defaults to false.
- The odom→base_link TF comes only from `zupt_node`. The active controller config is `src/wc_control/config/wc_control_safe_v2.yaml` (`enable_odom_tf: false`). Wheel radius 0.1524 m and separation 0.565 m, with multipliers 1.0.
- The firmware runs a closed-loop PID per wheel on encoder speed (`final_control.ino:652-656`), so a bad encoder makes the chair itself drive wrong, not just the odometry.

## Process rules (from incidents)
- Before launching the real stack, check that no sim, `voice_nav` or test process is left. Stop processes by PID or process group only, never with `pkill -f`.
- Launch the stack in a persistent `hc` terminal. `nohup setsid` launches from an agent shell have died without a logged cause.
- The user can't see the chair's state from the terminal. Ask which side they mean: "right" has meant the user's right while facing the chair, which is the chair's left.

## Branches
- `voice-slm-nav` (this checkout): 2026-11-09 work is committed, not pushed. Today's changes are uncommitted: `final_control.ino` (`CPR_L`, flashed), `fixes.md`, `HANDOFF.md`, `scripts/open_straight.py` and `scripts/scan_rate.py`.
- `ros-nav` is at `fd114ff` and `voice-vlm-nav` at `28adb56`, both pushed, neither has the 0.40 change.

## Real-chair run steps
```
rs-enumerate-devices | rg "Serial Number|Usb Type Descriptor"   # front D455 must show 3.x
cd ~/wheelchair_nav && source setup.bash --skip
hc run --bg --outlive-parent --name chairstack --cwd ~/wheelchair_nav -- bash -ic 'source setup.bash --skip && ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py record_bag:=false 2>&1 | tee logs/real_nav_$(date +%H%M%S).log'
# wait for all 3 "Managed nodes are active", run python3 scripts/scan_rate.py 120 (/scan ~10 Hz, /scan_fused >= 8 Hz), then 2D Pose Estimate in RViz in open floor
source .venv-voice/bin/activate && ros2 run wheelchair_description voice_nav.py
```
`setup.bash` overwrites a shell variable named `S`, so use task-prefixed names in scripts that source it. Right after launch the `map` TF can be missing for a few seconds. Wait before running a test script.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run. The firmware's 300 ms serial timeout stops the chair if the controller manager stalls. Until the left encoder is fixed, the chair veers left on any drive, so don't run goals near walls or people.
