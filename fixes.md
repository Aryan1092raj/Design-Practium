# Fixes

This file lists the problems that stopped or degraded real-chair runs and how each one was fixed. Newest first. Each entry gives the symptom, the cause, the fix and how it was checked.

## 2026-11-11: Mode 3 base-frame reference and collision envelope (hardware validation pending)

**Problem.** The real wheel controller reports axle motion as `base_link`, although the URDF puts `base_link` 0.31984 m ahead of the driven-wheel axle. Recorded real turns moved the localized base about 0.23–0.36 m while wheel odometry barely translated. In simulation, correcting that translation exposed a second mismatch: DifferentialMotionModel injected excessive noise for sideways base swing. CAD contact telemetry also showed that the old +/-0.45 m footprint omitted the forward body and castors.

**Prepared fix.** `scripts/mode3_real.sh` passes `axle_to_base:=0.31984`. ZUPT still integrates axle motion and shifts its published pose, lateral twist, covariance and TF to the base frame. The launch selects AMCL OmniMotionModel for that shifted output, preserving the existing wheel-heading fusion and speed limits. Startup reads back those parameters and checks actual front RGB/aligned depth before starting voice. v30's footprint conservatively encloses all CAD collision shapes at x=-0.57..0.69 m, y=+/-0.50 m, with 0.03 m padding; stop/slowdown/approach zones enclose the same body. Mode 3 turn clearance is 1.32 m about the axle.

**Evidence.** Actual publication/covariance and original odometry regression checks passed offline. A live ZUPT on isolated ROS domain 64 passed 160 synthetic turn samples, including the offset pose and lateral velocity, and the real parameter gate passed there. No wheel commands were sent in that check. All 14 converted CAD collision shapes fit the final footprint and padded turn disc. The simulation chair/door smoke run and stop check are recorded in HANDOFF.md. Real lab routes, microphone and hardware remain to be checked in the user's first limited chair/door/box trial.

## 2026-11-09: left wheel encoder overcounts about 1.54× (FIXED 2026-11-10 by calibration; a rigid mount is still pending)

**Symptom.** On a 4 m straight goal the chair curves left until it stops at a wall, and it moves again only after it is pushed away. The tape reads about 3 m while odometry reads about 4 m. After every drive the lidar scan sits rotated off the map walls, and 2D Pose Estimate has to be set again. Earlier drives showed the same fault as a "37% / 32% odometry overcount" and as a left-right zigzag.

**Cause.** The left encoder counts about 1.54× the right on straight drives. Per-wheel distance integrated from `WheelchairInterface` log lines (`python3 scripts/encoder_ratio.py <log> <t0> <t1>`):

| Drive | Right | Left | L/R | Encoder yaw |
|---|---|---|---|---|
| 4 m test, tape 3.045 m | 3.41 m | 5.23 m | 1.53 | −184° |
| Optical lab voice goal | 11.77 m | 18.38 m | 1.56 | −671° |
| 4 m test, tape about 3 m | 3.20 m | 4.92 m | 1.54 | −175° |

The right encoder roughly matches the tape. Both wheels use `CPR = 10000` in `final_control.ino:43`. The firmware speed loop trusts the inflated left reading and slows the left motor, so the chair veers left. Odometry averages the two wheels, which overcounts distance, and it reads a large false right turn, which ruins the heading. The user suspects the left encoder is loose.

**Update 2026-11-10: still open after re-attaching both encoders.** On the ground, a 5 m drive at 0.4 m/s (`scripts/open_straight.py 5.0 0.4`) read L/R 1.04, yet the chair veered visibly left (AMCL +36° over 1.6 m) until the StopZone stopped it at an object on its left. L/R near 1.0 proves nothing here: the firmware PID (`final_control.ino:652-656`) forces both encoder speeds to the same target, so it only shows the PID is tracking. With the wheels lifted, the chair's left wheel made fewer real turns than its encoder counted, while the right wheel matched:

| Lifted run (0.15 m/s, stop on `/wc_control/odom`) | Right encoder / real | Left encoder / real | Left factor |
|---|---|---|---|
| 3 rev (2.871 m) | 2.98 / about 3.0 | 3.07 / about 2.4 | about 1.28 |
| 5 rev (4.788 m) | 5.00 / about 5.0 | 5.08 / about 3.8 | about 1.34 |

Real turns were counted by hand against a tape mark, so each factor is good to a few percent. The factor moved from 1.54 to 1.28 to 1.34, so a per-wheel CPR would not hold. Overcounting rules out a slipping coupling, which loses counts. Two causes fit: a drive ratio that depends on how the encoder is pressed or mounted (a roller or belt), or electrical noise adding edges on the left encoder lines (pins 2 and 3).

**Fix (applied 2026-11-10, calibration).** On the user's request, `final_control.ino` now uses `CPR_L = 13100` (10000 × 1.31, the mean of the two lifted runs) for the left wheel, while the right keeps `CPR = 10000`. It was flashed with `arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM1 src/wheelchair_firmware/firmware/final_control`. The calibration holds only while the mount stays put. Mounting the left encoder rigidly is still the lasting fix, and the factor must be measured again after any remount.

**Check.** Lifted 5-rev run after the reflash (`logs/real_nav_232421.log` 1794333390–430): the encoders read right 5.02 rev and left 5.06 rev, and the user counted about 5 real turns on both wheels. That passes. On the ground, `scripts/open_straight.py 5.0 0.4` (`logs/real_nav_232421.log` 1794333536–553) drove the full 5 m in 16 s with no collision-monitor stops. Odom read 5.09 m, AMCL 5.13 m, the AMCL yaw change was 3.3° (+36° before the fix), and the encoders read L/R 1.01 with −3° yaw. The user taped about 5 m with 4–5° of drift at most. That passes, from one run. Repeat it when convenient to confirm the factor holds.

**Check procedure.** Lifted 5-rev run, twice: `python3 scripts/open_straight.py 4.788 0.15 /wc_control/odom`, counting real turns against a tape mark. Pass: both wheels make 5.0 ± 0.1 turns. Then `scripts/straight_test.py 4.0` twice on the ground. Pass: the chair drives straight, AMCL yaw change stays under 5°, and filtered distance is within 2% of the tape.

## 2026-11-09: motors not powered gave an encoder stall fault

**Symptom.** Nav2 planned a path, but the chair didn't move. `cmd_vel` reached `/wc_control/cmd_vel` at 0.4 m/s, then `Failed to make progress`, then spin and backup recoveries failed.

**Cause.** The motor supply was off while the laptop side was up. The firmware drove above 40 output for 1.5 s with zero encoder counts and cut both wheels: `Arduino status: Encoder stall fault - stopping both wheels; send zero to rearm` (`final_control.ino:129`).

**Fix.** Power the motors before sending goals. The next goal sends zero first, which rearms the fault.

**Check.** After the motors were powered, the next voice goal drove with no stall fault.

## 2026-11-09: filtered odometry endpoint runs ahead; steering alternates (source fixes deployed; validation blocked by the left encoder)

**Symptom.** A 4 m Nav2 goal stopped after 3.045 m on the tape. `/wc_control/odom` endpoint moved 3.092 m; `/odometry/filtered` moved 4.021 m. These are endpoint displacements, not integrated path lengths.

**Evidence and cause.** Rounded wheel-speed lines in `logs/real_nav_224630.log` over ROS time 1794246557–1794246575 reconstruct about 4.326 m of wheel travel but only a 3.104 m endpoint displacement. The old `zupt_node` integrates wheel speed along a different, gyro-dominated heading; above the slip threshold it silently changes encoder weight from 0.30 to 0.05 despite the launch calling slip diagnostic-only. This can straighten an encoder arc and increase its endpoint without a velocity scale error. Exact live gyro and odom samples were unavailable: a passive DDS probe was blocked by the sandbox. A separate parser bug discarded complete wheel frames such as `rp-0.00,lp0.04,`, leaving stale feedback; there are 16 frame-rejection warnings near the start of this drive. The drive also shows 21 SlowdownZone/normal transitions and alternating steering commands.

**Source fix.** `zupt_node.py` now consumes wheel pose increments, derives signed rates from those increments, and defaults to wheel heading (`gyro_blend_alpha=1.0`, also set in `wheelchair_fusion_localization.launch.py`). This preserves the source trajectory up to a constant pose offset/rotation; gyro bias and slip remain diagnostic. Slip no longer switches the heading blend. Missing/stale gyro falls back to wheels; integer stamps reject duplicate/out-of-order samples without moving the baseline; long gaps retain measured displacement; slow motion is retained. `wheelchair_interface.cpp` accepts an optional minus sign in the outer wheel-record regex, preserving both fields when the right value is negative zero. Active controller and Madgwick launch configs already disable their odom/IMU TF publication; live TF ownership still needs verification. RPP, smoother, scan fusion, wheel dimensions and speed limits were not edited for this fix.

**Offline check.** `python3 src/wheelchair_zupt/test/odom_selfcheck.py` passed. A synthetic 3.092 m straight drive with deliberately inconsistent twist yields 3.092 m (0% source error); a 4 m curved path with a 3.365884 m endpoint stays at that endpoint, whereas the original slip equation gives 3.998334 m. Callback freshness, reverse/slow motion, yaw wrap, drops/gaps and the actual C++ parser regex also pass. Full evidence and outputs: `/tmp/claude-1000/odomfix-report.md`.

**Live result (2026-11-09 23:55).** The fix was built and launched, and `gyro_blend_alpha` read back 1.0. The straight test then showed that the larger fault is the left encoder (entry above). Its wheel yaw is wrong, so using wheel heading only (alpha 1.0) made the chair veer left. Revisit the alpha choice once the encoder is fixed.

**Earlier note.** At first nothing was built, installed or restarted. After separately authorized deployment, compare both odom endpoints and path lengths on repeated controlled runs (<2% source disagreement), check physical heading/gyro signs and scale, confirm TF frame-pair authorities, and observe steering and AMCL corrections. Wheel endpoint agreement alone does not validate physical yaw or eliminate all zigzag causes.

## 2026-11-09: `/scan_fused` too slow (4.6 Hz, needs 8 Hz or more)

**Symptom.** The hardware check failed: `/scan` ran at 9.6 Hz but `/scan_fused` only at 4.6 Hz, and 4.2–6.0 Hz the session before. A slow fused scan makes the collision monitor drop it as stale ("timestamps differ ... Ignoring the source"), and the chair then drives in a stop-and-go pattern with false "collision ahead" stops. Load average was only about 5 of 12, so the CPU had room to spare.

**Cause.** `scan_fusion_v9` (Python) processed every camera point cloud inside the ROS executor callback. It converted the full-resolution cloud with `read_points_numpy`, then strided and binned it with `np.minimum.at` and `np.add.at`. Three cameras sent clouds at about 6 Hz each, and each one took 100–860 ms while holding the GIL. That starved the lidar callback, which builds and publishes `/scan_fused`, and the node's stats showed 95–318 ms average latency per fused frame.

**Fix** (`src/wheelchair_localization/wheelchair_localization/scan_fusion_v9.py`, edited in place):

- The camera callback only stores the newest cloud for that camera and returns. One worker thread takes the latest cloud per camera and processes it. Clouds that arrive while the worker is busy replace the waiting one, so no backlog builds up.
- XYZ is read as a strided view of the message buffer (the existing `downsample` stride, 4), so the full cloud is never converted or copied. NaN handling still drops NaNs before the stride, as `skip_nans=True` did.
- Per-bin minima and the `min_points_per_bin` count check use a sort and `np.minimum.reduceat` instead of the unbuffered `ufunc.at` loops.
- Camera overlays are immutable snapshots, so the lidar callback reads them without a lock and never waits on camera work.

The fusion rule did not change: a camera reading replaces the lidar reading only when it is closer. Topics, bins and params are the same.

**Check.** An offline self-check (`/tmp/claude-1000/fusecheck.py`, 640x480 synthetic cloud) matched the old and new binning exactly, including dense, row-padded and big-endian clouds, and the lidar path stayed under 0.1 ms while a camera stalled. On the chair, after rebuilding `wheelchair_localization` and restarting only the `scan_fusion` node, `/scan_fused` held 9.6 Hz over 2 minutes with `/scan` at 9.9 Hz, and fused-frame latency dropped to about 30 ms. Every real-chair hardware check now passes.

## 2026-11-09: launch aborted after the Arduino was powered again

**Symptom.** `wheelchair_fusion_nav.launch.py` came up, but the local costmap logged `Timed out waiting for transform from base_link to odom ... Invalid frame ID "odom"` without end. Then `lifecycle_manager_navigation` logged `Failed to change state for node: controller_server` and `Aborting bringup`.

**Cause.** The Arduino Mega was powered again and re-enumerated while the stack was starting. Earlier in that session the hub had already moved the devices (`ttyACM1`/`ttyUSB1` back to `ttyACM0`/`ttyUSB0`). `ros2_control` logged `Deactivating following hardware components as their write cycle resulted in an error: [ WheelchairSystem`, so `diff_drive_controller` stopped and nothing fed `/wc_control/odom`. With no odometry, `zupt_node` published no `odom` TF.

**Fix.** Stop the whole launch by its process group and relaunch once all devices are plugged in and powered. The launch already uses the stable `/dev/serial/by-id/...` path for the Arduino, so the port number doesn't matter, but the device must not reset during startup. Rule: power the Arduino and plug in the lidar before launch, and relaunch if it resets.

**Check.** The relaunch (`logs/real_nav_224630.log`) brought all three lifecycle managers to "Managed nodes are active" with no write-cycle error.

## 2026-11-08: lidar stops publishing (`RPLidar health status : 2`)

**Symptom.** `/scan` stopped while `rplidar_node` kept running, or the node logged `RPLidar health status : 2` ("internal error").

**Cause.** The RPLidar firmware entered an error state. A node restart doesn't clear it.

**Fix.** Unplug the lidar for 10 s and plug it back in, then relaunch. The launch reaches the lidar through `/dev/rplidar`, which follows it to whichever `ttyUSB` it gets.

**Check.** After the power cycle, `/scan` came back at about 10 Hz.

**Seen again 2026-11-09 23:50.** At launch the node logged `Error, RPLidar internal error detected. Please reboot the device to retry.` and `rplidar_node` exited with code 255. With no scan, AMCL published no `map` TF, so `lifecycle_manager_navigation` failed to bring up its nodes. After a power cycle, the log showed `RPLidar health status : OK.` and all of Nav2 came up.
