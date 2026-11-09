# Fixes

This file lists the problems that stopped or degraded real-chair runs and how each one was fixed. Newest first. Each entry gives the symptom, the cause, the fix and how it was checked.

## 2026-11-09: filtered odometry endpoint runs ahead; steering alternates (source fixes, live test pending)

**Symptom.** A 4 m Nav2 goal stopped after 3.045 m on the tape. `/wc_control/odom` endpoint moved 3.092 m; `/odometry/filtered` moved 4.021 m. These are endpoint displacements, not integrated path lengths.

**Evidence and cause.** Rounded wheel-speed lines in `logs/real_nav_224630.log` over ROS time 1794246557–1794246575 reconstruct about 4.326 m of wheel travel but only a 3.104 m endpoint displacement. The old `zupt_node` integrates wheel speed along a different, gyro-dominated heading; above the slip threshold it silently changes encoder weight from 0.30 to 0.05 despite the launch calling slip diagnostic-only. This can straighten an encoder arc and increase its endpoint without a velocity scale error. Exact live gyro and odom samples were unavailable: a passive DDS probe was blocked by the sandbox. A separate parser bug discarded complete wheel frames such as `rp-0.00,lp0.04,`, leaving stale feedback; there are 16 frame-rejection warnings near the start of this drive. The drive also shows 21 SlowdownZone/normal transitions and alternating steering commands.

**Source fix.** `zupt_node.py` now consumes wheel pose increments, derives signed rates from those increments, and defaults to wheel heading (`gyro_blend_alpha=1.0`, also set in `wheelchair_fusion_localization.launch.py`). This preserves the source trajectory up to a constant pose offset/rotation; gyro bias and slip remain diagnostic. Slip no longer switches the heading blend. Missing/stale gyro falls back to wheels; integer stamps reject duplicate/out-of-order samples without moving the baseline; long gaps retain measured displacement; slow motion is retained. `wheelchair_interface.cpp` accepts an optional minus sign in the outer wheel-record regex, preserving both fields when the right value is negative zero. Active controller and Madgwick launch configs already disable their odom/IMU TF publication; live TF ownership still needs verification. RPP, smoother, scan fusion, wheel dimensions and speed limits were not edited for this fix.

**Offline check.** `python3 src/wheelchair_zupt/test/odom_selfcheck.py` passed. A synthetic 3.092 m straight drive with deliberately inconsistent twist yields 3.092 m (0% source error); a 4 m curved path with a 3.365884 m endpoint stays at that endpoint, whereas the original slip equation gives 3.998334 m. Callback freshness, reverse/slow motion, yaw wrap, drops/gaps and the actual C++ parser regex also pass. Full evidence and outputs: `/tmp/claude-1000/odomfix-report.md`.

**Pending.** Nothing was built, installed or restarted; the live stack retains the old behavior. After separately authorized deployment, compare both odom endpoints and path lengths on repeated controlled runs (<2% source disagreement), check physical heading/gyro signs and scale, confirm TF frame-pair authorities, and observe steering and AMCL corrections. Wheel endpoint agreement alone does not validate physical yaw or eliminate all zigzag causes.

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
