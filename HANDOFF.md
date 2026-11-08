# Handoff — 2026-11-08 (evening, after the first real-chair session)

## Current goal
Mode 2 (voice + SLM, v31) on the real chair, on `maps/real_chair_indoor_2026-11-05_v2.yaml`. The user wants a clean video: say "take me to optical lab" and the chair drives there without stalling. The real-chair session showed the stack works end to end, but the fused scan is too slow and stale, which causes stop-and-go and failed goals. Fix that first, then the video, then Mode 3.

## Real-chair session result (2026-11-08, 16:20–17:11)
- Firmware `final_control.ino` (commit `31f94a3`) is flashed to the Mega on `/dev/ttyACM0`. It was never bench-tested; the user skipped it. The Arduino link worked all session (`[WHEEL] rp..,lp..` at about 20 Hz).
- Voice works: whisper on CUDA, `qwen3:4b` on Ollama, laptop mic (ALC256). Background speech mostly starts nothing, "stop" cancels a running goal, and "I am already moving. Say stop first." blocks a new goal while one runs. False match: "go to the doctor" started a `dp_lab` goal. Whisper also repeatedly invented "I'm going to go to the bathroom" from noise.
- Goals: `dp_lab` reached 3 times (one of them started at dp lab, so not a real test). `optical_lab` reached 0 times: it stalled about 0.3 m short at (13.65, 4.39). Several goals failed on "Start occupied", a blocked route, or stale scans. Mode 2 does not pass yet on the real chair (needs 4 of 5).
- Logs from the session are in `logs/` (untracked): `real_nav_*.log` for the stack, `voice_*.log` for the voice node.

## Problems found and their state
1. **Lidar internal error (fixed by a power cycle).** At about 16:24 `/scan` went to 0 Hz with `rplidar_node` still alive. The next launch printed `RPLidar health status : 2` and "Error, RPLidar internal error detected. Please reboot the device to retry." and the node died. Unplugging the lidar for 10 s cleared it (health 0). If `/scan` stops again, power-cycle the lidar before anything else.
2. **Cameras on USB 2 (partly fixed).** `rs-enumerate-devices` showed two of three cameras at `Usb Type Descriptor 2.1`, on the same USB 2 bus (Bus 001) as the lidar and the Arduino. Only one cable carries USB 3 and the user cannot buy more, so that cable now goes to the front D455 (337122300107, the only IMU), which shows 3.2. The left D455 (146222253403) and the D435i (207522077542) stay on USB 2.1. The D435i rejects `424x240x5` (`rgb_camera.color_profile`) and falls back to 1280x720x30, which adds USB and CPU load.
3. **`/scan_fused` slow and stale (open, main cause of stop-and-go).** `/scan` holds about 9–10 Hz, but `/scan_fused` runs at only 2.3–6.3 Hz. `scan_fusion_v9.py` is pure Python and sits near 90% of one core, with 50–220 ms average per frame. During the last goals the collision monitor logged "Latest source and current collision monitor node timestamps differ ... Ignoring the source" 76 times, alongside 206 RPP "detected collision ahead!", 11 `StopZone` stops and 8 "Controller patience exceeded". Next fix: make the fusion cheaper by raising the point-cloud `downsample` stride, lowering the camera depth rate or resolution in the launch file, or dropping the colour stream where only depth is used. Then re-measure with `scan_count.py` (below) and require `/scan_fused` at 8 Hz or more before driving.
4. **Start pose next to a wall.** The start pose (-0.70, -0.61) is only 0.10 m from a mapped wall (map distance transform), so the planner returned "Start occupied" and Spin/BackUp quit with "Collision Ahead". Start in open floor instead: the map origin (0, 0) has 0.70 m clearance. Set **2D Pose Estimate** in RViz carefully every time.
5. **`optical_lab` point in a narrow doorway (changed, uncommitted).** The old point (13.56, 4.69) sits in a passage about 0.4 m wider than the chair's 0.57 m turn radius, so any person nearby closes it. `src/wheelchair_description/config/locations.yaml` now has `optical_lab: {x: 13.92, y: 3.65}`, the open floor in front of the door with 1.2 m clearance. This change has not been driven yet. Commit it once a run reaches it.
6. **IMU warning (open, not investigated).** `imu_diagnostic` reported "MADGWICK NOT TRACKING GYRO! Ratio: infx" in every launch, with roll shown as about 153°. Check whether the heading drifts while the chair is still.
7. **Bag recording.** The launch tries to write to `/opt/nvidia/rosbags`, which does not exist, so `ros2 bag record` dies. Launch with `record_bag:=false`, or fix the path.

## Branches
- `voice-slm-nav` (Mode 2, this checkout) is pushed to `origin/voice-slm-nav` at `31f94a3`. Uncommitted: `locations.yaml` (item 5) and the untracked `logs/`.
- `ros-nav` (Mode 1) has the firmware cherry-picked as `ad82234`; `voice-vlm-nav` (Mode 3) has it as `c151e14`. Both are pushed.
- `stash@{0}` holds an obsolete v30 carry made on the stale Hybrid v30; `voice-vlm-nav` has the correct one.

## Real-chair run steps that worked
```
ls -l /dev/wheelchair_arduino /dev/rplidar
rs-enumerate-devices | rg "Serial Number|Usb Type Descriptor"   # front D455 must show 3.x
cd ~/wheelchair_nav && colcon build --base-paths src && source setup.bash --skip
ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py record_bag:=false   # defaults: v31, v3 BT, real_chair_indoor_2026-11-05_v2 map
# wait for lifecycle_manager_navigation "Managed nodes are active", then set 2D Pose Estimate in RViz
source .venv-voice/bin/activate && ros2 run wheelchair_description voice_nav.py
```
To check scan rates, run a small `rclpy` subscriber with `qos_profile_sensor_data` that counts messages for 6 s (the last session used one in its scratchpad). `ros2 topic hz` and `ros2 topic echo` printed nothing for the large scan topics from the agent's shell, so don't trust them. Run `clearance_monitor.py --params <v31> --scan-topic /scan_fused --duration 8` with the chair still to see the closest scan points and the map clearance at the AMCL pose.

To stop everything, kill by process, never with `pkill -f "ros2 launch"` from a shell whose own command line contains that string (it kills the shell first). Check afterwards that nothing under `/opt/ros/jazzy`, `wheelchair_nav/install`, `rviz2` or `voice_nav.py` is left.

## Left, in order
1. Speed up `scan_fusion` (item 3) and re-measure `/scan_fused` at 8 Hz or more.
2. Bench-test the firmware with the wheels off the ground: command timeout, stop during a pivot, stall cutoff.
3. Real-chair Mode 2: 5 goals between `dp_lab` and the new `optical_lab`, starting from open floor, for the video and the 4-of-5 pass.
4. Voice: refuse weak matches such as "doctor" → `dp_lab`, and add place descriptions to `locations.yaml` (Mode 2 context).
5. Mode 3 exploration: not started.

## Sim results (unchanged, 2026-11-08 morning)
- Mode 1, v29 with the v3 BT: 5/5 goals, 0 recoveries, 0 mid-path stops, cruise 0.50 m/s, 0 clearance hits (closest 0.154 m).
- Mode 2, v31 with the v3 BT: 5/5 purpose-only phrases and 3/3 name phrases, no BT recoveries, closest 0.23 m.
- v30 on `voice-vlm-nav`: 5/5, 0 recoveries, 0.50 m/s, closest 0.19 m.
- Sim commands are in `how to run.md` and the previous handoff in git history (`git show 18ee882:HANDOFF.md`).

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run. The new firmware's 300 ms serial timeout also stops the chair if the controller manager stalls, which can happen under heavy CPU load.
