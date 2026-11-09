# Handoff — 2026-11-10 (after the third real-chair session, 2026-11-09 night)

## Current goal
Mode 2 (voice + SLM, v31) on the real chair, on `maps/real_chair_indoor_2026-11-05_v2.yaml`. The chair is blocked on a **faulty left wheel encoder**. On straight drives it counts about 1.54× the right encoder. The user thinks the encoder is mechanically loose. Fix the encoder first. Every other drive result depends on it.

## Root cause found: left encoder overcounts about 1.54×
Integrated per-wheel distance from the logs (`python3 scripts/encoder_ratio.py <log> <t0> <t1>`):

| Drive | Right | Left | L/R | Encoder yaw |
|---|---|---|---|---|
| 4 m straight test, tape 3.045 m (`real_nav_224630.log` 1794246555–576) | 3.41 m | 5.23 m | 1.53 | −184° |
| Optical lab voice goal (`real_nav_224630.log` 1794246268–311) | 11.77 m | 18.38 m | 1.56 | −671° |
| 4 m straight test, tape about 3 m (`real_nav_235357.log` 1794248766–880) | 3.20 m | 4.92 m | 1.54 | −175° |

The right encoder roughly matches the tape. The firmware uses the same `CPR = 10000` for both wheels (`final_control.ino:43`), and the ratio is steady across drives. So this is a steady overcount on the left, which fits a loose encoder (slipping coupling or bad mount) or electrical noise on pins 2/3. It is not a units bug. This one fault explains every symptom seen this session:

- **The chair veers left into walls.** The firmware speed loop believes the left wheel is fast and slows the left motor. In the second straight test the chair curved left until it stopped at a wall. It only moved again after the user pushed it away.
- **Odometry overcounts distance.** The mean of the two wheels is about 4 m for a 3 m drive. This was the "37% / 32%" overcount from the earlier sessions.
- **Heading is wrong, so the chair zigzags and the pose drifts.** Wheel odom reads a 175–184° right turn on a straight drive. After every drive the scan sat rotated off the map walls, and 2D Pose Estimate had to be set again.

## Next session, in order
1. **Fix the left encoder** (re-seat or tighten it, check its cable), then run a bench test with the chair not driving. Lift the drive wheels or release the brakes, mark each wheel, and turn each one exactly 1 revolution by hand while logging. Each wheel should read 2π rad (0.957 m). If the left wheel reads about 1.54 × 2π on every turn, it's a resolution or scale problem: set a per-wheel CPR in the firmware and reflash, with the user's go-ahead. If the reading varies from turn to turn, it's mechanical or electrical: fix the hardware.
2. **4 m straight test, twice** (`python3 scripts/straight_test.py 4.0`, after 2D Pose Estimate in open floor with 5 m clear). Pass: the chair drives straight, filtered distance is within 2% of the tape, and `encoder_ratio.py` shows L/R ≈ 1.0 and yaw near 0.
3. Check the ZUPT heading choice. Codex's fix (below) set `gyro_blend_alpha: 1.0`, which means wheel heading only. That is correct once the encoders agree, and wrong while the left one overcounts. If the chair still drifts once the encoders agree, try 0.3 (pose-increment distance with 70% gyro heading).
4. Voice tests on `voice_nav.py`: redirect mid-drive, stop mid-drive, stop when idle (the RViz path clears), then a new place.
5. Real-chair Mode 2 pass: 5 goals between `dp_lab` and `optical_lab` (13.92, 3.65), 4 of 5 required.
6. Diagnose leftover goal failures: RPP `detected collision ahead!` near walls, `SlowdownZone` toggling (21 times in one 4 m drive), and "Transform data too old".
7. Carry 0.40 m/s to v29 on `ros-nav` and v30 on `voice-vlm-nav`.
8. Earlier open items: refuse weak voice matches ("doctor" → `dp_lab`), IMU "MADGWICK NOT TRACKING GYRO", Mode 3 exploration, and stale AGENTS.md lines (the speed note says 0.25 m/s, and it says AMCL uses the fused scan).

## Done 2026-11-09 (all committed on `voice-slm-nav`)
- **`/scan_fused` went from 4.6 to 9.6–9.9 Hz** (`scan_fusion_v9.py`, edited in place). Camera clouds now go to one worker thread that keeps only the newest cloud per camera. Every hardware check passes. See `fixes.md`.
- **ZUPT odometry** (`zupt_node.py`, launch `gyro_blend_alpha` 0.30 → 1.0). It now integrates wheel pose increments instead of re-integrating the averaged twist, and slip no longer switches the heading blend. Offline check: `python3 src/wheelchair_zupt/test/odom_selfcheck.py`. The chair loaded it (`gyro_blend_alpha` read back 1.0), but it couldn't be validated because of the encoder fault.
- **Encoder parser** (`wheelchair_interface.cpp`): frames like `rp-0.00,lp0.04,` are no longer rejected. Rejected frames per drive fell from 16 to 1.
- **`voice_nav.py`**: naming a new place mid-drive replaces the goal, and stop clears the RViz path. One voice goal to the optical lab reported arrived in 42 s with no collision stops. The chair physically stopped short, which was the encoder fault.
- Speed 0.40 m/s in v29, v30 and v31. The chair loaded 0.4 and `[0.4, 0.0, 0.8]`.
- Scripts in `scripts/`: `straight_test.py` (drives N m straight and prints the distance moved per odom source) and `encoder_ratio.py` (per-wheel distance from a log).

## Real-chair problems seen 2026-11-09 (details in `fixes.md`)
- Arduino powered again mid-launch → `write cycle resulted in an error: [ WheelchairSystem` → no `odom` TF → bringup aborts. Relaunch.
- Motors without power → `Encoder stall fault - stopping both wheels; send zero to rearm`, with `cmd_vel` at 0.4 and the wheels at 0. Turn on motor power; the next goal rearms the fault.
- Lidar `Error, RPLidar internal error detected. Please reboot the device to retry.` → `rplidar_node` exits 255 → no `map` TF → navigation bringup fails. Power-cycle the lidar for 10 s.

## Localization facts
- The real chair's AMCL loads `src/wheelchair_localization/config/amcl_fusion.yaml`, not the v31 file, so AMCL tuning in v31 has no effect there. Only one AMCL runs: `nav2_amcl_filtered` on `/scan_filtered` (lidar only), because `use_fused_scan` defaults to false.
- The odom→base_link TF comes only from `zupt_node`. The active controller config is `src/wc_control/config/wc_control_safe_v2.yaml` (`enable_odom_tf: false`), not `wheelchair_firmware/config/wheelchair_controllers.yaml`. Wheel radius 0.1524 m and separation 0.565 m, with multipliers 1.0.

## Process rules (from incidents)
- Before launching the real stack, check that no sim, `voice_nav` or test process is left. Stop processes by PID or process group only, never with `pkill -f`: on 2026-11-08 a stray sim script sent goals to the real chair.
- Launch the stack in a persistent `hc` terminal (`hc run --bg --outlive-parent --name chairstack ...`). A launch started with `nohup setsid` from an agent shell died once on 2026-11-09 without a logged cause.

## Branches
- `voice-slm-nav` (this checkout): all of the above is committed, not pushed.
- `ros-nav` is at `fd114ff` and `voice-vlm-nav` at `28adb56`, both pushed, neither has the 0.40 change.

## Real-chair run steps
```
rs-enumerate-devices | rg "Serial Number|Usb Type Descriptor"   # front D455 must show 3.x
cd ~/wheelchair_nav && source setup.bash --skip
ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py record_bag:=false   # defaults: v31, v3 BT, real_chair_indoor_2026-11-05_v2 map
# wait for all Nav2 nodes active [3], check /scan ~10 Hz and /scan_fused >= 8 Hz, then set 2D Pose Estimate in RViz in open floor
source .venv-voice/bin/activate && ros2 run wheelchair_description voice_nav.py
```
`setup.bash` overwrites a shell variable named `S`, so use task-prefixed names in scripts that source it.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run. The firmware's 300 ms serial timeout stops the chair if the controller manager stalls. Until the left encoder is fixed, the chair veers left on any goal, so don't run goals near walls or people.
