# Handoff — 2026-11-10 (after the fourth real-chair session)

This codebase is yours: what it's for and how it's built are your call, and you'll keep building on it. The user is switching to Fable 5.2 for the next session.

## Current goal
The user wants to test Mode 3 (voice + VLM, zero-shot) next. Repeated real-chair faults have blocked every test so far. The left wheel encoder overcounted by a factor that kept changing (1.54, then 1.28, then 1.34). It is now calibrated with `CPR_L = 13100`, and on 2026-11-10 the chair drove 5 m straight on the ground. Mode 2 on the real chair is unblocked. A rigid mount would stop the factor from drifting again.

Mode 3 does not have to wait for the encoder. AGENTS.md requires every mode to pass in Gazebo `small_house` before the real chair, and the sim has no encoder fault. Mode 3 lives on `voice-vlm-nav` with `nav2_params_3cam_v30.yaml` (the `gazebo_sim.launch.py` default). Everything from `voice-slm-nav` was merged into `voice-vlm-nav` on 2026-11-10, including the encoder calibration, the voice weak-match fix and 0.40 m/s cruise in v30. The Mode 3 state from this branch's own earlier handoff is at the end of this file.

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

## Voice run 2026-11-10 (after the encoder fix, v31, `logs/voice_real_233349.log` and `logs/real_nav_232421.log`)
The user set the 2D Pose Estimate and drove two real goals by voice. Both arrived, with no StopZone stops, no stack crash and no rider takeover:

| Goal | Time | Encoder L/R, yaw | Events |
|---|---|---|---|
| "take me to dp lab" | 38.8 s | 1.11, −116° (the route turns) | SlowdownZone 10, 1 `no valid path`, 1 spin, 1 backup after `Collision Ahead - Exiting DriveOnHeading` |
| "go to optical lab" | 16.6 s | 1.01, −3° | SlowdownZone 11, 1 spin |

Two more dp lab goals came while the chair was already there, so they didn't test driving. One came from the misheard "I will do a pickle lab", which voice_nav matched to the dp lab. That's the weak-match bug again: it should answer `none`. These two real goals count toward the Mode 2 pass (5 goals, 4 of 5 needed). Still open: the recovery near the start of the dp lab goal, and SlowdownZone toggling about 10 times per goal.

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
1. Mode 3 in sim, which doesn't need the chair: on `voice-vlm-nav` (0.40 m/s already carried into v30), run the VLM zero-shot path in Gazebo `small_house` (`scripts/vlm_server.sh`, `vlm_nav.py`). Pass: 4 of 5 objects reached.
2. The encoder is now calibrated and drives straight. Repeat the 5 m ground run once (`scripts/open_straight.py 5.0 0.4`), and re-measure the factor if the user ever remounts the encoder. Then finish the Mode 2 real-chair pass: 5 goals between `dp_lab` and `optical_lab` (13.92, 3.65), 4 of 5 required. 2 arrived on 2026-11-10 (see Voice run above). Fix the weak-match bug first ("pickle lab" → `dp_lab`).
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
- `voice-vlm-nav` (this checkout) has all of `voice-slm-nav` merged in as of 2026-11-10 (`1ae0a1e`). Its v30 cruise is 0.40 m/s, down from 0.50 (accel 0.40, decel −0.50 and angular 0.80/1.2 kept).
- `voice-slm-nav` is at `1ae0a1e`. Its last commit is the voice weak-match fix: `fuzz.ratio` in place of `WRatio` for name matching, plus an SLM rule that an unlisted place gives `none`. Check: `src/wheelchair_description/test/voice_match_check.py`, 27 of 27 phrases, run against the live Ollama.
- `ros-nav` is at `fd114ff` and doesn't have the 0.40 change.

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

## Mode 3 state (from the `voice-vlm-nav` handoff of 2026-11-07)
Kept as written then, apart from two dropped notes. The left veer was the left encoder, fixed on 2026-11-10. The speed ramp is settled at 0.40 m/s cruise in every mode.

### Status by mode

- **Mode 1, `ros-nav` (v29): passes in sim, with one caveat.** Two runs (`m1_a`, `m1_b`) reached all
  5 goals at p90 0.50 m/s with no mid-path stops and no collisions. The test script scored 3/5 and 4/5
  because it fails any recovery: each recovery was one global costmap clear after SmacPlanner2D said
  "no valid path found" mid-route on a ~15 m goal (west, once east), followed by a successful replan.
  AGENTS.md fails only a recovery loop. Cause of the planner failures is open (stale obstacle marks are
  the guess; raytrace 12 m >= obstacle 10 m looks right). Watch for it in doorways on the real chair.
- **Mode 2, `voice-slm-nav` (v31): passed earlier (5/5 sim, 19/19 offline).** Not rerun this session.
- **Mode 3, `voice-vlm-nav` (v30): not passing.** The last clean run on older code (`m3_h`) scored
  3/5. `m3_j`–`m3_m` were spoiled by the test bench. `m3_n` (first run on the fixed bench) scored
  0/5 with no collision-ahead stops or recoveries (the bench does not measure contact): the 1.11 m effective `clear()` rejected every standoff and
  close-in step (sofa "no free spot", far fridge and wardrobe "can't get closer"), bed and dumbbell
  not found. `clear()` is now axle-centred and lethal-only (below). `m3_o` was voided by earlyoom.
  `m3_p` (current code) scored 1/5, no recoveries. Contact is not measured: at the fridge stop the
  rack corner sat at (0.29, 0.35) in the chair's body frame, on the 0.35 m footprint edge, so likely
  touching (collision_ahead=1 there). Cheap add: footprint at the Gazebo pose vs the map's distance
  transform after each request (mid-drive contact would need polling). Sofa PASS (front gap 0.44 m, 5.9°).
  Wardrobe arrived, front gap 0.92 m, but bearing 45.1° (limit 45°). Bed: the VLM verified the
  sofa as "bed" and the chair went there (5.3 m from the bed). Fridge: seen at 8.2 m, the close-in
  drive ended with AMCL 0.96 m off Gazebo (14° yaw), so the chair stood 0.46 m from the rack
  corner while it believed it was in the open. The next spins hit "Exiting Spin", and `turn()`
  then ended the fridge and dumbbell searches at once ("not found" in 15 s and 11 s). A pass
  needs two runs at 4/5.

### Mode 3 changes (uncommitted on `voice-vlm-nav`)

`src/wheelchair_description/scripts/vlm_nav.py` and `src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml`:

- A close-in drive toward a far sighting stops `MIN_SIGHT` (2.5 m) short. The camera (1.34 m up, 64°
  vertical FOV) sees the floor only beyond ~2.2 m, so driving closer lost the dumbbell in `m3_e`.
- Exploration skips goals within 1.0 m of spots already scanned for the command (`m3_e` drove back to
  its own scan spot), uses integer 5 cm samples, and needs cost 0 at the goal.
- `near_face()` snaps the depth estimate to the first lethal global-costmap cell on the chair-to-object
  line, within ±1 m, for sightings <= 4 m. Depth put the sim bed and sofa estimates inside the object
  and the wardrobe 0.7 m short of it.
- Standoff and exploration goals both need `clear(p, yaw)`: no lethal or unknown cell within
  `TURN_CLEAR` 1.04 m of the drive axle (0.32 m behind `base_link` along yaw). The padded front
  corner sweeps 0.89 m about the axle, plus 0.15 m goal tolerance. A cost-30 goal 0.66 m from the
  bed corner trapped the chair in `m3_g`. In `m3_m` an exploration goal (1.88, 3.87), cost 0 but
  0.63 m from the dumbbell rack, let the corners clip the rack during the 4-view scan and wedged
  the chair; exploration goals had no turn check then. The old check (lethal or inscribed within
  0.76 m of `base_link`) needed ~1.11 m and still modelled the turn about the wrong point.
  Assumes the chair arrives facing the goal yaw (shortcut comment in `clear()`). Untested whether
  front gaps stay within 1 m. Self-checks: `~/wheelchair_bench/explore_check.py` and
  `wedge_check.py` (replays the `m3_m` goal on the sim map: rejected; the new pick's axle is 1.04 m
  from the rack).
- `turn()` counts only `STATUS_SUCCEEDED`; an aborted spin had let `choose()` run on four photos of
  one heading.
- v30 global costmap `always_send_full_costmap: true` (header CHANGES 2): vlm_nav reads only full maps
  and missed live obstacles.
- Tried and reverted: a full-resolution, min-size, black-padded verify crop. On the 16 saved views it
  added two false boxes at the frame edge and lost one real dumbbell.
- Self-check: `explore_check.py` (explore_goal, near_face and clear on a fake costmap) passes.
- Codex reviewed the diff twice; all findings fixed except: the `or goal` Nav2 fallback for far
  sightings still ignores `MIN_SIGHT` (shortcut comment in `run()`), and a revisited far endpoint
  rejects a direction even when a shorter unvisited stop exists.

### Test bench (now in `~/wheelchair_bench`, outside the repo, uncommitted)

The bench moved out of the old scratchpad: `m3.sh`, `m3_run.py`, `sim.sh`, `run.sh`, `nav_run.py`,
`views/`, the `m3_*` logs, and the self-checks. Run `bash m3.sh <v30 params> <label>` from there.

1. **earlyoom kills the VLM server** below 10% free RAM and swap. `m3.sh` now refuses to start under
   20% free or when 8091 `/health` fails. An earlyoom `--avoid` for llama-server needs sudo (user's
   call).
2. **One sim at a time:** `m3.sh` refuses to start while `gz sim`, `bt_navigator` or `vlm_nav.py`
   runs. To stop a chain, kill the background task first, then children; `pkill -f` needs the
   `[x]yz` bracket form.
3. **Scoring from Gazebo (done).** `m3_run.py` scores gap and bearing from `gz model -m wheelchair
   -p` (map frame = world frame, model root = `base_link`, AMCL agrees within 0.04 m at spawn). It
   logs amcl-vs-gazebo after warm-up and every request, and fails a goal only above 1.0 m (gross
   loss, like the 11 m `m3_m` wedge). 0.3 m failed every goal: sim AMCL is routinely 0.2–0.7 m off
   after spins (next section). `m3_score_check.py` covers it. Old windows files can't PASS.
4. **Wrong premise, no fix needed:** `maps/small_house_world.pgm` is built from world collision
   meshes by `world_to_map.py`, not lidar. The dumbbell rack (0.40 m tall) and FitnessEquipment are
   both in the map and both cross the 0.29 m lidar plane. The wedge came from the goal choice (above).
5. **Warm-up spot (0.20, 1.48) kept:** it is 2.18 m from the nearest mapped obstacle, and `m3_n` had
   no "Exiting Spin". The `m3_m` failures there are unexplained; load (RTF 0.69, stalled camera) is
   a guess.
6. `m3_run.py` starts on a `/voice/transcript` subscriber, since the one "ready" message can be
   missed. Its `rclpy.shutdown()` is guarded (the `m3_m` traceback was a double shutdown after Ctrl-C).
7. Other sessions' Chromium (`headless_shell`) used ~165% CPU in `m3_m`; none was running for `m3_n`.

### The chair turns about its axle, not `base_link` (user's call; affects every mode)

`~/wheelchair_bench/spin_check.sh` (sim, four 90° spins at the warm-up spot): every Gazebo pose,
moved 0.32 m back along its yaw, lands within 6 cm of one point, so the chair turns about the drive
axle (wheel joints at x −0.32 in the URDF). Nav2's spin and rotate-to-heading checks and the
footprint assume `base_link`. The front corners actually sweep 0.89 m on a turn, against the 0.61 m
Nav2 models. AMCL ended 0.17–0.66 m off Gazebo after the spins even though `odom_axle_to_base.py`
shifts the wheel odometry: the sim EKF fuses only `vx`, so the swing is dropped. Fusing `vy` as well
was tried and reverted: position error stayed (up to 0.70 m) and yaw went 16–19° off. Mode 1 and 2
sim passes ran with this gap. The real chair uses the same URDF; if it also turns about the axle,
every rotate-in-place under-models the front swing by up to ~0.3 m. The AMCL jump right after an
in-place turn (below) may share the cause, but that is untested. Root fixes are frame-level (put
`base_link` on the axle and shift the footprint, or fix the odometry in sim and on the chair) and
touch every mode. `vlm_nav.py`'s axle-centred `clear()` only covers Mode 3's own goals.
Bag check done (`maps/session_20261107_000512`): in all 4 in-place turns AMCL moved `base_link`
0.23–0.36 m, so the real chair very likely turns about the axle too. Real odometry has no axle
shift: `diff_drive_controller` publishes the axle motion as `base_link` (`wc_control.yaml`), and
`odom_axle_to_base.py` runs only in `gazebo_sim.launch.py`. In turns 3–4 odom moved 0.02–0.03 m
while AMCL moved 0.23–0.25 m. Fix when the user wants it: launch `odom_axle_to_base.py` in the real
launch and point `ekf.yaml` `odom0` at `/wc_control/odom_base`, sim-check first. The user chose to
run v31 as-is first and watch AMCL after every in-place turn. Untested lead for the 14° AMCL yaw error at the `m3_p` fridge stop (yaw comes from the IMU,
so the axle doesn't explain it): AMCL alphas are 0.05, which may keep the cloud too tight to
recover from systematically wrong rotation odometry.
The new `vlm_nav.py` changes (explore goals need `clear()`; `clear()` axle-centred, lethal-only,
`TURN_CLEAR` 1.04, `round()` radius) have not had a Codex review yet.

### Mode 3 open problem after the bench is fixed

The 4-view direction pick (`choose()`) sends the chair the wrong way: in `m3_e` and `m3_h`, with every
scan completing its turns, the VLM chose east for the bed (west) 4–5 times. Options for the user:
a better prompt (e.g. name each photo's room first), passing the search history into the prompt
(codex suggestion), or fine-tuning the VLM. Don't tune the prompt to one run.

### Real chair (only when the user says so; power cut within reach)

- **AMCL jump (bag `maps/session_20261107_000512`):** at 317.4 s, after an in-place turn to yaw
  1.3 rad near (6.3, 1.4), cov xx went 0.04 -> 27 m² in 1.5 s while y stayed tight; at 319.0 s the
  pose jumped 4.16 m along x with 0.03 m of odometry motion. AMCL settled at x ≈ 11–12 and the rider
  re-localized by hand at 375 s to (8.2, 1.9), so it went to a wrong pose, not a correction. Looks like
  corridor aliasing along x. Random recovery is off. Hypothesis, not tested: v29's
  `update_min_d/a: 0.01` with `resample_interval: 1` resamples every 0.2 s even when stationary and
  keeps the cloud overconfident (cov yy 0.001). No params changed.

### Notes

- Test scripts, logs and self-checks live in `~/wheelchair_bench` (see Test bench). The old
  `/tmp/claude-1000/.../953ad2f5-.../scratchpad` still holds the `sim_*.log`/`voice_*.log` files of
  `m3_a`–`m3_m`, `sheet_*.jpg` and the `wt-ros`/`wt-slm` worktrees; a reboot wipes it.
- `m3_run.py` PASS: arrived, front point (base_link + 0.45 m) within 1.0 m of the target footprint,
  bearing error <= 45°, recoveries 0. "collision ahead" (RPP's predicted stop) is reported, not failed.
- The VLM server runs in hc terminal `vlm2` (`src/wheelchair_description/scripts/vlm_server.sh`);
  earlyoom killed the first one at 19:09 during `m3_o` (6.5% RAM, 9.9% swap free), which voids
  `m3_o` from the dumbbell on (before that: fridge arrived, front gap 0.44 m, bearing 26°; bed not
  found). `m3.sh` now logs free memory and the top 3 processes every 5 s to `mem_<label>.log`.
- Run sim tests on `ROS_DOMAIN_ID=42`, one at a time.
- `vlm_nav.py` needs `--ros-args -p use_sim_time:=true` in Gazebo.
- Scratchpad worktrees for `ros-nav` (`wt-ros`) and `voice-slm-nav` (`wt-slm`) still exist.
- Codex helpers: put the parent's literal terminal id in the report instruction; `$HC_TERMINAL_ID`
  expands to the helper's own id, and codex's sandbox blocks hc anyway. Read the pane for results.
