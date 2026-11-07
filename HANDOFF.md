# Handoff — 2026-11-07 (evening)

## Current goal

Get every mode to a sim pass that holds up, then prepare the first real-chair run of Mode 1.
No real-chair runs until the user says so.

## Status by mode

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

## Mode 3 changes (uncommitted on `voice-vlm-nav`)

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

## Test bench (now in `~/wheelchair_bench`, outside the repo, uncommitted)

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

## The chair turns about its axle, not `base_link` (user's call; affects every mode)

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

## Mode 3 open problem after the bench is fixed

The 4-view direction pick (`choose()`) sends the chair the wrong way: in `m3_e` and `m3_h`, with every
scan completing its turns, the VLM chose east for the bed (west) 4–5 times. Options for the user:
a better prompt (e.g. name each photo's room first), passing the search history into the prompt
(codex suggestion), or fine-tuning the VLM. Don't tune the prompt to one run.

## Real chair (only when the user says so; power cut within reach)

- **Left veer:** on straight segments RPP needs a steady -0.21 rad/s and the chair still yaws
  +0.03 rad/s. Bench test with wheels off the ground: command equal left/right targets, log target vs
  measured wheel speed, and read the firmware boot banner to record which sketch is flashed.
- **AMCL jump (bag `maps/session_20261107_000512`):** at 317.4 s, after an in-place turn to yaw
  1.3 rad near (6.3, 1.4), cov xx went 0.04 -> 27 m² in 1.5 s while y stayed tight; at 319.0 s the
  pose jumped 4.16 m along x with 0.03 m of odometry motion. AMCL settled at x ≈ 11–12 and the rider
  re-localized by hand at 375 s to (8.2, 1.9), so it went to a wrong pose, not a correction. Looks like
  corridor aliasing along x. Random recovery is off. Hypothesis, not tested: v29's
  `update_min_d/a: 0.01` with `resample_interval: 1` resamples every 0.2 s even when stationary and
  keeps the cloud overconfident (cov yy 0.001). No params changed.
- Raised speed limits (0.50 m/s, 0.40 m/s²) have only run in sim. Suggested ramp on the chair:
  0.25, then 0.40, then 0.50 m/s.

## Notes

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
