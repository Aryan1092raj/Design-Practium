# Handoff — 2026-11-10 (after the fourth real-chair session)

This codebase is yours: what it's for and how it's built are your call, and you'll keep building on it.

## StopZone fix — 2026-11-12 (supersedes the diagnosis below)

The v30 StopZone blocker was the chair's own rear parts. The lidar's 1.00 m rear crop in `scan_fusion_v9.py` misses them, and the user confirmed they are chair parts. Three exclusion zones now cover them, and `vlm_nav.py` search turns check `SPIN_CLEAR` (1.20 m) instead of `TURN_CLEAR` (1.32 m). The StopZone size is unchanged. The zones also apply to v29 and v31. Last night's stop at the start of the approach is still unexplained, because stationary dumps from before it had 0 StopZone points. If the live probe is clean, watch the first metre of the drive. The codex review failed with a 402 budget error, so the diff has had only a manual review of `turn()`. The details and evidence are in `fixes.md`, and the offline replay check `src/wheelchair_description/test/self_filter_replay_check.py` passes. Both packages are rebuilt. The next step is on the real chair: launch `./scripts/v30_real.sh --text` and align the pose, then run `scripts/stopzone_probe.py --seconds 20`. It must show fewer than 4 StopZone points in every scan before the chair trial (2 to 4 m) and then the door trial. If a new cluster appears at the left rear corner, which would mirror the right-side part, the rider's legs probably hid it in these captures.

## StopZone diagnosis — 2026-11-12

**Fresh capture analysed:** `logs/stopzone_20261112_142210/` reads live v30 geometry and four-point threshold. Of 100 samples, 77 fused projections succeeded and all contain 24–537 StopZone points; the other 23 have observer TF errors, not evidence of clear space or a native monitor fault. After eight seconds every successful projection has 24–34 rear-right points. Main cluster base x `-0.39..-0.37`, y `-0.57..-0.48` m; smaller intermittent group near `(-0.62, -0.56)`. Seven late same-stamp snapshots match every hit through raw lidar, filtered lidar, lidar-only and fused scans; an eighth matches lidar-only. The persistent group is therefore lidar, not camera reintroduction. Front-camera hits were transient; the user was in front, later reported nothing beside the rear-right wheel and sitting about 1 m to the side. Physical identity remains unresolved; do not enlarge self filtering merely to remove the cluster. Saved TF agrees with the configured lidar mount, which does not prove physical calibration. Native CM markers/state were idle. Offline analysis/plot/JSON are beside the capture; detailed evidence is in `docs/STOPZONE_DIAGNOSIS.md`. Navigation/filter/protection code is unchanged. The real stack was already stopped when a follow-up capture was considered. Next: stationary capture with the observer at a different known position, then inspect/measure any cluster that stays fixed to the chair before selecting a correction. No agent hardware launch or motion occurred.

The user kept Qwen and asked how to fix StopZone. The logged real goal was **(2.91, 1.49)**, accepted from (1.83, 1.42) at `1794424828.9049`; StopZone activated ~62 ms later. The earlier (2.24, 1.46) position was an offline candidate. Costmap clears and controller retries did not resolve the blocked start. Earlier stationary fused snapshots have zero StopZone points and cannot identify the later trigger.

Prepared a read-only `scripts/stopzone_probe.py`: reads live CM params, captures raw/filtered/fused/lidar-only/per-camera scans and TF, plus Nav2's actual `/collision_monitor/collision_points_marker`, polygon and state. Reports trigger coordinates and separates stale/TF failures. With the normal stack running and chair stationary: `.venv-voice/bin/python -B scripts/stopzone_probe.py --seconds 20`. Captures go to `logs/stopzone_<timestamp>/`. The observer has no motion publishers or action clients. Its actual installed-Nav2 regression passed on isolated domain 72 (3 points clear; 4 stop; external obstacle clear; stale source stop; native/projected counts agree). All isolated processes were stopped.

Possible fusion cause: v9 filters the lidar branch before adding camera overlays, which receive no equivalent self filtering. Do not assume those camera points are wheelchair returns or apply the old exclusion sectors blindly: nearby real obstacles could be hidden. Fresh stationary capture is needed before selecting a sensor/geometry correction. No fusion filtering, speed limits or safety envelopes changed in this work. See [docs/STOPZONE_DIAGNOSIS.md](docs/STOPZONE_DIAGNOSIS.md). Real-chair readiness was requested asynchronously; no answer arrived during the offline preparation, so no hardware was launched.

## Model evaluation — 2026-11-12

The user asked whether LFM3 could replace the current VLM after last night's failed chair approach. LFM3-VL-2B QAD Q4 plus F16 projector runs in the installed llama.cpp (`0.8.0-dev`, `39febd2ad`), using about 2 GB GPU memory versus Qwen's 4 GB on the RTX 4050. **Keep Qwen3-VL 4B Instruct configured:** on identical saved-image canonical cases Qwen detected 9/11 present objects, direct LFM 4/11, and documented-prompt LFM 3/11. Bounded reasoning improved LFM to 7/11 but accepted an absent refrigerator in the selected negatives. Both models detected the chair in all four saved hallway frames. Full comparison: 166 image/phrase cases and 14 multi-image checks per model/configuration; these are perception replays, not navigation acceptance trials.

Details and limitations are in [docs/LFM3_EVALUATION.md](docs/LFM3_EVALUATION.md). Downloaded models, repeatable runner, hashes, source docs and raw responses remain local in `bench_tmp/lfm3_eval/`. All model servers started for the evaluation were stopped. No hardware or Gazebo launch/movement occurred. Existing uncommitted diagnostic/footprint edits were preserved; no navigation implementation or limits changed in this evaluation.

The manual real attempt in `logs/live_voice.log` detected the chair at 2.1 m and submitted an approach. `logs/manual_nav_002711.log` logged StopZone activation at `1794424828.966`; the approach ultimately failed. Earlier stationary captures are not simultaneous with that activation and cannot identify the triggering live points. Next navigation work should capture the actual collision-monitor scan, base transform, polygon and triggering points together while stationary; replacing the VLM supplies no reason to reduce obstacle protection.

## Latest checkpoint — 2026-11-11

**Local launcher additions after commit `a72c58c`:** the user requested real-chair entry points for v31 and v30 plus mapping that saves on Ctrl+C, and asked to keep them local. `scripts/v31_real.sh` selects v31/BT v3 and preloads local Qwen3 4B + CUDA Whisper; `scripts/v30_real.sh` selects v30/BT v5 and checks the correct Qwen3-VL 4B server + CPU Whisper. Both start RViz/Nav2, retain the hardware/rate checks and wait for operator pose alignment before voice. v31 now supports text-only startup and bounded shutdown cancellation. `scripts/map_real.sh` starts lidar-only SLAM/RViz by default, verifies scans for 120 s, saves/verifies map files while SLAM is alive on Ctrl+C, then stops the launch. It disables the legacy simultaneous-shutdown session manager. Optional posegraph/data are saved locally; no bag/dataset recorder is started. See [LOCAL_CHAIR_RUN.md](LOCAL_CHAIR_RUN.md). These additions are not pushed, and no real hardware was launched. Local model loads, synthetic map saving and fake-action v31 cancellation passed; all ten packages built. Full current acceptance remains open as below.

The current v30 code is prepared for the user's limited chair/door/box hardware trial. The final chair/door simulation passed 2/2 with the enclosing CAD footprint and live contact coverage; full Mode 3 acceptance and real-chair validation remain open. No hardware launch or movement occurred in this preparation session. Current speed is 0.40 m/s.

Compact scored windows, contact coverage, exact-stamp localization and stop evidence are preserved in [validation/mode3_20261111](validation/mode3_20261111). [MODE3_TRIAL_RESULTS.md](MODE3_TRIAL_RESULTS.md) summarizes the final results and [MODE3_RUN.md](MODE3_RUN.md) gives launch and hardware checks. Larger recordings and benchmark runners remain local in `bench_tmp/` and `~/wheelchair_bench`; they are not included in the commit. Older readiness notes below describe their historical code states.

## Mode 3 real-trial preparation 2026-11-11 (latest; supersedes readiness notes below)

The user now wants a limited first real-chair trial on a chair, door or box. They explicitly said TV failure and the simulation's hanging light should not delay that trial. Small objects, TV, AC, stairs and clothing-specific people are not requirements for this first trial. This is narrower than full Mode 3 acceptance; do not claim the mode is completely validated.

Done in this session:

- Door scoring now checks heading to the same physical panel's nearest surface, so a correctly approached visible end is not failed for facing away from its hidden centre. Distance <=1 m and heading <=45 degrees are retained. Fresh `m3_large_b` scored 4/5 (table, chair, door, fridge) with zero recoveries and sampled map overlaps. Door gap 0.40 m, heading 3.6 degrees. TV was found but no safe close approach was available. Earlier `m3_large_a` remains 3/5.
- Sim AMCL uses OmniMotionModel for the axle-corrected base motion. Real Mode 3 now explicitly passes `axle_to_base:=0.31984` to ZUPT via the real launch; ZUPT's internal integrator remains axle-based while published pose, twist, covariance and TF shift to base_link. AMCL switches to Omni in both real scan branches only for shifted ZUPT. Direct launch defaults remain zero offset.
- Offline actual ZUPT publication/reference/covariance checks and existing odometry selfchecks passed. A live ZUPT on isolated domain 64 passed a 160-message synthetic full axle pivot; the actual real parameter gate also passed. No wheel commands or hardware launch occurred.
- The real script reads back ZUPT and AMCL parameters and checks live front RGB + aligned depth, paired freshness, intrinsics, valid ranges and camera-to-base TF before voice startup. Existing USB/serial, full 120 s scan-rate, lifecycle, RViz pose and ordered shutdown checks remain.
- Elevated RGB-D objects retain measured depth instead of snapping to the cabinet underneath. Mixed separated surfaces remain rejected. An additional crop-label check now rejects disagreements for horizontally edge-clipped boxes.
- Native Gazebo contact telemetry now covers all 14 converted collision shapes. `m3_large_c` was stopped incomplete after a false table detection and physical main-body contact with Chandelier_01_003 during the TV search. Its chair, door and fridge routes had no contact. Do not count it as a full-suite pass.
- The CAD main body extends x=-0.562..0.678 m and nearly 2 m high, larger than the old +/-0.45 m rectangle. The user replied that these are real-chair dimensions. v30 now encloses every CAD collision shape at x=-0.57..0.69, y=+/-0.50, including the asymmetric right castor sphere reaching y=-0.4875, with existing 0.03 padding. StopZone, SlowdownZone and FootprintApproach enclose the same body. VLM turn room is 1.32 m around the axle. `bench_tmp/physical_geometry_check.py` passed against all converted STL/primitive collisions and the actual params. A new sim map `maps/small_house_clearance.yaml` includes collision geometry through 2.1 m (`world_to_map.py --max-height 2.1`) so hanging fixtures are represented. The old map is preserved.

**Ready for the user's limited chair/door real trial.** The final current-footprint run `m3_large_trial_b` passed **2/2**, zero recoveries, zero physical obstacle contacts, and zero sampled map overlaps. Chair: 0.569 m front-probe gap, 16.2 degrees heading; door: 0.943 m, 41.9 degrees toward its physical panel. There were five predicted collision-ahead stops on the chair route, so this does not establish Mode 1 smoothness. Continuous native support/contact telemetry covered all 14 publishers; contact summaries and same-stamp localization summaries are in `bench_tmp/m3_large_trial_b_*`. Moving localization p95 was 0.1335 m / 0.707 degrees. The benchmark exited normally.

Earlier `m3_large_trial_a` also reached chair and door with zero contact, before the final castor-width enlargement. Its scoring completed and saved both windows, then native Gazebo transport crashed at interpreter teardown. `GazeboTruth.close()` now destroys transport before Python shutdown; the final trial_b exited 0. Original score criteria and geometries are retained. These two-target smoke runs do **not** establish a full five-target Mode 3 pass.

Live sim stop check passed: an active search spin was canceled and the wheel command reached zero in 0.19 wall seconds, stayed zero. Result: `bench_tmp/mode3_stop_result.json`. `colcon build --base-paths src` passed for all 10 packages. Launch-substitution, actual ZUPT reference/covariance, original odometry, mixed-depth, contact-classification and CAD envelope checks passed. Python/shell syntax and CRLF-aware diff whitespace passed.

**Stopped:** all simulation, navigation, voice, recorder and isolated domain-64 test processes. The healthy VLM server remains on port 8091. No hardware launch or real movement was performed.

**Real run:** from an interactive terminal in this workspace, `./scripts/mode3_real.sh` (laptop mic + text subscriber), or `./scripts/mode3_real.sh --text`. The script supplies the corrected reference, checks USB/serial, measures scans for 120 s, checks live RGB-D and ZUPT/AMCL settings, and waits for the operator's RViz pose alignment/Enter before voice starts. Power the Arduino first, put the front D455 directly on USB 3, start with a clearly visible chair or door on open floor and keep the power cut within reach. Say “take me to the chair”, “go to the door” or “stop”. Ctrl+C stops voice before navigation. The speed remains 0.40 m/s. A box phrase is supported but not sim-validated. Full Mode 3 and real microphone/camera/route validation remain unfinished. See MODE3_RUN.md and fixes.md.

## Mode 3 sim session 2026-11-11 (read this first; it supersedes the older Mode 3 sections below)

### Later the same day

Florence-2 large was tested as a detector on 142 cases: zero misses, but a box on all 111 absent cases with both `OPEN_VOCABULARY_DETECTION` and `CAPTION_TO_PHRASE_GROUNDING`, at 0.22 s and 1.8 GB VRAM. Qwen3-VL stays: one miss and zero false boxes.

The localization root cause was missed updates after stopping: the map fits the lidar 97–99% at the true pose, but AMCL only updates while moving and was left 0.19–0.73 m and up to 15.5° off after spins. `amcl_settle.py` now requests 15 no-motion updates at 5 Hz once the chair stops, launched in sim and the real launch; the spin check gave 0.03–0.24 m and under 2°. AMCL params are unchanged; variants A/B/C changing update_min, alphas and sigma_hit did not help or jumped 3.34 m.

In sim, `odom_axle_to_base.py` takes the sideways swing from the IMU turn rate and the EKF fuses `vy`; odom error fell to 0.05–0.15 m. This is sim only: the real launch still reports axle motion as `base_link`.

A codex review found 14 defects; 10 are fixed. Shutdown and worker exceptions cancel the outstanding navigation or spin goal with a bounded 2 s wait while the executor runs. Goal submission checks stop before sending and after acceptance, cancels late acceptance, and bounds the response wait to 5 s. The v30 planner now excludes unknown floor, matching coverage BFS. Small lethal blobs only stop blocking turn clearance when fresh scan evidence identifies them as stale noise, while poles remain obstacles. The final face spin checks clearance at the actual arrival pose and yaw and is skipped when blocked. The last attempt stops instead of driving to a standoff from a sighting beyond `MAX_RANGE`. RGB and depth pairs more than 0.10 s apart are rejected within the existing freshness timeout. Re-looks keep at least 60 s but the whole command, including intermediate drives, is capped at `SEARCH_TIME + 120 s`. The nearer close-in fallback chooses the farthest eligible spot toward `MIN_SIGHT` to keep low objects in frame. When no exploration spot shows 4 m² unseen, the best spot showing at least 1 m² is accepted. Four defects remain: a partition in the last metre, blind-zone floor counted as seen, colour words, and low-object floor removal.

With those fixes, `m3_ae` scored 2/5: the first speck rule removed turn-safe spots (241 to 172 on a saved costmap), and a false fridge sighting ended the command. Both changed: the speck rule now uses scan evidence and unusable sightings keep searching. The wardrobe ended 1.81 m off AMCL in `m3_ae`; its cause remains open.

`m3_af` with these changes was still running at commit time, so THIS COMMIT STATE HAS NOT PASSED IN SIM. Do not run it on the real chair before a 4/5 sim run.

v30 equals v31 except `FollowPathReverse`, `always_send_full_costmap: true`, `allow_unknown: false`, and `min_velocity` x −0.10. Offline checks live in `~/wheelchair_bench` and `bench_tmp/` (untracked).

Mode 3 reached 4 of 5 objects in Gazebo `small_house` in three runs in a row (`m3_aa`, `m3_ab`, `m3_ac`), with no collisions and no recoveries. The fourth, `m3_ad`, scored 3/5 on the bench: the bed failed, and the sofa was reached (0.50 m, 0.7°) but the bench fails any recovery and there was one local-costmap clear as the chair left its parking spot at the wardrobe. AGENTS.md fails only a recovery loop, so by that rule `m3_ad` is 4/5. These earlier results predate the later changes and failures recorded above; the changes are committed on `voice-vlm-nav`.

The user wants Mode 3 to work in any indoor environment, so no fix below uses place names or small_house geometry.

### What changed in `vlm_nav.py`

- **Exploration by map coverage** replaced the straight-ray `explore_goal()`. `coverage()` builds turn-safe spots from the live global costmap: cost-0 cells on a 0.25 m lattice, reachable by BFS over cost 0–98, with `TURN_CLEAR + AXLE_BACK` (1.36 m) to solid structure. `mark_seen()` ray-casts 5 m from each finished scan. `coverage_goal()` picks the spot with the most unseen open floor per metre of path, times 1.5 when it lies in the view `choose()` picked. Nav2 routes there, so doorways need nothing special.
- **Lidar specks:** the obstacle layer leaves thousands of one-cell marks beside walls (4603 of 4777 lethal blobs in `m3_x`). Only blobs at most 16 cm² with fresh scan evidence of free space can be ignored for turn room; a spot is still a cost-0 cell, so inflation keeps it clear of each mark. The sim global costmap is 0.02 m per cell, not 0.05.
- **Search limit** is 600 s on the node clock (`now()`), not 180 s of wall time: Gazebo ran at about 0.6× real time, and a scan plus the drive to it takes about 50 s. `go == 0` from the VLM only drops the direction preference. Spots showing 4 m² of unseen floor are preferred; when none does, the best spot showing at least 1 m² is accepted.
- **Scan** is 6 looks at 60° (`LOOKS`), so an object cut by one frame edge is whole in the next view. A blocked spin no longer ends the search: the chair drives to the nearest turn-safe spot and scans there (`coverage_goal(nearest=True)`), twice at most.
- **Far sightings** go to `close_in()`: the nearest turn-safe spot at least `MIN_SIGHT` from the estimate with a lethal-free line to it. When no such spot exists it takes the farthest eligible spot from 1.5 m to `MIN_SIGHT` (`min_range`). Re-looks keep the rest of the search time or at least 60 s, within the whole-command cap of `SEARCH_TIME + 120 s`.
- **Standoff** tries every bearing, the chair's side first, and needs the spot reachable and in plain view of the object. When none fits, `close_in(side=True)` moves the chair at least 60° round the object for another look. `ground()` falls back to the left and right half of a wide box (the L-shaped sofa's centre sits in a nook).
- **Verify:** the rider's phrase is mapped once to a list label ("couch" → sofa). `KIN` lets wardrobe/cabinet and table/desk stand for each other, but only on a box at least a third of the frame; a 29 px "cabinet" sent the chair 6 m the wrong way in `m3_ab`.
- **Whisper:** no `initial_prompt` (it made Whisper invent "stop" from mic noise; `hotwords` uses the same prompt path in faster-whisper 1.2.1). Segments need `no_speech_prob <= 0.5` and `avg_logprob >= -0.95`. `voice_nav.py` still has the old prompt; carry the fix on `voice-slm-nav`.
- **`vlm_server.sh`:** `--cache-ram 0 --parallel 1`. The 8 GB host prompt cache grew the server until earlyoom killed it mid-run twice.

The later review state, with 10 fixes and four remaining defects, is recorded above.

### Bench (`~/wheelchair_bench`)

`cover_check.py` (coverage, close-in, standoff on the sim map), `explore_check.py`, `wedge_check.py`, `verify_check.py` (needs the VLM server) and `whisper_check.py` (run with `.venv-voice`; exits 1 on four eSpeak recognition misses, noise clips all pass). `m3.sh` sets `VLM_NAV_DUMP=~/wheelchair_bench/dump`, where `vlm_nav.py` saves each verified frame and the costmap whenever no spot is left, a sighting has no usable viewpoint, or it reports "can't get closer" or "can't find a free spot"; `m3_x`'s bed failure was reproduced offline from `costmap_145.npy`. Do not launch `m3.sh` from a shell command that contains the text `vlm_nav.py`: its `pgrep -f` guard matches the launching shell.

### Runs

| Run | Score | Failed | Cause |
|-----|-------|--------|-------|
| `m3_x` | 3/5 | bed, wardrobe | lidar specks left no spot facing the bedroom |
| `m3_y` | 4/5 | bed | VLM said `go: 0`, old rule ended the search |
| `m3_z` | 3/5 | wardrobe, sofa | wardrobe labelled "cabinet"; no standoff from the east |
| `m3_aa` | 4/5 | wardrobe | labelled "cabinet" again, side-on |
| `m3_ab` | 4/5 | wardrobe | far kitchen cabinet accepted as kin, re-look limit ended search |
| `m3_ac` | 4/5 | sofa | spin blocked where the chair parked at the wardrobe |
| `m3_ad` | 3/5 (bench) | bed, sofa | bed seen at 4.1 m, no close-in spot `MIN_SIGHT` away in the bedroom; sofa reached with one recovery |

### Still open

- **Localization is still a limit.** No-motion updates improved the spin check and sim axle/`base_link` odometry is corrected, but the wardrobe ended 1.81 m off AMCL in `m3_ae`; its cause is open. The real launch still reports axle motion as `base_link`.
- Not checked on the real chair: red-box verify on the real camera, Whisper thresholds on a human voice, the speck rule at the lab map's resolution.
- Florence-2 was tested and rejected as a detector because it boxed every absent case; Qwen3-VL stays. SAM mask depth and image tiling remain dropped as not needed by any run.

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
1. Mode 3 in sim, which doesn't need the chair: on `voice-vlm-nav` (0.40 m/s already carried into v30), run the VLM zero-shot path in Gazebo `small_house` (`src/wheelchair_description/scripts/vlm_server.sh`, `vlm_nav.py`). Pass: 4 of 5 objects reached; this commit state has not passed yet.
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
  `m3_p` (older code) scored 1/5, no recoveries. Contact is not measured: at the fridge stop the
  rack corner sat at (0.29, 0.35) in the chair's body frame, on the 0.35 m footprint edge, so likely
  touching (collision_ahead=1 there). Cheap add: footprint at the Gazebo pose vs the map's distance
  transform after each request (mid-drive contact would need polling). Sofa PASS (front gap 0.44 m, 5.9°).
  Wardrobe arrived, front gap 0.92 m, but bearing 45.1° (limit 45°). Bed: the VLM verified the
  sofa as "bed" and the chair went there (5.3 m from the bed). Fridge: seen at 8.2 m, the close-in
  drive ended with AMCL 0.96 m off Gazebo (14° yaw), so the chair stood 0.46 m from the rack
  corner while it believed it was in the open. The next spins hit "Exiting Spin", and `turn()`
  then ended the fridge and dumbbell searches at once ("not found" in 15 s and 11 s). A pass
  needs a 4/5 sim run on this commit state before a real-chair run.

### Earlier Mode 3 changes (now committed on `voice-vlm-nav`)

`src/wheelchair_description/scripts/vlm_nav.py` and `src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml`:

- A close-in drive toward a far sighting stops `MIN_SIGHT` (2.5 m) short. The camera (1.34 m up, 64°
  vertical FOV) sees the floor only beyond ~2.2 m, so driving closer lost the dumbbell in `m3_e`.
- Exploration now tracks seen floor and chooses reachable turn-safe spots on a 0.25 m lattice,
  with cost 0 at the goal; this replaces the older distance-from-scanned-spots rule.
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
  The final spin now checks the actual arrival pose and yaw. Untested whether
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
- The unsafe `or goal` fallback for far sightings is removed; the later review fixes are recorded
  at the top of this section.

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
   loss, like the 11 m `m3_m` wedge). Before no-motion updates, 0.3 m failed every goal because
   sim AMCL was routinely 0.2–0.7 m off after spins. `m3_score_check.py` covers it. Old windows files can't PASS.
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
Nav2 models. Earlier AMCL ended 0.17–0.66 m off Gazebo after spins, when the sim EKF fused only
`vx` and dropped the swing. The current sim correction takes the sideways swing from the IMU
turn rate and fuses `vy`; no-motion updates fix the missed AMCL updates after stopping, with the
spin-check results recorded above. Mode 1 and 2 sim passes ran with the earlier gap.
The real chair uses the same URDF; if it also turns about the axle,
every rotate-in-place under-models the front swing by up to ~0.3 m. The sim odometry correction
is implemented; the real launch still has no axle shift. `vlm_nav.py`'s axle-centred `clear()`
only covers Mode 3's own goals.
Bag check done (`maps/session_20261107_000512`): in all 4 in-place turns AMCL moved `base_link`
0.23–0.36 m, so the real chair very likely turns about the axle too. Real odometry has no axle
shift: `diff_drive_controller` publishes the axle motion as `base_link` (`wc_control.yaml`), and
`odom_axle_to_base.py` runs only in `gazebo_sim.launch.py`. In turns 3–4 odom moved 0.02–0.03 m
while AMCL moved 0.23–0.25 m. Fix when the user wants it: launch `odom_axle_to_base.py` in the real
launch and point `ekf.yaml` `odom0` at `/wc_control/odom_base`, sim-check first. The user chose to
run v31 as-is first and watch AMCL after every in-place turn. AMCL parameter variants did not
help the sim spin error or jumped 3.34 m; params remain unchanged. The later review of
`vlm_nav.py` and the fixes are recorded at the top of this section.

### Mode 3 open problem after the bench is fixed

The older 4-view direction pick (`choose()`) sent the chair the wrong way: in `m3_e` and `m3_h`, with every
scan completing its turns, the VLM chose east for the bed (west) 4–5 times. Options for the user:
a better prompt (e.g. name each photo's room first), passing the search history into the prompt
(codex suggestion), or fine-tuning the VLM. Don't tune the prompt to one run.

### Real chair (only when the user says so; power cut within reach)

- **AMCL jump (bag `maps/session_20261107_000512`):** at 317.4 s, after an in-place turn to yaw
  1.3 rad near (6.3, 1.4), cov xx went 0.04 -> 27 m² in 1.5 s while y stayed tight; at 319.0 s the
  pose jumped 4.16 m along x with 0.03 m of odometry motion. AMCL settled at x ≈ 11–12 and the rider
  re-localized by hand at 375 s to (8.2, 1.9), so it went to a wrong pose, not a correction. Looks like
  corridor aliasing along x. Random recovery is off. Hypothesis, not tested: v29's
  `update_min_d/a: 0.01` with `resample_interval: 1` kept the cloud overconfident (cov yy 0.001).
  AMCL actually gates normal updates on motion; no-motion updates are now requested after stopping.
  No params changed.

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
