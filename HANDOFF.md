# Handoff — 2026-11-08

## Current goal
Close out Mode 1 (ROS autonomous nav) in simulation, then move to Mode 2 (voice + SLM) in sim. Real-chair runs wait until the user says so.

## Done
- Mode 1 passed in Gazebo small_house with v29, the collision monitor on, and the v3 behavior tree: 5/5 goals (bedroom, living_room, gym, balcony, kitchen), 0 recoveries, 0 mid-path stops on `/cmd_vel_safe`, max speed 0.50–0.51 m/s. Log: the run printed by `mode1_sim_check.py` on 2026-11-08.
- New script `src/wheelchair_description/scripts/mode1_sim_check.py` (installed through CMakeLists.txt). It sends a staging goal to (1.2, 0.0) first, because the spawn pose has furniture inside the chair's 0.61 m turn radius, then the measured goals. `--selftest` checks the stop counter without ROS.
- v29 and v30: collision monitor `points` changed to the string-of-pairs format Jazzy needs. The old flat list aborted Nav2 bringup, which also affects the real-chair Mode 1 launch now that the wheels go through the monitor.
- `nav2_sim.yaml`: the collision monitor reads `/scan` in sim. It read `/scan_fused`, which Gazebo does not publish, so the monitor stopped every command.
- Codex review of the uncommitted diffs is in the session scratchpad (`review.md`); findings summarized below.

## Sim command for Mode 1
```
ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml \
  nav2_params_file:=$HOME/wheelchair_nav/src/wheelchair_navigation/config/nav2_params_3cam_v29.yaml \
  use_collision_monitor:=true \
  bt_xml:=$HOME/wheelchair_nav/src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v3.xml
```
Then publish `/initialpose` at (0, 0) and run `mode1_sim_check.py`. The sim default BT (`wheelchair_robust_nav_v5.xml`) calls `FollowPathReverse`, which only v30 defines, so v29 and v31 need the v3 BT that the real chair uses.

- Mode 2 passed in Gazebo small_house with v31 (including the user's uncommitted SlowdownZone change), the collision monitor on, and the v3 BT: 5/5 purpose-only phrases ("take me somewhere I can lie down", "I want to cook something", "I'd like to watch some TV", "I need to work out", "I want some fresh air") reached the right place in 18–31 s, and "take me to the garage" started no goal. Driver: `src/wheelchair_description/scripts/mode2_sim_check.py`, which publishes on `/voice/transcript` and reads `/voice/status`; `voice_nav.py` runs from `.venv-voice` with Ollama `qwen3:4b`.

## Left
1. Branch routing. Each mode has its own branch: `ros-nav` (v29), `voice-slm-nav` (v31, this one), `voice-vlm-nav` (v30). The v29/v30 polygon fix and the `nav2_sim.yaml` scan fix were committed here (`c682274`); `ros-nav` and `voice-vlm-nav` still lack them. `voice-vlm-nav` already moved v30 to SmacPlanner2D (`9322d36`), so this branch's v30 is a stale copy.
2. The v30 carry (speed values and v31 SlowdownZone) sits uncommitted in this tree against the stale v30. A copy is in the session scratchpad as `v30_carry.patch`. It belongs on `voice-vlm-nav`.
3. Firmware safety (see below) before any real-chair run.
4. Mode 3 exploration: not started.

## Open review findings (uncommitted user changes)
- `final_control.ino`, verified in code: there is no serial command timeout (only `PPM_TIMEOUT` for the RC input), so if ROS or the laptop dies the chair keeps the last command. A serial stop does not clear `pivot_mode`, so `updatePivot()` restores the stored motion command 300 ms after a stop. Both predate the user's diff.
- `final_control.ino:333` (user's diff, from the codex report): the static-friction boost applies whenever the target is nonzero and measured speed is below 0.3 rad/s, including while decelerating; with no encoder counts and a held nonzero target the output can reach 100% PWM. Full trace and proposed fixes: `fw_review.md` in the session scratchpad.
- v31 SlowdownZone narrowed to ±0.45 m with a 0.15 m/s / 0.8 rad/s limit: side obstacles no longer slow in-place turns (StopZone still covers ±0.42 m).
- `voice_nav.py:258`: Whisper hard-coded to CUDA float16 with no CPU fallback.

## Not checked
- The sim has no contact sensor, so "no collision" in the pass means Nav2 reported success with no recoveries, not a measured contact check.
- Whether any goal drove in reverse; the script reports absolute speed.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run.
