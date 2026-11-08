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

## Left
1. Commit the work (nothing is committed): v29/v30 polygon fix, `nav2_sim.yaml`, `mode1_sim_check.py`, CMakeLists.txt, this file. The user's own uncommitted changes (firmware, `voice_nav.py`, v31 SlowdownZone) need their review first.
2. Carry the v29 speed changes to v30 (v30 still cruises at 0.25 m/s), and decide whether v31's SlowdownZone `limit` change goes to v30.
3. Mode 2 sim run: 5 voice places with `locations_sim.yaml`, v31 params, v3 BT.
4. Mode 3 exploration: not started.

## Open review findings (uncommitted user changes)
- `final_control.ino:333`: static-friction boost can re-engage below 0.3 rad/s, including after a stop command. Not yet verified by the main session. Check before any real-chair run.
- v31 SlowdownZone narrowed to ±0.45 m with a 0.15 m/s / 0.8 rad/s limit: side obstacles no longer slow in-place turns (StopZone still covers ±0.42 m).
- `voice_nav.py:258`: Whisper hard-coded to CUDA float16 with no CPU fallback.

## Not checked
- The sim has no contact sensor, so "no collision" in the pass means Nav2 reported success with no recoveries, not a measured contact check.
- Whether any goal drove in reverse; the script reports absolute speed.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run.
