# Handoff — 2026-11-08

## Current goal
Modes 1 and 2 pass in simulation with a measured "never touches anything" check. Next is the real-chair step, which needs the firmware fixes uploaded first and the user's go-ahead, then Mode 3 exploration.

## Done (all in Gazebo small_house, collision monitor on)
- Mode 1, v29 with the v3 BT: 5/5 goals, 0 recoveries, 0 mid-path stops, cruise 0.50 m/s. `clearance_monitor.py`: 0 hits, 0 near misses under 0.05 m, closest approach 0.154 m to a lidar point and 0.173 m to a map wall.
- Mode 2, v31 with the v3 BT: 5/5 purpose-only phrases and 3/3 name phrases reached the right place, "take me to the garage" started no goal, no BT recoveries (the 4 Spin calls are `voice_nav.face()` turning to the saved heading). Clearance: 0 hits, closest 0.23 m. The "stop" cancel was not tested.
- v30 on `voice-vlm-nav` with its v5 BT, same 5-goal check: 5/5, 0 recoveries, 0 stops, 0.50 m/s. Clearance: 0 hits, closest 0.19 m.
- Collision monitor `FootprintApproach` zone (action `approach`, chair outline 0.90 x 0.70 m + 0.05 m, 1.2 s) in v29, v31 and v30. It simulates the current command, turns included, and slows it before the chair would touch anything. The rectangular zones miss the corner sweep of an in-place turn (corners reach 0.57 m).
- Collision monitor polygons use the string format Jazzy needs (v29, v30, v31). `nav2_sim.yaml` points the monitor at `/scan` in sim.
- Firmware (`final_control.ino`, uncommitted, compiled with `arduino-cli compile --fqbn arduino:avr:mega`, not uploaded): serial command timeout of 300 ms (the host writes every controller cycle at 100 Hz, `wheelchair_interface.cpp:548`), a zero command cancels a pending pivot, the static-friction boost only breaks away from rest (at most 400 ms, never while decelerating), and an encoder-stall cutoff (output above 40% with no counts for 1.5 s stops both wheels until a zero command).
- Scripts: `mode1_sim_check.py` (stages to (1.2, 0) first because the spawn pose has furniture inside the turn radius), `mode2_sim_check.py` (plays the rider on `/voice/transcript`), `clearance_monitor.py` (footprint-to-obstacle distance from the scan and the map; `--scan-topic /scan_fused` on the real chair).

## Branches
- `ros-nav` (Mode 1): has the polygon and sim-scan fix, `mode1_sim_check.py`, the v29 approach zone and `clearance_monitor.py` (cherry-picked). Its v29 matches the one tested.
- `voice-slm-nav` (Mode 2, this checkout): same commits plus `mode2_sim_check.py`.
- `voice-vlm-nav` (Mode 3): merged `voice-slm-nav` (`6d2ac3f`), keeping its own SmacPlanner2D v30 and HANDOFF, then `3cddd1a` carried the v31 SlowdownZone limit and the approach zone into v30.
- Worktrees for `ros-nav` and `voice-vlm-nav` live in the old session's scratchpad. Run `git worktree prune` if `git checkout` of those branches complains.
- `stash@{0}` holds an obsolete v30 carry made on the stale Hybrid v30; `voice-vlm-nav` now has the correct one.
- Nothing is pushed.

## Uncommitted in this checkout
- `final_control.ino`: the user's feedforward and boost changes plus the safety fixes above. Review, then upload.
- `nav2_params_3cam_v31.yaml`: the user's SlowdownZone change plus the approach zone (tested together in the Mode 2 run).
- `voice_nav.py`: the user's switch of Whisper to CUDA float16, which has no CPU fallback.

## Sim commands
```
ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house use_rviz:=false nav2:=true \
  map:=$HOME/wheelchair_nav/maps/small_house_world.yaml \
  nav2_params_file:=$HOME/wheelchair_nav/src/wheelchair_navigation/config/nav2_params_3cam_v29.yaml \
  use_collision_monitor:=true \
  bt_xml:=$HOME/wheelchair_nav/src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v3.xml
```
Publish `/initialpose` at (0, 0), start `clearance_monitor.py --params <params file> --out <json>`, then run `mode1_sim_check.py`, or start `voice_nav.py --ros-args -p locations:=locations_sim.yaml` from `.venv-voice` and run `mode2_sim_check.py`. Stop the monitor with SIGINT for its summary. v29 and v31 need the v3 BT, because the sim default (v5) calls `FollowPathReverse`, which only v30 defines.

## Left
1. Upload the firmware and bench-test it with the wheels off the ground: command timeout (kill the ROS stack while a wheel turns), stop during a pivot, stall cutoff (hold a wheel).
2. Real-chair runs for Modes 1 and 2, only when the user says so, with `clearance_monitor.py --scan-topic /scan_fused` running.
3. Mode 3 exploration: not started.

## Not covered by the clearance check
- The sim lidar sees -99° to 160°, so the rear-right sector is blind to the scan check. The map check covers static walls there, not moving obstacles.
- Map clearance uses the AMCL pose, so it carries localization error of a few centimetres.
- Reversing was inferred, not measured: `allow_reversing: false` and BackUp only in the BT recovery branch, with 0 recoveries.

## Safety
The FlySky is off, so there is no stick override. Keep the power cut within reach on every real-chair run.
