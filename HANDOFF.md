# Handoff — 2026-11-05

## Current goal
Make the real chair do a U-turn (rotate in place, then drive forward) when a Nav2 goal is behind it, instead of driving the whole way in reverse. It must not hit anything while turning.

## Done
- Nav2 is stopped. Work is back in the main checkout on branch `voice-sim`; the `worktree-sensor-check` worktree is kept but no longer used.
- New config `src/wheelchair_navigation/config/nav2_params_3cam_v31.yaml`, copied from the real-chair v29 (SmacPlanner2D) with these changes:
  - RPP `allow_reversing: false`
  - `use_rotate_to_heading: true`, with `rotate_to_heading_min_angle` set to 0.785 (45°)
  - `max_angular_accel` raised from 0.8 to 3.2, so a turn starting from rest doesn't stall
  - `use_collision_detection: true`, with a 1.0 s lookahead
  - progress checker switched to `PoseProgressChecker` (`required_movement_angle` 0.5 rad, `movement_time_allowance` 10.0)
  - `inflation_radius` raised from 0.40 to 0.60 in both costmaps, which covers the 0.57 m circumscribed radius
- `wheelchair_fusion_nav.launch.py` now defaults to the v31 params and the `wheelchair_robust_nav_v3.xml` BT.
- The D455 depth profile is now `480x270x5` in `wc_control/wheelchair_sensors.launch.py` (front camera) and `wheelchair_fusion_localization.launch.py` (left and right). The cameras run on a USB 2.1 link, which has no 424x240 mode.
- `wheelchair_navigation` was rebuilt with `colcon build --base-paths src`, and v31 is installed.

## Left
1. Relaunch nav. It was stopped before it came up, so v31 has not run on the chair yet.
2. Check the U-turn: send a goal behind the chair. It should rotate in place, then drive forward. Watch for collision stops, for the progress checker firing during the turn, and for "Start occupied" now that inflation is 0.60.
3. If narrow passages fail at 0.60 inflation, lower it toward 0.45 or so.
4. Once goals work, record named points such as "door" and "kitchen" for `voice_nav.py` and `vlm_nav.py`.
5. Nothing is committed yet. Uncommitted changes include the launch files, v31, the rviz config, `voice_nav.py`, and new maps.

## Open questions
- The earlier 14.5 m goal failed with "Start occupied". The fused scan was clean, so the likely cause is localization or inflation, but that is not proven. That goal may also have run before the camera fix.
- Safety: the FlySky is off, so there is no stick override. Keep the power cut within reach.
