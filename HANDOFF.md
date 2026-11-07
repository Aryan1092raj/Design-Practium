# Handoff — 2026-11-07

## Current goal

Work through the AGENTS.md order of work in simulation: Mode 1 speed and smoothness, then Mode 2
place descriptions, then Mode 3 exploration. No real-chair runs until the user says so.

## Done (pushed)

Each branch builds on the one before it; fixes reach the later branches by merge (no force pushes).

- **Mode 1, `ros-nav`.** v29 has the v31 U-turn changes (turn in place, never reverse) plus
  0.50 m/s cruise, 0.40 m/s² accel, -0.50 m/s² decel, and RPP `max_angular_accel` 16. The last
  one fixes a real-chair stall: at 3.2 RPP asked 0.16 rad/s from rest, which sat in the motor
  deadband for 5-10 s (bag `maps/session_20261107_000512`). Speed caps raised to 0.55 m/s and
  0.85 rad/s in `twist_stamped_teleop.py`, `wc_control_safe_v2.yaml` and `wc_control.xacro`;
  wheel command limits ±3.5 -> ±5.5 rad/s (codex review: ±3.5 clipped the outer wheel at 0.50 m/s).
  `wheelchair_fusion_nav.launch.py` defaults to v29 here. Sim: 5/5 goals at p90 0.50 m/s, no
  mid-path stops, no collisions.
- **Mode 2, `voice-slm-nav`.** Same fixes carried into v31 (launch default stays v31). Places in
  `locations*.yaml` take an optional `desc`. `voice_nav.py` matches place names first, then asks
  `qwen3:4b` (user's choice; `qwen3:0.6b` missed every purpose request) with the descriptions.
  Negated commands ("don't go to the kitchen") skip the name match. Sim places with descriptions:
  `config/locations_sim.yaml`, via `--ros-args -p locations:=locations_sim.yaml`. Offline check
  19/19, real-lab names 7/7; sim 5/5 places (4 by purpose, 1 by name). `how to run.md` updated.
- **Mode 3 params, `voice-vlm-nav`.** Same fixes carried into v30. Its SmacPlannerHybrid hit
  "exceeded maximum iterations" (3/5 before and after the carry), so v30 now uses SmacPlanner2D
  (user's choice), 5/5 in sim; FollowPathReverse and the v5 tree stay, and the launch defaults to
  v30 with the v5 tree.

## In progress (uncommitted on `voice-vlm-nav`)

`vlm_nav.py`, `vlm_server.sh`, `AGENTS.md`, `how to run.md`, this file.

- VLM switched to Qwen3-VL 4B Instruct (`ollama pull qwen3-vl:4b-instruct`, ~4.2 GB VRAM).
  Plain `qwen3-vl:4b` is the Thinking model (empty replies at 120 tokens). Boxes are 0-1000
  relative; `vlm()` scales them. Crop check uses a crop padded by half the box (no false boxes
  on the saved views, but it drops small objects like dumbbells).
- Exploration: after a missed 4-view scan, one multi-image call returns `{"found", "go"}`; the
  chair drives up to 3 m toward the chosen view (other views if blocked), rescans; 3-minute limit
  (exploration drives are cancelled when it passes); "Sorry, the X was not found." Far sightings
  (> 4 m, depth unreliable) close in along the bearing, or let Nav2 route to the estimate, then
  look again (up to 4 looks). Standoff 0.8-1.3 m centre-to-face, so the front stops within ~1 m.
  Floor pixels are dropped from the depth patch.
- Latest sim run (`m3_e`, interrupted before the unscored "piano"): 3/5. Fridge, wardrobe and
  sofa reached with front gaps 0.55, 0.41 and 0.57 m. Bed not found after 5 exploration drives
  (186 s); dumbbell not found (crop check rejects it). Needs 4/5 to pass.

## Next

1. Get Mode 3 to 4/5: look at why the bed search wanders (it was found with 1 drive in `m3_d`);
   consider a less strict check for small objects or a different 5th test object. Then
   codex-review the diff and commit to `voice-vlm-nav` (≤300 lines per commit, 6-10 word message).
2. Exploration goals only need costmap cost < 50; the first exploration leg in `m3_d` logged 26
   "collision ahead" and one recovery. Consider requiring cost 0 at exploration goals.
3. Real chair (only when the user says so; keep the power cut within reach):
   - The chair veers left: on straight segments RPP needs a steady -0.21 rad/s and the chair still
     yaws +0.03 rad/s. Firmware runs per-wheel PID; which sketch is flashed is unrecorded (read
     the boot banner). Needs a bench test of left/right target vs measured wheel speed.
   - One AMCL jump of 4.16 m in the same bag (~319.5 s).
   - Raised speed limits have only run in sim.
4. User will send descriptions for `dp_lab` and `optical_lab` in `config/locations.yaml`.

## Notes

- The sim spawn (0, 0) has furniture 0.6-0.7 m away, inside the 0.61 m turn radius; tests drive
  to (0.20, 1.48) first.
- Run sim tests on `ROS_DOMAIN_ID=42`, one at a time, and start the next only after the previous
  one's exit notice: the scratchpad scripts share pid files.
- `vlm_nav.py` needs `--ros-args -p use_sim_time:=true` in Gazebo, or every frame looks stale.
- The VLM server once exited mid-run with no cause found; it now runs in an `hc` terminal (`vlm`).
- Test scripts (`nav_run.py`, `voice_run.py`, `m3_run.py`, `slm_check.py`, `choose_check.py`,
  saved scan views) are in the session scratchpad, not the repo.
- Scratchpad worktrees for `ros-nav` and `voice-slm-nav` exist (user chose to keep them), so those
  branches can't be checked out in `~/wheelchair_nav` until removed.
