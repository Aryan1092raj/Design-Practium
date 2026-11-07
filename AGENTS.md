# AGENTS.md

## Goal

The goal is a powered wheelchair that drives itself to a place the rider names. The project has three modes, and each one builds on the one before it. Nav2 does all of the driving in every mode. The voice and vision layers only choose the goal or cancel it.

### Mode 1: ROS autonomous navigation (done)

The chair localizes on a saved map with AMCL, using the fused lidar and 3-camera depth scan. It plans with SmacPlanner2D and follows the path with RPP. The current real-chair config is `src/wheelchair_navigation/config/nav2_params_3cam_v31.yaml`, launched by `src/wheelchair_bringup/launch/wheelchair_fusion_nav.launch.py`. A goal behind the chair should make it rotate in place and then drive forward, never reverse the whole way.

Two issues are still open:

- **Speed is too slow.** RPP `desired_linear_vel` and the velocity smoother `max_velocity` are both capped at 0.25 m/s, and `max_linear_accel` is 0.25 m/s². Target: cruise at 0.5 m/s, with accel around 0.4 m/s². Test it in simulation before the real chair.
- **Motion is not smooth.** The chair moves in a stop-and-go pattern. Suspected causes, none confirmed: the 1.0 s collision lookahead, the low accel limit, and the collision monitor's `StopZone` and `SlowdownZone`. In the last voice run the chair stopped on a lidar return within 30 cm of its front-right corner. Target: mid-path, the chair never drops to zero velocity unless a real obstacle is inside the `StopZone`.

### Mode 2: voice + SLM (saved places, with context)

`src/wheelchair_description/scripts/voice_nav.py` listens on the laptop mic and turns speech into text with faster-whisper. Place names are matched first with a regex and fuzzy match. When no name matches, a small language model, `qwen3:4b` on Ollama, maps the sentence to one of the saved places in `src/wheelchair_description/config/locations.yaml`, or to `none`; `qwen3:0.6b` could not resolve requests by purpose. Its reply is restricted by a JSON schema enum to those names, which stops qwen3 from padding its output until it runs out of tokens. In the sim, run it with `--ros-args -p locations:=locations_sim.yaml` for the small_house places. Saying "stop" cancels the goal. Replies are printed and published on `/voice/status`, because the real chair has no speaker.

"Context" means the SLM knows what each place is for, not just its name. Each entry in `locations.yaml` gets a short description, so "take me where I can print" resolves to the printer room even though the rider never said its name. Free phrasing of a name ("optics lab please" resolves to "optical lab") must keep working, and anything that matches no place still returns `none`.

### Mode 3: voice + VLM (zero-shot, no SLM)

`src/wheelchair_description/scripts/vlm_nav.py` needs no saved places. The rider names an object ("take me to the table"). Qwen2.5-VL 3B, served by `scripts/vlm_server.sh`, looks for it in the front RGB camera and returns a bounding box. The box plus the aligned depth image give the object's position in the map, and the goal is a free spot a short distance in front of it. If the object isn't in view, the chair turns in place 90° at a time and looks again. No SLM sits in this path: the target phrase goes straight from the transcript to the VLM.

Mode 3 adds exploration beyond the current view. If a full 360° scan doesn't find the object, the VLM chooses where to look next from its own knowledge of the world. It gets all four views from the scan in one call and answers two questions together: is the target in any view, and if not, which view most likely leads to it. For example, a kitchen has counters and appliances, so a doorway showing tiles or a fridge is a better bet than a bedroom. The chair drives a few metres that way, Nav2 picks a free spot, and the chair scans again. The search stops when the object is found, when the VLM sees no promising direction, or after 3 minutes, whichever comes first. If the object isn't found, it publishes "not found" on `/voice/status`. When it is found, the chair stops within 1 m in front of it. The VLM gets no saved place names and no map labels: it works zero-shot from the camera alone.

### Param versions per mode

Each mode has its own Nav2 params file in `src/wheelchair_navigation/config/`:

| Mode | Params file |
|------|-------------|
| 1, ROS autonomous nav | `nav2_params_3cam_v29.yaml` |
| 2, voice + SLM | `nav2_params_3cam_v31.yaml` |
| 3, voice + VLM | `nav2_params_3cam_v30.yaml` |

Fixes flow forward in mode order: v29 to v31, then v31 to v30. When a fix lands in an earlier mode's file, carry it into the later ones unless that file already solves the problem its own way. For example, v30 keeps its SmacPlannerHybrid and its `FollowPathReverse` recovery controller. These three files are edited in place, which overrides the "DO NOT EDIT, create a new versioned file" line in their headers. Git keeps the history. Record each carried fix in the target file's header under "CHANGES" or "CARRIED FROM". `gazebo_sim.launch.py` defaults to v30, so to test v29 or v31 pass `nav2_params_file:=<path>`. `src/wheelchair_description/config/nav2_sim.yaml` overrides `inflation_radius` to 0.65 in sim for every version, so sim won't show inflation differences between them.

### Order of work

The work runs in this order: Mode 1 speed and smoothness first, then Mode 2 place descriptions, then Mode 3 exploration. Modes 2 and 3 both drive through Nav2, so slow or jerky driving hurts every demo.

## Definition of done

Each mode is done when it passes in Gazebo `small_house` first and then on the real chair on the lab map. Both are required. For now all testing happens in simulation. The real chair comes only after a mode passes in sim, and only when the user says so. A pass means at least 4 of 5 different goals reached, where a goal is a pose for Mode 1, a saved place for Mode 2, or an object for Mode 3. A reached goal has no collision, no rider takeover, and no recovery loop. Mode 1 must also meet the speed and smoothness targets above.

## Safety

The FlySky remote is off, so there is no stick override. Keep the power cut within reach on every real-chair run. Never raise speed or accel limits on the real chair without a test in simulation first.

## Working notes

- Handoff state lives in `HANDOFF.md`. Read it first.
- Build with `colcon build --base-paths src`.
- Simulation run steps are in `how to run.md`. Nav steps for both sim and the real chair are in `AUTONOMOUS_NAV.md`.
