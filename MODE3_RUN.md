# Run Mode 3

From the workspace, in a persistent interactive terminal:

```bash
./scripts/mode3_sim.sh          # small_house, domain 42, text commands
./scripts/mode3_real.sh         # real chair, laptop microphone
./scripts/mode3_real.sh --text  # real chair, text commands only
```

Build first with `colcon build --base-paths src`. Both scripts source `setup.bash --skip`
and use `.venv-voice/bin/python`. The existing voice environment, downloaded Whisper
model (for the mic), and Qwen3-VL 4B Instruct server dependencies must already be installed.
The sim also needs `~/fastdds_udp.xml`, as in the bench scripts. Stop previous manually
started ROS/Gazebo/voice sessions first: the PID guard only knows runs started by these
launchers. Both entry points share the startup and shutdown code in `mode3_real.sh`. The real entry point supplies `axle_to_base:=0.31984`: ZUPT publishes the actual offset base pose and lateral twist, and AMCL uses OmniMotionModel for the resulting lateral swing. Direct navigation launches keep their previous zero-offset default; use the Mode 3 entry point for this trial.

The user authorized a limited chair/door/box real trial on 2026-11-11, with TV failure excluded from that trial. The updated footprint is checked with the chair/door smoke run recorded in HANDOFF.md. This is a first hardware trial, not a completed Mode 3 acceptance result; full acceptance still requires the project's 4-of-5 simulation and real-chair goals. **Keep the power cut within reach. FlySky is off; there is
no stick override.** Power the Arduino before launch; put the front D455's USB 3 cable
directly in the laptop, and start on open floor.

## Startup checks

Every checked step prints `PASS` or `FAIL` with its reason. A failure stops startup and
cleans up any voice/navigation processes this invocation started.

| Check | Meaning / response to failure |
|---|---|
| Camera serials | Real only: front D455 `337122300107`, left D455 `146222253403`, right D435i `207522077542` must all enumerate. Reconnect the missing camera. |
| Front USB | Real only: the front D455 must report USB `3.x`. The other two may use USB 2.1. |
| Serial devices | Real only: `/dev/rplidar` and `/dev/serial/by-id/usb-Arduino__www.arduino.cc__0042_9513731393235170B2F1-if00` must be character devices readable/writable by this user. These are the launch defaults; an unrelated `ttyUSB`/`ttyACM` does not pass. |
| Memory | `(MemAvailable + SwapFree) / (MemTotal + SwapTotal)` must be at least 20%, checked before loading the VLM. Close memory-heavy programs if low; this is a startup check, not ongoing monitoring. |
| VLM health | Reuse HTTP-success `/health` on `127.0.0.1:8091`; otherwise start `src/wheelchair_description/scripts/vlm_server.sh` and wait up to 180 s. An already tracked starting server is awaited. |
| Navigation active | Wait up to 180 s for `Managed nodes are active`: all three real managers (localization, navigation, collision monitor), or the sim's single manager. Known lidar/Arduino errors and `Aborting bringup` fail immediately. |
| Scan rates | After discovery, count messages with `rclpy` and `qos_profile_sensor_data` for 120 wall seconds on the real chair, or 20 wall seconds in sim. Real: `/scan` 9–11 Hz, `/scan_fused` at least 8 Hz. Sim: `/scan` only, measured per simulated second, so slower rendering does not itself fail. A paused clock fails. |
| Mode 3 localization | Real only: read back ZUPT offset 0.31984 m, TF publication and wheel heading, plus AMCL OmniMotionModel. A mismatch stops startup. |
| Front RGB-D stream | Both: fresh RGB and aligned depth within 0.10 s, matching dimensions, valid depth, intrinsics and camera-to-base TF. Camera enumeration alone does not pass this check. |
| `amcl_settle` | Must appear in the live ROS graph. This checks presence, not localization accuracy or completed no-motion updates. Rebuild if the node is missing. |
| Initial pose | Real only: set **2D Pose Estimate** in RViz until scans align with walls, then press Enter in the launch terminal. No voice node starts before Enter. The sim uses its configured initial pose and runs without RViz. |

`--no-checks` skips hardware, memory and scan-rate checks, with an explicit `SKIP` line.
It keeps server health, lifecycle readiness, live front RGB-D checks, real localization parameter checks, `amcl_settle`, PID guards and the real pose
confirmation. It is intended for checks already performed manually.

The real launcher's rate gate covers AGENTS.md's full two-minute hardware check.
When using `--no-checks`, first keep the chair still and run
`python3 scripts/scan_rate.py 120` from a sourced terminal. `/scan` should remain
about 10 Hz and `/scan_fused` at least 8 Hz.
If the lidar reports health status 2 or stops producing scans, stop the launch, unplug it
for 10 s and reconnect. If the Arduino resets or the log reports a `WheelchairSystem`
write error, power it first and relaunch. Cable placement, open floor and scan alignment
remain operator checks.

Real launch: `record_bag:=false front_color:=true`; default v30 params, lab map,
RViz and collision monitor are retained. The inherited sensor launch declares
`front_color`. Sim launch loads physics contact telemetry for all 14 collision shapes (simulation only). The contact system has no motor command interface. Sim launch: `world_name:=small_house nav2:=true bridge_camera:=true`,
the `small_house_clearance.yaml` map, v30 params and simulated time. Its collision monitor
keeps the sim launch default (off). The voice node uses `use_sim_time:=true` in sim;
the microphone is always off there.

## Commands and replies

Wait for `Voice navigation ready.`. The launcher prints a text command with the correct
domain. For the sim, use another terminal:

```bash
cd ~/wheelchair_nav
source setup.bash --skip
export ROS_DOMAIN_ID=42 FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fastdds_udp.xml"
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'go to the bed'}"
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'stop'}"
```

For the real chair, use the domain printed by its launcher (normally `0`) and the same
DDS environment as the launch terminal. Text commands work with or without the mic.
Say “take me to the table”, “go to the bed”, or “stop”. Ordinary speech without a
navigation phrase is ignored. Replies are printed and published on `/voice/status`;
there is no speaker. `{target}` below is the requested object.

| Exact reply | Meaning |
|---|---|
| `Voice navigation ready.` | The node is listening for commands. |
| `Turning to look for the {target}.` | It is scanning other headings. |
| `I don't see the {target} here. Looking further that way.` | It is moving to another search viewpoint. |
| `Going to the {target}. Say stop to cancel.` | It has selected an approach goal. |
| `Arrived at the {target}.` | Nav2 completed the approach; actual object distance still needs validation. |
| `I am already next to the {target}.` | Its current estimate needs no approach drive. |
| `Sorry, the {target} was not found.` | The search returned no usable sighting, exhausted reachable viewpoints or reached its time limit; it does not prove the object is absent. |
| `I see the {target}, but I can't get closer to it.` | A sighting could not be approached or rechecked from a usable closer viewpoint. |
| `I see the {target}, but I can't find a free spot next to it.` | No usable final standoff or alternate approach was found. |
| `I could not reach the {target}.` | The selected approach did not complete successfully. |
| `I am already moving. Say stop first.` | A search/drive is already in progress; cancel before choosing another object. |
| `Stopping.` | Cancellation was requested; the stack remains running for another command. |
| `My microphone stopped working. I keep trying to reconnect it.` | Audio capture failed; the node retries. Text commands remain available. |

Search allows up to 10 minutes on the node clock; the overall command, including drives,
is capped at 12 minutes. Simulated minutes take longer when Gazebo runs slowly.

## Stop and logs

Say/publish `stop` to cancel just the current task. **Ctrl+C in the launch terminal**
stops the voice process group first, giving it time to cancel while Nav2 is alive,
then stops the entire navigation/Gazebo process group. Unresponsive groups receive
SIGTERM then SIGKILL, with explicit failure messages. The VLM server stays running.

Navigation and voice logs are `logs/mode3_{real,sim}_{nav,voice}_<timestamp>_<pid>.log`.
Setup and camera enumeration have separate logs; the persistent server appends to
`logs/mode3_vlm.log`. Active process-group leaders are recorded in
`logs/mode3_{real,sim}_{nav,voice}.pid`; `logs/mode3_vlm.pid` records a server started
by these scripts. Never delete a live run's PID file to bypass the guard.

## Validation and first real trial

The launchers have now run the complete navigation/VLM chain in simulation, including the scan-rate and front RGB-D checks and ordered Ctrl+C cleanup. The VLM server was reused and remains running after cleanup. Real localization launch parameters were evaluated without executing hardware actions. An isolated live ZUPT node passed a full synthetic axle turn (160 output messages), and the actual real-localization parameter gate passed on that isolated graph. Covariance/reference and mixed-depth checks passed offline. These checks prepare the real launch; they are not a real-chair navigation pass.

The five large-object requests are table, chair, door, refrigerator and television. Door means approach and face a physical panel, including its visible end. It does not open the panel or drive through it. `m3_large_b` reached 4/5 with no recoveries and no sampled static-map overlap; the TV was seen but could not be approached safely. The contact-instrumented repeat was stopped after a false table detection and a hanging-light contact during TV search. That run is not an acceptance pass. The user subsequently narrowed the first real trial to chair/door/box. The final chair/door smoke check `m3_large_trial_b` reached **2/2** using the full CAD footprint and castor clearance, with no recoveries or measured obstacle contact. Physics contact telemetry was live for both routes. The stop check reached zero wheel command in 0.19 s. This supports the authorized limited trial, not full Mode 3 acceptance. See `HANDOFF.md` for results and artifacts.

For the first real trial, start the laptop microphone and text-command subscriber together:

```bash
./scripts/mode3_real.sh
```

Keep the power cut within reach; FlySky is off. Power the Arduino first, connect the front D455 directly over USB 3, and keep the chair still during the full two-minute scan check. Set 2D Pose Estimate in RViz and align the scan with the walls before pressing Enter. The launcher then starts voice navigation. Start with a visible chair or door on open floor. Say “take me to the chair” or “go to the door”; the text-publish command printed by the launcher is also available. Say/publish `stop` to cancel a task; Ctrl+C stops the stack. For text input without the microphone, use `./scripts/mode3_real.sh --text`. A box request is supported through the zero-shot target phrase but has not been checked in this simulator.

The current v30 speed remains 0.40 m/s. Its footprint conservatively encloses the CAD body and castors at x=-0.57..0.69 m and y=+/-0.50 m; the axle turn-clearance disc is 1.32 m. Stop/slowdown/approach zones enclose the same body. Small objects are not an acceptance requirement. Air conditioners, stairs and clothing-specific people have not passed this suite; do not use stairs or person-selection requests in the first trial. The real camera, microphone, encoder calibration and lab routes still need hardware validation.
