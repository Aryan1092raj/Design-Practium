# Architecture: saved-map autonomous navigation with named places

The chair already maps, localizes on a saved map and drives to a named place, in simulation and on hardware. This document describes how those pieces fit together, what is still missing to make the workflow repeatable on a real chair, and where the VLM and voice layer plugs in. Nothing here changes the motor path: Nav2 and the safety layer remain the only components that command the wheels.

The proposal rests on one idea. A "marker" is a named pose (`x`, `y`, `yaw` in the map frame) stored next to the map it was recorded on. The user drives the chair to a spot once, saves it under a name, and later asks for it by name.

## 1. What exists today

All of this is in the repository now. The simulation figures come from `AUTONOMOUS_NAV.md`; the real-chair mapping and navigation launches have not been re-tested in this work.

| Piece | Where | State |
|---|---|---|
| Manual mapping on the chair | `run_slam` (`wheelchair_slam_mapping.launch.py`) | Drive with the joystick, Ctrl+C saves `.pgm`, `.yaml` and a rosbag to `maps/session_*/` |
| Navigation on a saved map | `run_nav` (`wheelchair_fusion_nav.launch.py`) | map_server, AMCL, Nav2, velocity smoother, collision monitor |
| Named places | `src/wheelchair_description/config/locations.yaml` | One global file with three sim places |
| Go to a place | `scripts/go_to_location.py` (`go_to(name)`) | Sends a `NavigateToPose` goal, returns success as a boolean |
| Simulation | `gazebo_sim.launch.py nav2:=true map:=...` | Same Nav2 parameters as the real chair, plus `nav2_sim.yaml` overlay |
| Sim map | `scripts/world_to_map.py` or `slam:=true teleop:=true` | Exact map from the world file, or a map driven by hand |

## 2. System layers

Data flows up through the layers. Only the velocity command flows back down.

```mermaid
flowchart TB
    subgraph Intent["Intent layer (new, vlm-pipeline / voice-pipeline)"]
        VLM["VLM / voice: 'take me to the kitchen'"]
        VAL["Intent validator: target must exist in places.yaml"]
    end

    subgraph Places["Place layer (small additions)"]
        PL["places.yaml (per map)"]
        GOTO["go_to(name)"]
    end

    subgraph Nav["Navigation layer (exists)"]
        NAV2["Nav2: planner, controller, behavior tree"]
        SMOOTH["velocity_smoother"]
        SAFE["collision monitor"]
    end

    subgraph World["World frame (exists)"]
        MAP["map_server: map.yaml + map.pgm"]
        AMCL["AMCL: map -> odom"]
    end

    subgraph State["State estimation and sensing (exists)"]
        EKF["ZUPT / EKF: odom -> base_link"]
        FUSE["scan fusion: lidar + 3 depth cameras"]
    end

    subgraph HW["Hardware or Gazebo"]
        MOT["Arduino motors + encoders / diff_drive plugin"]
        SENS["RPLidar, RealSense / simulated lidar"]
    end

    VLM --> VAL --> GOTO
    PL --> GOTO
    GOTO -- "NavigateToPose action" --> NAV2
    MAP --> AMCL
    AMCL --> NAV2
    FUSE --> AMCL
    FUSE --> NAV2
    SENS --> FUSE
    SENS --> EKF
    EKF --> AMCL
    NAV2 --> SMOOTH --> SAFE --> MOT
```

The only link from the intent layer to the wheels goes through `go_to(name)` and the Nav2 action. The VLM never publishes `/cmd_vel`.

## 3. Workflow: build the map and mark places

Mapping is manual, as requested. The chair is driven once, the map is saved, then the user drives to each spot and saves it as a named place.

```mermaid
flowchart TD
    A["Start run_slam<br/>(or sim: slam:=true teleop:=true)"] --> B["Keep chair still ~3 s<br/>IMU gyro bias"]
    B --> C["Drive every area slowly<br/>return to start (loop closure)"]
    C --> D["Ctrl+C: map saved to<br/>maps/session_YYYYMMDD_HHMMSS/"]
    D --> E{"PGM check:<br/>thin straight walls,<br/>no ghost walls?"}
    E -- "no" --> A
    E -- "yes" --> F["Start run_nav with this map"]
    F --> G["Localize: chair at map origin<br/>or RViz 2D Pose Estimate"]
    G --> H["Drive to a spot with the joystick"]
    H --> I["save_location.py name<br/>(proposed): reads /amcl_pose,<br/>appends to places.yaml"]
    I --> J{"More places?"}
    J -- "yes" --> H
    J -- "no" --> K["Map folder holds map + places.yaml"]
```

Today steps H to I are manual: read `/amcl_pose`, compute `yaw = 2 * atan2(z, w)`, edit `locations.yaml`. `save_location.py` automates exactly that, and nothing more.

## 4. Workflow: go to a place

```mermaid
sequenceDiagram
    actor U as User
    participant V as VLM / voice
    participant G as go_to(name)
    participant P as places.yaml
    participant N as Nav2 (NavigateToPose)
    participant S as smoother + collision monitor
    participant M as Motors / Gazebo

    U->>V: "go to the kitchen"
    V->>G: intent {action: go_to, target: kitchen}
    G->>P: look up kitchen
    P-->>G: x, y, yaw (or unknown name: reject)
    G->>G: check AMCL is localized (proposed)
    G->>N: goal pose in map frame
    N->>S: /cmd_vel plan
    S->>M: limited velocity (max 0.25 m/s, 0.35 rad/s)
    N-->>G: SUCCEEDED / FAILED
    G-->>V: boolean result
    V-->>U: "arrived" or "could not reach it"
```

A rejected name, a lost localization or a failed goal all return `False`. The chair stops where Nav2 leaves it; the intent layer only reports the result.

## 5. Per-map layout

Places only make sense on the map they were recorded on, so they live in the same folder as that map. This replaces the single global `locations.yaml`.

```mermaid
flowchart LR
    ROOT["maps/"] --> S1["session_20261004_101500/"]
    ROOT --> S2["small_house_world/ (sim)"]
    S1 --> S1A["map.yaml"]
    S1 --> S1B["map.pgm"]
    S1 --> S1C["places.yaml"]
    S1 --> S1D["pose graph + rosbag"]
    S2 --> S2A["small_house_world.yaml / .pgm"]
    S2 --> S2B["places.yaml"]
```

`go_to` should find the file from the map that is actually loaded, so a place list can never be applied to the wrong map. Reading the `yaml_filename` parameter of the `/map_server` node gives that path without a new argument. A `--places <file>` override stays available for tests. See `docs/examples/places.example.yaml` for the file format.

## 6. Simulation parity

The simulation runs the same chain with a few substitutions, so the whole workflow above can be tested without hardware.

```mermaid
flowchart LR
    subgraph Real["Real chair"]
        R1["RPLidar + 3 RealSense"] --> R2["/scan_fused"]
        R3["Arduino diff drive"]
        R4["run_slam / run_nav"]
    end
    subgraph Sim["Gazebo small_house"]
        S1["simulated lidar"] --> S2["/scan (raw)"]
        S3["gz diff_drive plugin"]
        S4["gazebo_sim.launch.py nav2:=true"]
    end
    subgraph Shared["Identical in both"]
        C1["nav2_params_3cam_v29.yaml"]
        C2["go_to_location.py"]
        C3["places.yaml format"]
        C4["intent layer (VLM)"]
    end
    R2 --> Shared
    S2 --> Shared
    Shared --> R3
    Shared --> S3
```

Differences that matter: the sim uses simulated time, feeds the raw `/scan` to AMCL and the costmaps in place of `/scan_fused`, and layers `nav2_sim.yaml` over the real parameters. It spawns the chair at the world origin, so AMCL needs one `/initialpose` message before the first goal. Simulation odometry is about 7 percent off in scale and has not been calibrated, so sim travel times do not predict real ones.

## 7. What to build

Four small additions, in this order. Each one is independent and testable on the sim.

1. **`save_location.py <name>`** reads one `/amcl_pose` message, converts the quaternion to yaw and appends the entry to the active map's `places.yaml`. Check: save a place in the sim, restart `go_to_location.py` on it, confirm `SUCCEEDED`.
2. **Per-map places in `go_to_location.py`.** Resolve the places file from the loaded map, with `--places` as an override, and add `--list`. Check: run the same command against two maps with different places and confirm each list differs.
3. **Initial pose from the map.** Save a `home` place when mapping starts (the origin) and publish it to `/initialpose` after `run_nav` starts. This removes the manual RViz step. Check: restart the sim, confirm AMCL converges without RViz.
4. **Localization guard in `go_to`.** Refuse to send a goal while the AMCL covariance is above a threshold. Check: publish a bad initial pose in the sim and confirm the goal is refused instead of attempted.

The intent layer on this branch then needs only `list_places()` (to give the VLM its vocabulary) and `go_to(name)`. The JSON contract is in `docs/examples/vlm_intent.example.json`.

## 8. Physical markers (optional, later)

The named poses above are virtual markers, and they need no hardware. If the chair drifts or restarts away from the origin, fixed visual tags (AprilTag or ArUco) at a few known spots could reset AMCL's pose when seen by a RealSense camera. This costs a detector node and a tag-to-map calibration step, so it is only worth adding if AMCL loses the chair in practice. Nothing in the first four steps depends on it.

## 9. Risks and open questions

- **Real chair, not re-tested.** `run_slam` and `run_nav` were not run in the simulation work. Step 1 should be proven on the chair before the VLM layer relies on it.
- **Odometry offset.** The sim needed a 0.32 m axle-to-`base_link` shift in its odometry. The real chair may have the same offset; this is unchecked.
- **Clearance.** Places closer than about 1.3 m to furniture failed with "Start occupied" in the sim, because the 0.9 x 0.7 m footprint plus inflation covered the goal. `save_location.py` should warn when the saved pose is inside the inflated costmap.
- **`go_to` uses a new node per call** and expects `rclpy.init()` first. The Python import path from the VLM process is untested; wrap it in a small ROS node or service if the VLM runs in a separate process.
- **`run_nav` kills all ROS 2 processes** at start (`pkill -9 -f ros2`). It must not be launched while the simulation is running.
