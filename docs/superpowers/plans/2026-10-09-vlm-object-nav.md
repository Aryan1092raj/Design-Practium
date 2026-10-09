# Zero-Shot Object Search (VLM Phase 2) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A seated passenger in a building the chair has never mapped says "take me to the sofa" (any object name, no training), and the chair builds the map live, searches on its own, confirms the object with a vision-language model, and drives to a free spot next to it, with everything running on the device.

**Architecture:** slam_toolbox builds `/map` live and Nav2 (v30 params, v5 tree) plans on it, with no map_server and no AMCL. A new node, `object_nav.py`, extends `voice_nav.py`, which keeps the microphone, the stop keyword and the spoken replies. For each command the node runs YOLOE with the spoken word as its only class on all three RGB-D cameras, turns each detection into a map point using the aligned depth and TF, and explores the live map's frontiers until two viewpoints agree on one spot. Qwen3-VL-2B then checks that spot on the full image with the box drawn in. When it says yes, the chair announces the target and drives to a free cell 1.5–2.4 m short of it. Step A picks frontiers by size and distance only. Step B adds a VLFM-style value map from MobileCLIP image-text similarity, so the chair heads first toward the areas that look like the target.

**Tech Stack:** ROS 2 Jazzy, Nav2, slam_toolbox, Gazebo (`small_house`), Python 3.12 in `.venv-voice` (created with `--system-site-packages`), Ultralytics YOLOE (`yoloe-26s-seg.pt`), llama.cpp `llama-server` (CUDA) serving Qwen3-VL-2B-Instruct GGUF Q4_K_M, open_clip MobileCLIP (step B), numpy, scipy, OpenCV, pytest.

**Spec:** There is no separate spec file. The design was agreed in the brainstorming session on 2026-10-09 and is summarised in "Design decisions" below. `docs/architecture.md` section 11 records the parked prototype (`scripts/vlm_nav.py`) and why it failed: the 3B VLM's yes/no check on a crop let a wrong object through, 8 m from the real refrigerator.

## Design decisions (agreed 2026-10-09)

The brainstorming session settled these choices:

- **Environment:** no saved map. The chair maps while it searches.
- **Vocabulary:** zero-shot. Any spoken object name works without training or fine-tuning.
- **Compute:** on the device only, with no cloud calls. Runtime is offline (`HF_HUB_OFFLINE=1`).
- **Passenger:** seated. The search starts automatically after the announcement and its 2 s wait.
- **Approach:** A (geometric frontiers + detector + VLM check) first, then B (semantic frontier scoring). Using the VLM as the planner (approach C) was rejected: a 2B model reasons poorly, it takes 1–2 s per call on the Jetson, and it would be one more way to drive somewhere wrong.
- **Model picks:**
  - Detector: YOLOE-26s-seg, kept in PyTorch so `set_classes([word])` can change per command. A TensorRT export bakes the class list into the weights. NanoOWL is the fallback if YOLOE is too slow on the Jetson.
  - Verifier: Qwen3-VL-2B-Instruct, which replaces Qwen2.5-VL 3B. It runs only on clusters that the detector has already confirmed.
  - Semantic cue (step B): MobileCLIP image and text encoders through open_clip. YOLOE ships only the text encoder, so step B adds open_clip as one new dependency.
- **False-positive defence:** a cluster needs two detector hits within 0.5 m of each other from different viewpoints (another camera, ≥0.3 m of travel, or ≥0.3 rad of turn), and then the VLM must answer yes on the full image with the box drawn. A rejected spot is ignored for the rest of the command.
- **Rollout:** laptop + Gazebo, then the laptop on the real chair (empty first, then seated), then the Jetson Orin Nano Super.

## Global Constraints

- Nav2 is the only component that moves the wheels. `object_nav.py` sends and cancels `NavigateToPose` and `Spin` goals and never publishes `/cmd_vel`.
- "stop", "halt" or "cancel" cancels the active goal at any phase. The search loop checks `self.cancelled` at least every `LOOK_PERIOD = 0.5` s.
- The chair announces and waits 2 s at the start of a search and again before driving to a found object.
- Search caps per command: `SEARCH_TIME = 300.0` s and `MAX_TRIPS = 20` frontier goals.
- Depth is accepted only between `MIN_DEPTH = 0.3` and `MAX_DEPTH = 5.0` m. Standoff distances are `STANDOFF = (1.5, 1.9, 2.4)` m, and a goal cell is free when its global costmap cost is in `0 <= cost < 50`.
- Camera topics, for `camera` (front), `mapping_camera` (left) and `right_camera` (right): `/<cam>/color/image_raw`, `/<cam>/aligned_depth_to_color/image_raw`, `/<cam>/color/camera_info`.
- All object-search runs use `nav2_params_3cam_v30.yaml` and `wheelchair_robust_nav_v5.xml`. Do not change the launch defaults (v29 and v3).
- Do not modify `scripts/vlm_nav.py` or `scripts/vlm_server.sh`. They stay parked as the record of the prototype.
- Python packages go into `.venv-voice` with `"numpy<2"` pinned, because ROS 2 Jazzy's Python bindings are built against numpy 1.26.
- Unit tests run without ROS and without models: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test -q`.
- Build: `cd ~/wheelchair_nav && colcon build --base-paths src --symlink-install --build-base build_src --install-base install_src --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`, with conda off `PATH` (see `progress.md`, "Build").
- Git: do the work on branch `vlm-objnav`, created from `voice-sim`. The user's rule is "commit only when I ask", so run the commit steps only if the user has approved commits for this execution. Otherwise stage the files and list them in the task report.

## Review Focus

These five conditions are implied by the design but are not obviously covered. Each one is pinned by a test in the task named.

1. **The VLM server is down or crashes mid-search.** Expected: the chair says its camera check is not running and does not drive to anything unverified. Pinned in Task 4 (`test_verifier_error_counts_as_no`, `test_ping_false_when_server_down`) and Task 7 (ping before each search).
2. **The object is in view but its depth is unusable** (glass, a black sofa, farther than 5 m). Expected: no hit is recorded, the search keeps exploring, and the object is picked up later from closer. Pinned in Task 2 (`test_object_depth_rejects_far_and_invalid`).
3. **The SLAM map's origin moves as it grows,** and cells sit at negative coordinates. Expected: costmap lookups never wrap to the wrong cell. Pinned in Task 2 (`test_grid_value_outside_is_unknown_with_negative_origin`).
4. **A detector box is rejected by the VLM while the chair is driving to a frontier.** Expected: that frontier is not marked as tried, so the chair still explores it. Pinned in Task 7 (`run_and_look` returns `True` when interrupted, and the caller then skips `tried.append`). Checked in the Task 8 protocol, row "false hit mid-trip".
5. **"stop" is said during the 2 s announcement, the 180° look-behind spin, or a VLM check.** Expected: no further goal is sent, and `/cmd_vel` is zero within 1 s. Checked in the Task 8 protocol rows "stop during announce", "stop during spin" and "stop during search".

---

## File structure

All paths are under `~/wheelchair_nav/src/wheelchair_description/` unless noted.

| File | Responsibility |
|---|---|
| `scripts/objnav_geometry.py` (create) | Pure numpy: grid lookups, depth to map point, standoff goal, and hit clustering (`Hits`) |
| `scripts/objnav_frontiers.py` (create) | Pure numpy and scipy: frontier extraction from the live map and frontier choice (`pick`) |
| `scripts/objnav_value.py` (create, step B) | Pure numpy: VLFM-style value map |
| `scripts/objnav_models.py` (create) | Model wrappers: `Detector` (YOLOE), `Verifier` (llama-server HTTP), `SceneScorer` (open_clip, step B). Heavy imports happen inside the classes, so the unit tests need neither torch nor a GPU |
| `scripts/objnav_smoke.py` (create) | Model latency and accuracy check on one image, without ROS (laptop and Jetson) |
| `scripts/objnav_vlm_server.sh` (create) | Starts llama-server with Qwen3-VL-2B on port 8091 |
| `scripts/object_nav.py` (create) | ROS node: the per-command search state machine |
| `scripts/voice_nav.py` (modify) | Adds an `on_unknown` hook and a node-name argument |
| `launch/gazebo_sim.launch.py` (modify) | Adds an `explore:=true` mode: slam_toolbox + Nav2, no map_server and no AMCL |
| `CMakeLists.txt` (modify) | Installs the new scripts |
| `test/test_objnav_geometry.py`, `test/test_objnav_frontiers.py`, `test/test_objnav_models.py`, `test/test_objnav_value.py` (create) | Unit tests |
| `~/wheelchair_nav/src/wc_control/launch/multi_camera.launch.py` (modify, Task 11) | Adds a `side_color` argument for colour on the side cameras |
| `~/wheelchair_nav/src/wheelchair_bringup/launch/wheelchair_fusion_nav.launch.py` (modify, Task 11) | Adds a `localization` argument so Nav2 can run on a live SLAM map |
| `~/wheelchair_nav/docs/architecture.md`, `~/wheelchair_nav/progress.md` (modify, Tasks 8, 10, 11, 12) | Results and how to run |

---

## Stage 0: Models on the laptop

### Task 1: Model runtime: llama.cpp, Qwen3-VL-2B, YOLOE, VLM server script

**Files:**

- Create: `scripts/objnav_vlm_server.sh`

**Interfaces:**

- Produces: an OpenAI-compatible endpoint `http://127.0.0.1:8091/v1/chat/completions` with `GET /health`, and the `ultralytics` package in `.venv-voice`. Tasks 4, 7 and 8 rely on both.

- [ ] **Step 1: Create the branch**

```bash
cd ~/wheelchair_nav && git switch -c vlm-objnav
```

- [ ] **Step 2: Build llama.cpp with CUDA**

The RTX 4050 is Ada (sm_89). The CUDA 12.6 toolkit is already at `/usr/local/cuda-12.6`.

```bash
cd ~ && git clone https://github.com/ggml-org/llama.cpp && cd llama.cpp
git checkout "$(git describe --tags --abbrev=0)" && git describe --tags   # note this tag in the task report
cmake -B build -DGGML_CUDA=ON -DCMAKE_CUDA_ARCHITECTURES=89 -DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc
cmake --build build -j8 --target llama-server
ls build/bin/llama-server
```

Expected: the file exists.

- [ ] **Step 3: Download Qwen3-VL-2B-Instruct (model + projector)**

```bash
cd ~/wheelchair_nav && source .venv-voice/bin/activate
hf download Qwen/Qwen3-VL-2B-Instruct-GGUF --include "*Q4_K_M*.gguf" "mmproj*Q8_0*.gguf" \
  --local-dir ~/models/qwen3-vl-2b
ls ~/models/qwen3-vl-2b
```

Expected: one `*Q4_K_M*.gguf` and one `mmproj*Q8_0*.gguf`. If `hf` is missing, run `pip install -U huggingface_hub` and retry. If no `Q8_0` projector exists, use the `F16` one.

- [ ] **Step 4: Write the server script**

```bash
#!/usr/bin/env bash
# Serve Qwen3-VL-2B-Instruct on http://127.0.0.1:8091 for scripts/object_nav.py.
#
# The VLM only answers "is the object in the red box a <target>?" for detector hits, so it
# needs a small context and a small image budget. llama.cpp is built from source with CUDA
# (docs/superpowers/plans/2026-10-09-vlm-object-nav.md, Task 1); set LLAMA and MODELS to
# override the default paths (e.g. on the Jetson).
set -euo pipefail

LLAMA=${LLAMA:-$HOME/llama.cpp/build/bin/llama-server}
MODELS=${MODELS:-$HOME/models/qwen3-vl-2b}
MMPROJ=$(ls "$MODELS"/mmproj*.gguf | head -1)
MODEL=$(ls "$MODELS"/*Q4_K_M*.gguf | grep -v mmproj | head -1)

exec "$LLAMA" --model "$MODEL" --mmproj "$MMPROJ" -ngl 99 -c 4096 \
  --port 8091 --host 127.0.0.1 --no-webui \
  --image-min-tokens 64 --image-max-tokens 512
```

Save it as `scripts/objnav_vlm_server.sh`, then make it executable with `chmod +x scripts/objnav_vlm_server.sh`.

- [ ] **Step 5: Start the server and check health**

```bash
~/wheelchair_nav/src/wheelchair_description/scripts/objnav_vlm_server.sh   # terminal A, leave running
curl -s http://127.0.0.1:8091/health                                        # terminal B
```

Expected: `{"status":"ok"}`. If llama-server rejects `--image-min-tokens` or `--image-max-tokens` as unknown, remove both flags and note it in the report.

- [ ] **Step 6: Install YOLOE into the voice venv**

```bash
cd ~/wheelchair_nav && source .venv-voice/bin/activate
pip install ultralytics "numpy<2"
python -c "import numpy, torch, ultralytics; print(numpy.__version__, torch.cuda.is_available(), ultralytics.__version__)"
python -c "import rclpy, sensor_msgs.msg; print('ros ok')"
```

Expected: numpy `1.26.x`, `True`, a version string, then `ros ok`. If numpy shows 2.x, run `pip install "numpy<2"` again. ROS message imports break on numpy 2.

- [ ] **Step 7: First YOLOE run (downloads weights and the text encoder once)**

```bash
python - <<'EOF'
from ultralytics import YOLOE
m = YOLOE("yoloe-26s-seg.pt")
m.set_classes(["bus", "person"])
r = m.predict("https://ultralytics.com/images/bus.jpg", verbose=False)[0]
print(len(r.boxes), [r.names[int(c)] for c in r.boxes.cls])
EOF
```

Expected: at least one `bus` and several `person` boxes. The weights land in the working directory, so move `yoloe-26s-seg.pt` to `~/models/` and pass that path from now on. If `yoloe-26s-seg.pt` cannot be found, the installed Ultralytics release predates YOLOE-26: use `yoloe-11s-seg.pt` and record the substitution.

- [ ] **Step 8: Commit**

```bash
git add src/wheelchair_description/scripts/objnav_vlm_server.sh
git commit -m "objnav: llama-server script for Qwen3-VL-2B verifier"
```

---

## Stage 1: Approach A in simulation

### Task 2: Geometry module

**Files:**

- Create: `scripts/objnav_geometry.py`
- Test: `test/test_objnav_geometry.py`

**Interfaces:**

- Produces:
  - `Grid(data: np.ndarray[int8, (rows, cols)], res: float, ox: float, oy: float)` with `.cell(x, y) -> (row, col)`, `.world(row, col) -> (x, y)` and `.value(x, y, default=-1) -> int`
  - `is_free(costmap: Grid, x, y) -> bool`
  - `object_depth(depth: np.ndarray[float, HxW], mask: np.ndarray[bool, HxW]) -> float | None`
  - `box_mask(shape, box) -> np.ndarray[bool]`
  - `mask_centre(mask) -> (u, v)`
  - `pixel_to_optical(u, v, z, k) -> np.ndarray(3)`
  - `quat_to_matrix(x, y, z, w) -> np.ndarray(3, 3)`
  - `standoff(obj, chair, costmap) -> (x, y, yaw) | None`
  - `Hit(xy: np.ndarray(2), view: (x, y, yaw), cam: str, frame)`
  - `Hits` with `.add(hit)`, `.ready() -> (centre: np.ndarray(2), newest: Hit) | None` and `.reject(xy)`
  - Constants `MIN_DEPTH`, `MAX_DEPTH`, `STANDOFF` and `FREE_COST`

- [ ] **Step 1: Write the failing tests**

`test/test_objnav_geometry.py`:

```python
import math

import numpy as np
import pytest

from objnav_geometry import (Grid, Hit, Hits, box_mask, mask_centre, object_depth,
                             pixel_to_optical, quat_to_matrix, standoff)


def test_grid_cell_floors_with_negative_origin():
    g = Grid(np.zeros((10, 10), np.int8), 0.1, -0.5, -0.5)
    assert g.cell(-0.45, -0.45) == (0, 0)
    assert g.cell(-0.01, 0.0) == (5, 4)


def test_grid_value_outside_is_unknown_with_negative_origin():
    g = Grid(np.zeros((10, 10), np.int8), 0.1, -0.5, -0.5)
    assert g.value(-0.55, 0.0) == -1  # int() would truncate -0.5 to cell 0 and report free
    assert g.value(0.0, 0.0) == 0


def test_object_depth_uses_near_quartile():
    depth = np.full((10, 10), 3.0)
    depth[:5] = 1.0
    assert object_depth(depth, np.ones((10, 10), bool)) == pytest.approx(1.0)


def test_object_depth_rejects_far_and_invalid():
    mask = np.ones((10, 10), bool)
    assert object_depth(np.full((10, 10), np.nan), mask) is None
    assert object_depth(np.full((10, 10), 6.0), mask) is None
    assert object_depth(np.full((10, 10), 0.1), mask) is None
    few = np.zeros((10, 10), bool)
    few[0, :5] = True  # 5 pixels: too few to trust
    assert object_depth(np.full((10, 10), 2.0), few) is None


def test_box_mask_is_central_half_and_centre_matches():
    m = box_mask((100, 100), [20, 40, 60, 80])
    ys, xs = np.nonzero(m)
    assert xs.min() == 30 and xs.max() == 50 and ys.min() == 50 and ys.max() == 70
    assert mask_centre(m) == pytest.approx((40.0, 60.0))


def test_pixel_to_optical():
    k = [500.0, 0, 320.0, 0, 500.0, 240.0, 0, 0, 1]
    assert pixel_to_optical(320, 240, 2.0, k) == pytest.approx([0, 0, 2.0])
    assert pixel_to_optical(820, 240, 2.0, k) == pytest.approx([2.0, 0, 2.0])


def test_quat_to_matrix_yaw_90():
    s = math.sin(math.pi / 4)
    r = quat_to_matrix(0, 0, s, s)
    assert r @ np.array([1.0, 0, 0]) == pytest.approx([0, 1.0, 0])
    assert quat_to_matrix(0, 0, 0, 1) == pytest.approx(np.eye(3))


def test_standoff_faces_object_from_first_free_spot():
    g = Grid(np.full((100, 100), 100, np.int8), 0.1, 0.0, 0.0)
    g.data[48:53, 29:34] = 0  # only around (3.1, 5.0) is free
    x, y, yaw = standoff((5.0, 5.0), (1.0, 5.0), g)
    assert (x, y, yaw) == pytest.approx((3.1, 5.0, 0.0))


def test_standoff_none_when_boxed_in():
    g = Grid(np.full((100, 100), 100, np.int8), 0.1, 0.0, 0.0)
    assert standoff((5.0, 5.0), (1.0, 5.0), g) is None


def hit(x, y, view=(0.0, 0.0, 0.0), cam="camera"):
    return Hit(np.array([x, y]), view, cam, None)


def test_one_viewpoint_is_not_enough():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0))
    assert h.ready() is None


def test_second_camera_confirms():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0, cam="right_camera"))
    centre, newest = h.ready()
    assert centre == pytest.approx([3.05, 0.0])
    assert newest.cam == "right_camera"


def test_moved_or_turned_viewpoint_confirms():
    moved = Hits()
    moved.add(hit(3.0, 0.0))
    moved.add(hit(3.0, 0.1, view=(0.4, 0.0, 0.0)))
    assert moved.ready() is not None
    turned = Hits()
    turned.add(hit(3.0, 0.0))
    turned.add(hit(3.0, 0.1, view=(0.0, 0.0, 0.5)))
    assert turned.ready() is not None


def test_far_apart_hits_do_not_cluster():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(5.0, 0.0, cam="right_camera"))
    assert h.ready() is None


def test_rejected_spot_ignores_new_hits():
    h = Hits()
    h.add(hit(3.0, 0.0))
    h.add(hit(3.1, 0.0, cam="right_camera"))
    centre, _ = h.ready()
    h.reject(centre)
    h.add(hit(3.0, 0.1, cam="mapping_camera"))
    h.add(hit(3.1, 0.1, view=(1.0, 0.0, 0.0)))
    assert h.ready() is None
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_geometry.py -q`
Expected: collection fails with `ModuleNotFoundError: No module named 'objnav_geometry'`.

- [ ] **Step 3: Implement**

`scripts/objnav_geometry.py`:

```python
"""Geometry for object search: map grids, depth to map points, standoff goals, hit clusters.

No ROS and no models in here, so all of it is unit-tested with plain numpy
(test/test_objnav_geometry.py).
"""
import math
from dataclasses import dataclass

import numpy as np

MIN_DEPTH, MAX_DEPTH = 0.3, 5.0  # m; stereo depth error grows fast past ~5 m
STANDOFF = (1.5, 1.9, 2.4)  # m short of the object; sim places needed >= 1.3 m clearance
FREE_COST = 50  # global costmap cells below this count as free for a goal


@dataclass
class Grid:
    """An OccupancyGrid as numpy: data[row, col], row along y; (ox, oy) is the corner of cell (0, 0)."""
    data: np.ndarray
    res: float
    ox: float
    oy: float

    def cell(self, x, y):
        # floor, not int(): the live SLAM map has cells at negative coordinates
        return math.floor((y - self.oy) / self.res), math.floor((x - self.ox) / self.res)

    def world(self, row, col):
        return self.ox + (col + 0.5) * self.res, self.oy + (row + 0.5) * self.res

    def value(self, x, y, default=-1):
        r, c = self.cell(x, y)
        if 0 <= r < self.data.shape[0] and 0 <= c < self.data.shape[1]:
            return int(self.data[r, c])
        return default


def is_free(costmap, x, y):
    return 0 <= costmap.value(x, y) < FREE_COST


def object_depth(depth, mask):
    """Near-quartile depth of the object's pixels, or None if fewer than 20 are valid.
    The near quartile skips the background seen through and around the object."""
    d = depth[mask]
    d = d[np.isfinite(d) & (d > MIN_DEPTH) & (d < MAX_DEPTH)]
    if d.size < 20:
        return None
    return float(np.percentile(d, 25))


def box_mask(shape, box):
    """Central half of a box: the fallback when the detector gives no mask."""
    x1, y1, x2, y2 = box
    w, h = x2 - x1, y2 - y1
    m = np.zeros(shape, bool)
    m[int(y1 + h / 4):int(y2 - h / 4) + 1, int(x1 + w / 4):int(x2 - w / 4) + 1] = True
    return m


def mask_centre(mask):
    ys, xs = np.nonzero(mask)
    return float(xs.mean()), float(ys.mean())


def pixel_to_optical(u, v, z, k):
    """Back-project pixel (u, v) at depth z with CameraInfo.k (3x3, row-major)."""
    return np.array([(u - k[2]) / k[0] * z, (v - k[5]) / k[4] * z, z])


def quat_to_matrix(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def standoff(obj, chair, costmap):
    """First free spot short of the object, trying wider angles and then longer distances.
    Returns (x, y, yaw) with yaw facing the object, or None."""
    base = math.atan2(obj[1] - chair[1], obj[0] - chair[0])
    for dist in STANDOFF:
        for dth in (0.0, 0.5, -0.5, 1.0, -1.0):
            a = base + dth
            x, y = obj[0] - dist * math.cos(a), obj[1] - dist * math.sin(a)
            if is_free(costmap, x, y):
                return x, y, a
    return None


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class Hit:
    xy: np.ndarray  # object position in the map frame
    view: tuple  # chair (x, y, yaw) when it was seen
    cam: str
    frame: object  # (rgb, box), kept for the VLM check


class Hits:
    """Detector hits grouped by map position. A cluster is ready for the VLM check only once
    two hits from different viewpoints agree, so one bad box in one frame never moves the chair."""

    MAX_HITS = 60  # ponytail: each hit keeps its RGB frame (~1 MB); keep the newest 60

    def __init__(self, radius=0.5, min_baseline=0.3, min_turn=0.3, reject_radius=0.7):
        self.radius, self.min_baseline, self.min_turn = radius, min_baseline, min_turn
        self.reject_radius = reject_radius
        self.hits, self.rejected = [], []

    def add(self, hit):
        if any(np.hypot(*(hit.xy - r)) < self.reject_radius for r in self.rejected):
            return
        self.hits = (self.hits + [hit])[-self.MAX_HITS:]

    def ready(self):
        for h in reversed(self.hits):
            near = [o for o in self.hits if np.hypot(*(o.xy - h.xy)) < self.radius]
            for o in near:
                moved = math.hypot(o.view[0] - h.view[0], o.view[1] - h.view[1]) >= self.min_baseline
                turned = abs(wrap(o.view[2] - h.view[2])) >= self.min_turn
                if moved or turned or o.cam != h.cam:
                    return np.mean([n.xy for n in near], axis=0), h
        return None

    def reject(self, xy):
        xy = np.asarray(xy)
        self.rejected.append(xy)
        self.hits = [h for h in self.hits if np.hypot(*(h.xy - xy)) >= self.reject_radius]
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_geometry.py -q`
Expected: `14 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/wheelchair_description/scripts/objnav_geometry.py src/wheelchair_description/test/test_objnav_geometry.py
git commit -m "objnav: geometry for depth-to-map points, standoff goals and hit clusters"
```

### Task 3: Frontier module

**Files:**

- Create: `scripts/objnav_frontiers.py`
- Test: `test/test_objnav_frontiers.py`

**Interfaces:**

- Consumes: `Grid` and `is_free` from Task 2.
- Produces:
  - `Frontier(x: float, y: float, size: float)`
  - `find_frontiers(grid: Grid, costmap: Grid, min_size=0.6, search=1.0) -> list[Frontier]`
  - `nearest_free(costmap: Grid, x, y, radius) -> (x, y) | None`
  - `pick(frontiers, chair: (x, y), tried: list[(x, y)], value=None) -> Frontier | None`, where `value` is `callable(x, y) -> float`, NaN meaning unseen

- [ ] **Step 1: Write the failing tests**

`test/test_objnav_frontiers.py`:

```python
import numpy as np
import pytest

from objnav_frontiers import Frontier, find_frontiers, pick
from objnav_geometry import Grid


def grid(data, res=0.1):
    return Grid(np.array(data, np.int8), res, 0.0, 0.0)


def half_known():
    d = np.full((40, 40), -1, np.int8)
    d[:, :20] = 0  # x < 2.0 m is known free, the rest is unknown
    return d


def test_one_frontier_along_the_known_edge():
    fs = find_frontiers(grid(half_known()), grid(np.zeros((40, 40))))
    assert len(fs) == 1
    assert fs[0].x == pytest.approx(1.95)
    assert fs[0].y == pytest.approx(2.0, abs=0.1)
    assert fs[0].size == pytest.approx(4.0)


def test_wall_between_free_and_unknown_is_not_a_frontier():
    d = half_known()
    d[:, 19] = 100
    assert find_frontiers(grid(d), grid(np.zeros((40, 40)))) == []


def test_single_unknown_speckle_is_ignored():
    d = np.zeros((40, 40), np.int8)
    d[10, 10] = -1
    assert find_frontiers(grid(d), grid(np.zeros((40, 40)))) == []


def test_goal_moves_to_a_costmap_free_cell():
    cost = np.full((40, 40), 100, np.int8)
    cost[:, 15] = 0  # only the column at x = 1.55 is free
    fs = find_frontiers(grid(half_known()), grid(cost))
    assert fs[0].x == pytest.approx(1.55)


def test_no_goal_when_nothing_free_nearby():
    assert find_frontiers(grid(half_known()), grid(np.full((40, 40), 100))) == []


NEAR, FAR = Frontier(1.0, 0.0, 2.0), Frontier(5.0, 0.0, 2.0)


def test_pick_prefers_close_over_far_of_same_size():
    assert pick([FAR, NEAR], (0.0, 0.0), []) is NEAR


def test_pick_skips_tried_and_frontiers_under_the_chair():
    assert pick([NEAR, FAR], (0.0, 0.0), [(1.1, 0.0)]) is FAR
    assert pick([Frontier(0.2, 0.0, 5.0)], (0.0, 0.0), []) is None


def test_value_can_outweigh_distance():
    def value(x, y):
        return 0.9 if x > 3 else 0.1
    assert pick([FAR, NEAR], (0.0, 0.0), [], value) is FAR


def test_unseen_value_is_neutral():
    def value(x, y):
        return float("nan") if x > 3 else 0.5
    assert pick([FAR, NEAR], (0.0, 0.0), [], value) is NEAR
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_frontiers.py -q`
Expected: `ModuleNotFoundError: No module named 'objnav_frontiers'`.

- [ ] **Step 3: Implement**

`scripts/objnav_frontiers.py`:

```python
"""Frontiers of the live SLAM map: known free cells next to unknown space.

Approach A picks the frontier that is long and close; approach B also weighs how much the area
around it looks like the target (objnav_value.ValueMap). Pure numpy/scipy, unit-tested in
test/test_objnav_frontiers.py.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from objnav_geometry import is_free

OCCUPIED = 65  # slam_toolbox publishes 0 free, 100 occupied, -1 unknown
CROSS = np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]], bool)


@dataclass
class Frontier:
    x: float  # goal point, free in the global costmap
    y: float
    size: float  # frontier length, m


def nearest_free(costmap, x, y, radius):
    steps = int(radius / costmap.res)
    offsets = sorted(((i, j) for i in range(-steps, steps + 1) for j in range(-steps, steps + 1)),
                     key=lambda o: o[0] ** 2 + o[1] ** 2)
    for i, j in offsets:
        gx, gy = x + i * costmap.res, y + j * costmap.res
        if is_free(costmap, gx, gy):
            return gx, gy
    return None


def find_frontiers(grid, costmap, min_size=0.6, search=1.0):
    """min_size 0.6 m keeps doorway frontiers (~0.8 m) and drops one-cell speckles (0.4 m)."""
    d = grid.data
    free = (d >= 0) & (d < OCCUPIED)
    edge = free & ndimage.binary_dilation(d == -1, structure=CROSS)
    labels, n = ndimage.label(edge, structure=np.ones((3, 3), bool))
    out = []
    for i in range(1, n + 1):  # ponytail: one full-grid pass per frontier; fine at 0.05 m up to ~50 x 50 m
        rows, cols = np.nonzero(labels == i)
        size = len(rows) * grid.res
        if size < min_size:
            continue
        k = int(np.argmin((rows - rows.mean()) ** 2 + (cols - cols.mean()) ** 2))
        goal = nearest_free(costmap, *grid.world(rows[k], cols[k]), search)
        if goal is not None:
            out.append(Frontier(goal[0], goal[1], size))
    return out


def pick(frontiers, chair, tried, value=None, tried_radius=1.0):
    """Best frontier by size / (distance + 1); with a value function, also by how much its
    area looks like the target. Frontiers already tried or under the chair are skipped."""
    cands = []
    for f in frontiers:
        if any(math.hypot(f.x - t[0], f.y - t[1]) < tried_radius for t in tried):
            continue
        d = math.hypot(f.x - chair[0], f.y - chair[1])
        if d >= 0.5:
            cands.append((f, f.size / (d + 1.0)))
    if not cands:
        return None
    if value is not None:
        vals = [value(f.x, f.y) for f, _ in cands]
        seen = [v for v in vals if not math.isnan(v)]
        lo, hi = (min(seen), max(seen)) if seen else (0.0, 0.0)
        norm = [0.5 if math.isnan(v) else (v - lo) / (hi - lo) if hi > lo else 1.0 for v in vals]
        cands = [(f, s * (0.2 + 0.8 * w)) for (f, s), w in zip(cands, norm)]
    return max(cands, key=lambda c: c[1])[0]
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_frontiers.py -q`
Expected: `9 passed`.

- [ ] **Step 5: Commit**

```bash
git add src/wheelchair_description/scripts/objnav_frontiers.py src/wheelchair_description/test/test_objnav_frontiers.py
git commit -m "objnav: frontier extraction and choice on the live SLAM map"
```

### Task 4: Model wrappers and smoke check

**Files:**

- Create: `scripts/objnav_models.py`, `scripts/objnav_smoke.py`
- Test: `test/test_objnav_models.py`

**Interfaces:**

- Consumes: the Task 1 server at `http://127.0.0.1:8091` and `ultralytics` in `.venv-voice`.
- Produces:
  - `Detector(weights: str, device: str, conf=0.25)` with `.set_target(name)` and `.detect(rgb: HxWx3 uint8 RGB) -> list[(box: [x1, y1, x2, y2], score: float, mask: np.ndarray[bool, HxW] | None)]`
  - `Verifier(url: str, timeout=20)` with `.check(rgb, box, target) -> bool` (any error counts as False) and `.ping() -> bool`
  - `parse_yes(text) -> bool`
  - `boxed_jpeg(rgb, box, width=640) -> base64 str`

- [ ] **Step 1: Write the failing tests**

`test/test_objnav_models.py`:

```python
import base64

import cv2
import numpy as np

import objnav_models as m


def test_parse_yes():
    assert m.parse_yes("Yes.")
    assert m.parse_yes(' "yes" ')
    assert m.parse_yes("**Yes**")
    assert not m.parse_yes("No")
    assert not m.parse_yes("")


def test_boxed_jpeg_is_640_wide_with_red_box():
    jpg = base64.b64decode(m.boxed_jpeg(np.zeros((960, 1280, 3), np.uint8), [10, 10, 200, 200]))
    img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[:2] == (480, 640)
    b, g, r = img[5, 50]  # top edge of the box after scaling by 0.5
    assert r > 150 and g < 100 and b < 100


class Reply:
    def __init__(self, text):
        self.text = text

    def raise_for_status(self):
        pass

    def json(self):
        return {"choices": [{"message": {"content": self.text}}]}


def test_verifier_sends_image_and_reads_yes(monkeypatch):
    sent = {}

    def post(url, json, timeout):
        sent.update(json)
        return Reply("Yes")

    monkeypatch.setattr(m.requests, "post", post)
    assert m.Verifier("http://x/v1/chat/completions").check(
        np.zeros((480, 640, 3), np.uint8), [1, 1, 50, 50], "sofa") is True
    content = sent["messages"][0]["content"]
    assert content[0]["image_url"]["url"].startswith("data:image/jpeg;base64,")
    assert "sofa" in content[1]["text"]
    assert sent["temperature"] == 0


def test_verifier_error_counts_as_no(monkeypatch):
    def post(*a, **k):
        raise m.requests.ConnectionError("down")

    monkeypatch.setattr(m.requests, "post", post)
    assert m.Verifier("http://x/v1/chat/completions").check(
        np.zeros((480, 640, 3), np.uint8), [1, 1, 50, 50], "sofa") is False


def test_ping_false_when_server_down(monkeypatch):
    def get(*a, **k):
        raise m.requests.ConnectionError("down")

    monkeypatch.setattr(m.requests, "get", get)
    assert m.Verifier("http://x/v1/chat/completions").ping() is False
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_models.py -q`
Expected: `ModuleNotFoundError: No module named 'objnav_models'`.

- [ ] **Step 3: Implement the wrappers**

`scripts/objnav_models.py`:

```python
"""Models for object search. Heavy imports (torch, ultralytics, open_clip) happen inside the
classes, so the unit tests need no GPU.

Detector: YOLOE with the spoken word as its only class. It stays in PyTorch because a
TensorRT export bakes the class list into the weights.
Verifier: Qwen3-VL-2B on scripts/objnav_vlm_server.sh. It sees the full frame with the box
drawn in red, not a crop; the crop-only check let a wrong "refrigerator" through on 2026-10-04.
"""
import base64
import logging

import cv2
import requests

log = logging.getLogger("objnav_models")

VERIFY_PROMPT = ("Look at the object inside the red box. Is it a {target}? "
                 "Answer only yes or no.")


class Detector:
    def __init__(self, weights, device, conf=0.25):
        from ultralytics import YOLOE
        self.model, self.device, self.conf = YOLOE(weights), device, conf

    def set_target(self, name):
        self.model.set_classes([name])

    def detect(self, rgb):
        r = self.model.predict(rgb[:, :, ::-1], conf=self.conf, device=self.device,
                               half=self.device.startswith("cuda"), retina_masks=True,
                               verbose=False)[0]
        out = []
        for i, box in enumerate(r.boxes.xyxy.cpu().numpy()):
            mask = r.masks.data[i].cpu().numpy() > 0.5 if r.masks is not None else None
            out.append((box.tolist(), float(r.boxes.conf[i]), mask))
        return out


def boxed_jpeg(rgb, box, width=640):
    img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    x1, y1, x2, y2 = (int(v) for v in box)
    cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 255), max(2, img.shape[1] // 160))
    s = width / img.shape[1]
    img = cv2.resize(img, (width, int(round(img.shape[0] * s))))
    return base64.b64encode(cv2.imencode(".jpg", img)[1]).decode()


def parse_yes(text):
    return text.strip().lstrip("\"'*` ").lower().startswith("yes")


class Verifier:
    def __init__(self, url, timeout=20):
        self.url, self.timeout = url, timeout

    def ping(self):
        try:
            return requests.get(self.url.replace("/v1/chat/completions", "/health"),
                                timeout=2).status_code == 200
        except requests.RequestException:
            return False

    def check(self, rgb, box, target):
        """True only on a clear yes. Errors count as no (ponytail: a server hiccup can reject a
        real object; the search then keeps exploring and sees it again)."""
        try:
            r = requests.post(self.url, timeout=self.timeout, json={
                "temperature": 0, "max_tokens": 8,
                "messages": [{"role": "user", "content": [
                    {"type": "image_url",
                     "image_url": {"url": "data:image/jpeg;base64," + boxed_jpeg(rgb, box)}},
                    {"type": "text", "text": VERIFY_PROMPT.format(target=target)}]}]})
            r.raise_for_status()
            return parse_yes(r.json()["choices"][0]["message"]["content"])
        except (requests.RequestException, KeyError, IndexError, ValueError) as e:
            log.error("VLM check failed: %s", e)
            return False
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_models.py -q`
Expected: `5 passed`.

- [ ] **Step 5: Write the smoke script**

`scripts/objnav_smoke.py`:

```python
#!/usr/bin/env python3
"""Check the object-search models on one image without ROS: boxes, VLM verdicts, timings.
Run on the laptop and again on the Jetson; the numbers decide the model sizes.

    python3 scripts/objnav_smoke.py photo.jpg sofa --weights ~/models/yoloe-26s-seg.pt
"""
import argparse
import time

import cv2

from objnav_models import Detector, Verifier


def main():
    p = argparse.ArgumentParser()
    p.add_argument("image")
    p.add_argument("target")
    p.add_argument("--weights", default="yoloe-26s-seg.pt")
    p.add_argument("--device", default="cuda:0")
    p.add_argument("--url", default="http://127.0.0.1:8091/v1/chat/completions")
    a = p.parse_args()

    rgb = cv2.cvtColor(cv2.imread(a.image), cv2.COLOR_BGR2RGB)
    det = Detector(a.weights, a.device)
    t = time.perf_counter()
    det.set_target(a.target)
    print(f"set_target: {(time.perf_counter() - t) * 1000:.0f} ms")
    det.detect(rgb)  # warm-up
    t, n = time.perf_counter(), 20
    for _ in range(n):
        dets = det.detect(rgb)
    print(f"detector: {(time.perf_counter() - t) / n * 1000:.0f} ms/frame, {len(dets)} boxes")

    ver = Verifier(a.url)
    print(f"VLM server up: {ver.ping()}")
    for box, score, _ in dets[:3]:
        t = time.perf_counter()
        ok = ver.check(rgb, box, a.target)
        print(f"box {[round(v) for v in box]} score {score:.2f} -> VLM {'yes' if ok else 'no'}, "
              f"{(time.perf_counter() - t) * 1000:.0f} ms")


if __name__ == "__main__":
    main()
```

- [ ] **Step 6: Run the smoke check on the laptop (VLM server from Task 1 running)**

```bash
cd ~/wheelchair_nav/src/wheelchair_description && source ~/wheelchair_nav/.venv-voice/bin/activate
curl -sL -o $TMPDIR/bus.jpg https://ultralytics.com/images/bus.jpg
python3 scripts/objnav_smoke.py $TMPDIR/bus.jpg bus --weights ~/models/yoloe-26s-seg.pt
python3 scripts/objnav_smoke.py $TMPDIR/bus.jpg sofa --weights ~/models/yoloe-26s-seg.pt
```

Expected for `bus`: at least 1 box with VLM `yes`. Expected for `sofa`: either 0 boxes, or every box gets VLM `no`. Gates on the RTX 4050: the detector at or under 60 ms/frame, and each VLM check at or under 1500 ms after the first call. Record all printed numbers in the task report. If a gate fails, report it and do not change the models without asking.

- [ ] **Step 7: Commit**

```bash
git add src/wheelchair_description/scripts/objnav_models.py src/wheelchair_description/scripts/objnav_smoke.py src/wheelchair_description/test/test_objnav_models.py
git commit -m "objnav: YOLOE detector and Qwen3-VL verifier wrappers with smoke check"
```

### Task 5: Hook in voice_nav.py for targets that are not saved places

**Files:**

- Modify: `scripts/voice_nav.py:46-48` (constructor) and `scripts/voice_nav.py:89-91` (unknown target)

**Interfaces:**

- Produces:
  - `VoiceNav.__init__(self, name="voice_nav")`
  - `VoiceNav.on_unknown(self, target: str, names: dict[str, str])`, which by default says "I don't know …". Task 7 overrides it.
  - Also used from outside the class: `wait(future)`, `listen(node, stt)`, `self.say(text)`, `self.say_async(text)`, `self.send(client, goal) -> status | None`, `self.face(yaw)`, `self.cancelled` (`threading.Event`), `self.busy`, `self.goal_handle`, `self.tf`, `self.nav`, `self.spin_client` and `self.places`.

- [ ] **Step 1: Make the edits**

Replace:

```python
class VoiceNav(Node):
    def __init__(self):
        super().__init__("voice_nav")
```

with:

```python
class VoiceNav(Node):
    def __init__(self, name="voice_nav"):
        super().__init__(name)
```

Replace:

```python
        if not match:
            self.say_async(f"I don't know {target}. I know " + ", ".join(names.values()) + ".")
            return
```

with:

```python
        if not match:
            self.on_unknown(target, names)
            return
```

Then add this method directly after `on_text`:

```python
    def on_unknown(self, target, names):
        """No saved place matches. object_nav.py overrides this to search with the cameras."""
        self.say_async(f"I don't know {target}. I know " + ", ".join(names.values()) + ".")
```

- [ ] **Step 2: Check that phase 1 behaviour is unchanged**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && /usr/bin/python3 -m py_compile scripts/voice_nav.py && echo ok`
Expected: `ok`. The behaviour itself is re-checked in the Task 8 protocol, row "phase 1 regression".

- [ ] **Step 3: Commit**

```bash
git add src/wheelchair_description/scripts/voice_nav.py
git commit -m "voice_nav: on_unknown hook for targets that are not saved places"
```

### Task 6: Simulation explore mode (live SLAM + Nav2, no AMCL)

**Files:**

- Modify: `launch/gazebo_sim.launch.py:72`, `:77-84`, `:438-444`, `:461-466`, `:470-475`, and the `LaunchDescription` list near `:517`

**Interfaces:**

- Produces: `gazebo_sim.launch.py nav2:=true explore:=true`. slam_toolbox publishes `/map` and `map -> odom`, and Nav2 runs without `map_server` and without `amcl`.

- [ ] **Step 1: Add the argument and the conditions**

After `bt_xml = LaunchConfiguration("bt_xml")`, add:

```python
    explore = LaunchConfiguration("explore")
```

Replace the `slam_ready` expression with:

```python
    # SLAM needs odom -> base_link (the EKF) plus the bridged /scan. In explore mode it runs
    # alongside Nav2 and replaces map_server + AMCL.
    slam_ready = PythonExpression([
        "'", ekf, "' == 'true' and (('", slam, "' == 'true' and '", nav2, "' != 'true') or '",
        explore, "' == 'true')",
    ])
```

After the `nav2_ready` expression, add:

```python
    # map_server + AMCL only when Nav2 drives on a saved map.
    amcl_ready = PythonExpression([
        "'", nav2, "' == 'true' and '", ekf, "' == 'true' and '", bridge_lidar,
        "' == 'true' and '", explore, "' != 'true'",
    ])
```

After `declare_collision_monitor = DeclareLaunchArgument(...)`, add:

```python
    declare_explore = DeclareLaunchArgument(
        "explore", default_value="false",
        description="Explore an unmapped world: slam_toolbox builds /map live and Nav2 plans "
                    "on it, with no map_server and no AMCL. Use with nav2:=true "
                    "(scripts/object_nav.py)."
    )
```

In `map_server = Node(...)` and in `amcl_node = LifecycleNode(...)`, change `condition=IfCondition(nav2_ready)` to `condition=IfCondition(amcl_ready)`.

In `nav2_lifecycle_manager`, replace the `node_names` expression with:

```python
        parameters=[nav2_common, {"autostart": True, "node_names": PythonExpression([
            "(['map_server', 'amcl'] if '", explore, "' != 'true' else []) + ",
            "['controller_server', 'smoother_server', ",
            "'planner_server', 'behavior_server', 'bt_navigator', ",
            "'velocity_smoother', 'waypoint_follower'] + ",
            "(['collision_monitor'] if '", use_collision_monitor, "' == 'true' else [])",
        ])}],
```

In the returned `LaunchDescription([...])`, add `declare_explore,` directly after `declare_collision_monitor,`.

- [ ] **Step 2: Build**

```bash
cd ~/wheelchair_nav && source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --build-base build_src --install-base install_src \
  --packages-select wheelchair_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 3: Launch explore mode and check it**

Terminal 1 (setup per `progress.md`, "Run"):

```bash
ros2 launch wheelchair_description gazebo_sim.launch.py world_name:=small_house use_rviz:=false \
  nav2:=true explore:=true bridge_camera:=true \
  nav2_params_file:=$HOME/wheelchair_nav/src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml \
  bt_xml:=$HOME/wheelchair_nav/src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v5.xml
```

Terminal 2, after about 30 s:

```bash
ros2 node list | grep -E "/amcl|/map_server" || echo "no amcl, no map_server"
for n in slam_toolbox controller_server planner_server behavior_server bt_navigator velocity_smoother; do printf '%s: ' $n; ros2 lifecycle get /$n; done
ros2 run tf2_ros tf2_echo map odom --ros-args -p use_sim_time:=true   # Ctrl+C after one print
ros2 topic echo --once /camera/aligned_depth_to_color/image_raw --field header.frame_id
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.5, y: 0.0}, orientation: {w: 1.0}}}}"
```

Expected:

- `no amcl, no map_server`.
- Every node prints `active [3]`.
- `tf2_echo` prints a transform.
- The depth `frame_id` ends in `_optical_frame`.
- The goal finishes `SUCCEEDED`, and the map has grown (`ros2 topic echo --once /map --field info.width` is larger after the trip than before it).

If the goal is rejected because the start is in unknown space, report it. The planner's `allow_unknown` is the setting to inspect.

- [ ] **Step 4: Check that saved-map mode is unchanged**

Restart terminal 1 with the `progress.md` command (`nav2:=true map:=...`, no `explore`) and check that `ros2 lifecycle get /amcl` prints `active [3]`.

- [ ] **Step 5: Commit**

```bash
git add src/wheelchair_description/launch/gazebo_sim.launch.py
git commit -m "gazebo_sim: explore:=true runs slam_toolbox with Nav2 instead of map_server and AMCL"
```

### Task 7: object_nav.py node (approach A)

**Files:**

- Create: `scripts/object_nav.py`
- Modify: `CMakeLists.txt:15-17`

**Interfaces:**

- Consumes:
  - Task 2: `Grid`, `Hit`, `Hits`, `box_mask`, `mask_centre`, `object_depth`, `pixel_to_optical`, `quat_to_matrix`, `standoff` and `STANDOFF`.
  - Task 3: `find_frontiers` and `pick`.
  - Task 4: `Detector` and `Verifier`.
  - Task 5: `VoiceNav(name)`, `on_unknown`, `listen` and `wait`.
- Produces:
  - Executable `ros2 run wheelchair_description object_nav.py` with parameters `detector` (str), `device` (str) and `vlm_url` (str).
  - One log line per command: `SEARCH_RESULT target=<t> found=<bool> time=<s>s path=<m>m`.
  - Methods that Task 10 extends: `look(hits)` and `next_frontier(tried)`.

- [ ] **Step 1: Write the node**

`scripts/object_nav.py`:

```python
#!/usr/bin/env python3
"""Zero-shot object search in an unmapped building: "take me to the sofa" -> find it, drive there.

Needs slam_toolbox (live /map) and Nav2 without AMCL (gazebo_sim.launch.py explore:=true), plus
scripts/objnav_vlm_server.sh. Reuses voice_nav.py for the mic, "stop" and spoken replies; saved
places are switched off, because they belong to a different map.

Per command: announce, then look with the three RGB-D cameras (YOLOE with the spoken word as its
only class) and explore frontiers of the live map until two viewpoints agree on the object.
Qwen3-VL confirms it on the full frame, and the chair drives to a free spot 1.5-2.4 m short of it
and turns to face it. Nav2 does all the driving.

    ros2 run wheelchair_description object_nav.py --ros-args -p use_sim_time:=true \
        -p detector:=$HOME/models/yoloe-26s-seg.pt
    ros2 topic pub --once /voice/transcript std_msgs/String "{data: 'take me to the sofa'}"
"""
import math
import threading
import time

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from faster_whisper import WhisperModel
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, Spin
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from objnav_frontiers import find_frontiers, pick
from objnav_geometry import (STANDOFF, Grid, Hit, Hits, box_mask, mask_centre, object_depth,
                             pixel_to_optical, quat_to_matrix, standoff)
from objnav_models import Detector, Verifier
from voice_nav import VoiceNav, listen, wait

CAMERAS = ["camera", "mapping_camera", "right_camera"]  # front, left, right (RealSense names)
SEARCH_TIME = 300.0  # s; the passenger is seated, so a search gives up after 5 minutes
MAX_TRIPS = 20  # frontier goals per command
LOOK_PERIOD = 0.5  # s between looks while a goal runs; also the stop-check period
LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def to_rgb(msg):
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[:, :, ::-1]


def to_metres(msg):
    """32FC1 metres (Gazebo) or 16UC1 millimetres (RealSense)."""
    if msg.encoding == "16UC1":
        return np.frombuffer(msg.data, np.uint16).reshape(msg.height, -1)[:, :msg.width] / 1000.0
    return np.frombuffer(msg.data, np.float32).reshape(msg.height, -1)[:, :msg.width]


def to_grid(msg):
    i = msg.info
    return Grid(np.array(msg.data, np.int8).reshape(i.height, i.width), i.resolution,
                i.origin.position.x, i.origin.position.y)


def nav_goal(x, y, yaw):
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = "map"
    g.pose.pose.position.x, g.pose.pose.position.y = float(x), float(y)
    g.pose.pose.orientation.z = math.sin(yaw / 2)
    g.pose.pose.orientation.w = math.cos(yaw / 2)
    return g


def spin_goal(angle):
    g = Spin.Goal()
    g.target_yaw = float(angle)
    g.time_allowance.sec = 30
    return g


class ObjectNav(VoiceNav):
    def __init__(self):
        super().__init__("object_nav")
        self.places = {}  # saved places belong to a saved map; here the map is built live
        self.detector = Detector(self.declare_parameter("detector", "yoloe-26s-seg.pt").value,
                                 self.declare_parameter("device", "cuda:0").value)
        self.verifier = Verifier(self.declare_parameter(
            "vlm_url", "http://127.0.0.1:8091/v1/chat/completions").value)
        self.frames = {c: {} for c in CAMERAS}
        for cam in CAMERAS:
            for key, typ, topic in (("rgb", Image, "color/image_raw"),
                                    ("depth", Image, "aligned_depth_to_color/image_raw"),
                                    ("info", CameraInfo, "color/camera_info")):
                self.create_subscription(typ, f"/{cam}/{topic}",
                                         lambda m, c=cam, k=key: self.frames[c].__setitem__(k, m),
                                         qos_profile_sensor_data)
        self.map = self.costmap = None
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), LATCHED)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                                 lambda m: setattr(self, "costmap", m), LATCHED)
        self.deadline, self.travelled = 0.0, 0.0

    def on_unknown(self, target, names):
        if self.busy:
            self.say_async("I am already moving. Say stop first.")
            return
        self.busy = True
        self.cancelled.clear()
        threading.Thread(target=self.search, args=(target,), daemon=True).start()

    # ---- one command, start to finish ---------------------------------------------------
    def search(self, target):
        start, found = time.monotonic(), False
        self.deadline, self.travelled = start + SEARCH_TIME, 0.0
        try:
            if not self.verifier.ping():
                self.say("My camera check is not running, so I can't search.")
                return
            self.say(f"Looking for the {target}. Say stop to cancel.")
            if self.cancelled.wait(2.0):
                return
            self.detector.set_target(target)
            hits, tried, trips = Hits(), [], 0
            self.look(hits)
            if hits.ready() is None:  # the cameras cover ~259 deg, so look behind once
                self.run_and_look(self.spin_client, spin_goal(math.pi), hits)
            while not self.cancelled.is_set():
                obj = self.confirmed(hits, target)
                if obj is not None:
                    found = self.approach(target, obj)
                    return
                frontier = None
                if trips < MAX_TRIPS and time.monotonic() < self.deadline:
                    frontier = self.next_frontier(tried)
                if frontier is None:
                    if not self.cancelled.is_set():
                        self.say(f"I could not find the {target}.")
                    return
                trips += 1
                chair = self.chair_pose() or (frontier.x, frontier.y, 0.0)
                yaw = math.atan2(frontier.y - chair[1], frontier.x - chair[0])
                if not self.run_and_look(self.nav, nav_goal(frontier.x, frontier.y, yaw), hits):
                    tried.append((frontier.x, frontier.y))  # reached or failed; not if a hit cut it short
        finally:
            self.get_logger().info(f"SEARCH_RESULT target={target} found={found} "
                                   f"time={time.monotonic() - start:.0f}s path={self.travelled:.1f}m")
            self.busy = False

    def confirmed(self, hits, target):
        """Map position of a cluster the VLM agrees is the target, or None."""
        while not self.cancelled.is_set():
            ready = hits.ready()
            if ready is None:
                return None
            centre, newest = ready
            rgb, box = newest.frame
            ok = self.verifier.check(rgb, box, target)
            self.get_logger().info(f"VLM check {target} at ({centre[0]:.2f}, {centre[1]:.2f}): "
                                   f"{'yes' if ok else 'no'}")
            if ok:
                return centre
            hits.reject(centre)
        return None

    def approach(self, target, obj):
        chair = self.chair_pose()
        if chair is None or self.costmap is None:
            self.say(f"I see the {target}, but I lost track of where I am.")
            return False
        dist = math.hypot(obj[0] - chair[0], obj[1] - chair[1])
        if dist < STANDOFF[0] + 0.3:
            self.face(math.atan2(obj[1] - chair[1], obj[0] - chair[0]))
            self.say(f"I am next to the {target}.")
            return True
        goal = standoff(obj, chair[:2], to_grid(self.costmap))
        if goal is None:
            self.say(f"I see the {target}, but I can't find a free spot next to it.")
            return False
        self.say(f"I found the {target}, {dist:.0f} metres away. Going there. Say stop to cancel.")
        if self.cancelled.wait(2.0):
            return False
        ok = self.send(self.nav, nav_goal(*goal)) == GoalStatus.STATUS_SUCCEEDED
        if ok:
            self.face(goal[2])
        if self.cancelled.is_set():
            return False
        self.say(f"Arrived at the {target}." if ok else
                 f"I could not reach the {target}. I may be stuck. Please help me.")
        return ok

    def run_and_look(self, client, goal, hits):
        """Run one Nav2 action while the cameras keep looking. Returns True if it was cut short
        because two viewpoints agreed on the target; stop and the time cap also cancel it."""
        if self.cancelled.is_set() or not client.wait_for_server(timeout_sec=5.0):
            return False
        handle = wait(client.send_goal_async(goal))
        if not handle.accepted:
            return False
        self.goal_handle = handle
        result, interrupted, last = handle.get_result_async(), False, self.chair_pose()
        while not result.done():
            if self.cancelled.is_set() or time.monotonic() > self.deadline:
                handle.cancel_goal_async()
                break
            self.look(hits)
            if hits.ready() is not None:
                handle.cancel_goal_async()
                interrupted = True
                break
            now = self.chair_pose()
            if now and last:
                self.travelled += math.hypot(now[0] - last[0], now[1] - last[1])
            last = now or last
            time.sleep(LOOK_PERIOD)
        wait(result)
        self.goal_handle = None
        return interrupted

    # ---- perception ---------------------------------------------------------------------
    def look(self, hits):
        view = self.chair_pose()
        if view is None:
            return
        for cam in CAMERAS:
            f = dict(self.frames[cam])  # snapshot; callbacks keep replacing entries
            if len(f) < 3:
                continue
            rgb, depth = to_rgb(f["rgb"]), to_metres(f["depth"])
            if rgb.shape[:2] != depth.shape:
                self.get_logger().warn(f"{cam}: aligned depth {depth.shape} != colour "
                                       f"{rgb.shape[:2]}", throttle_duration_sec=10.0)
                continue
            dets = self.detector.detect(rgb)
            if not dets:
                continue
            try:
                tf = self.tf.lookup_transform("map", f["depth"].header.frame_id,
                                              Time.from_msg(f["depth"].header.stamp),
                                              timeout=Duration(seconds=0.1))
            except Exception as e:  # tf2 raises several unrelated exception types
                self.get_logger().warn(f"tf {cam}: {e}", throttle_duration_sec=5.0)
                continue
            q, p = tf.transform.rotation, tf.transform.translation
            r, t = quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])
            for box, score, mask in dets:
                m = mask if mask is not None and mask.shape == depth.shape else box_mask(depth.shape, box)
                z = object_depth(depth, m)
                if z is None:
                    continue
                u, v = mask_centre(m)
                xy = (r @ pixel_to_optical(u, v, z, f["info"].k) + t)[:2]
                hits.add(Hit(xy, view, cam, (rgb, box)))
                self.get_logger().info(f"{cam}: box {[round(b) for b in box]} score {score:.2f} "
                                       f"at map ({xy[0]:.2f}, {xy[1]:.2f}), {z:.1f} m")

    def chair_pose(self):
        try:
            tf = self.tf.lookup_transform("map", "base_link", Time())
        except Exception as e:  # tf2 raises several unrelated exception types
            self.get_logger().warn(f"tf: {e}", throttle_duration_sec=5.0)
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        return t.x, t.y, 2 * math.atan2(q.z, q.w)

    def next_frontier(self, tried):
        chair = self.chair_pose()
        if chair is None or self.map is None or self.costmap is None:
            return None
        return pick(find_frontiers(to_grid(self.map), to_grid(self.costmap)), chair[:2], tried)


def main():
    rclpy.init()
    node = ObjectNav()
    stt = WhisperModel("base.en", device="cpu", compute_type="int8")
    threading.Thread(target=listen, args=(node, stt), daemon=True).start()
    node.say_async("Object search ready.")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Install the scripts**

In `CMakeLists.txt`, replace:

```cmake
install(PROGRAMS scripts/go_to_location.py scripts/odom_axle_to_base.py scripts/voice_nav.py
  DESTINATION lib/${PROJECT_NAME}
)
```

with:

```cmake
install(PROGRAMS scripts/go_to_location.py scripts/odom_axle_to_base.py scripts/voice_nav.py
  scripts/object_nav.py scripts/objnav_geometry.py scripts/objnav_frontiers.py
  scripts/objnav_models.py scripts/objnav_value.py
  DESTINATION lib/${PROJECT_NAME}
)
```

`objnav_value.py` arrives in Task 9. Until then, create it as an empty file (`touch scripts/objnav_value.py`) so the build does not fail. Then make the node executable with `chmod +x scripts/object_nav.py`.

- [ ] **Step 3: Build and run all unit tests**

```bash
cd ~/wheelchair_nav && source /opt/ros/jazzy/setup.bash
colcon build --base-paths src --symlink-install --build-base build_src --install-base install_src \
  --packages-select wheelchair_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
cd src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test -q
```

Expected: the build succeeds and `28 passed`.

- [ ] **Step 4: Start-up check in the sim**

Run the Task 6 explore launch (terminal 1), `scripts/objnav_vlm_server.sh` (terminal 2), and in terminal 3:

```bash
cd ~/wheelchair_nav && source /opt/ros/jazzy/setup.bash && source install_src/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds_udp.xml HF_HUB_OFFLINE=1
unset NO_PROXY no_proxy
source .venv-voice/bin/activate
ros2 run wheelchair_description object_nav.py --ros-args -p use_sim_time:=true \
  -p detector:=$HOME/models/yoloe-26s-seg.pt
```

Expected: the log shows `Object search ready.` and no traceback. Then, from terminal 4:

```bash
ros2 topic pub --once /voice/transcript std_msgs/msg/String "{data: 'take me to the sofa'}"
```

Expected: `Looking for the sofa. Say stop to cancel.`, then `box ... at map (...)` lines, a `VLM check sofa ... yes`, `I found the sofa ...`, `Arrived at the sofa.` and a `SEARCH_RESULT target=sofa found=True` line.

- [ ] **Step 5: Commit**

```bash
git add src/wheelchair_description/scripts/object_nav.py src/wheelchair_description/scripts/objnav_value.py src/wheelchair_description/CMakeLists.txt
git commit -m "object_nav: zero-shot object search with frontiers, YOLOE and a Qwen3-VL check"
```

### Task 8: Simulation trials for approach A, and docs

**Files:**

- Modify: `~/wheelchair_nav/docs/architecture.md` (replace section 11 "Phase 2: camera and VLM (parked)" with the new design and the results), `~/wheelchair_nav/progress.md` (add a "Object search (phase 2)" run section)

**Interfaces:**

- Consumes: the Task 6 launch and the Task 7 node.
- Produces: an approach A baseline table that Task 10 compares against.

- [ ] **Step 1: Get ground truth for the objects**

```bash
cd ~/wheelchair_nav/src/wheelchair_description
for o in SofaC Refrigerator Bed_ TV_ KitchenTable; do echo "$o"; rg -A6 "residential_${o}" worlds/small_house.world | rg -m1 "<pose>"; done
```

The chair spawns at the world origin with yaw 0, and slam_toolbox starts the map there, so world x and y equal map x and y. Write the poses into the results table.

- [ ] **Step 2: Run the protocol**

Restart the sim for each row, so every search begins with an empty map. Send each command with `ros2 topic pub --once /voice/transcript ...`. In a separate terminal, watch for reversing: `ros2 topic echo /cmd_vel --field linear.x | awk '$1+0 < 0'`.

| Row | Command / action | Pass when |
|---|---|---|
| sofa | "take me to the sofa" | `found=True`; the logged object position is within 0.7 m of ground truth; the chair ends within 2.6 m of it |
| refrigerator | "take me to the refrigerator" | Same. This is the prototype's failure case |
| bed | "take me to the bed" | Same; it needs exploration through a doorway |
| television | "take me to the television" | Same |
| absent | "take me to the bathtub" | `found=False`, "I could not find the bathtub." within 330 s, no crash |
| stop during announce | "take me to the bed", then "stop" within 1 s | No goal is sent; `/cmd_vel` stays 0 |
| stop during spin | "take me to the bed", then "stop" during the look-behind spin | The spin is cancelled and `/cmd_vel` is 0 within 1 s |
| stop during search | "take me to the bed", then "stop" while driving to a frontier | `/cmd_vel` is 0 within 1 s; `SEARCH_RESULT found=False` |
| already moving | a second "take me to the tv" during a search | "I am already moving. Say stop first." |
| VLM down | stop `objnav_vlm_server.sh`, then "take me to the sofa" | "My camera check is not running, so I can't search."; no motion |
| false hit mid-trip | from the logs of all rows, find a `VLM check ... no` that interrupted a frontier trip | The same frontier is visited later in that search (or the log shows why not) |
| phase 1 regression | `progress.md` saved-map sim plus `voice_nav.py`, "take me to the kitchen" | Same as `progress.md`: "Arrived at the kitchen." |

Record each `SEARCH_RESULT` line, the pass or fail result, and any VLM `no` answers in the results table. If a row fails, do not tune parameters on your own. Report the row, the logs, and a proposed change.

- [ ] **Step 3: Update the docs**

In `docs/architecture.md`, replace section 11 with:

- "Phase 2: zero-shot object search", the design from this plan's "Design decisions", in normal prose.
- A mermaid flowchart of the per-command loop: announce, look, spin 180°, then the loop of confirmed → approach, otherwise next frontier → run_and_look, ending in "could not find".
- The results table from Step 2, dated.

Update the phase table at the top: phase 2 is "built; sim tested (approach A)". Add the sim commands from Task 7 Step 4 to `progress.md` under a new "Object search (phase 2)" heading.

- [ ] **Step 4: Commit**

```bash
git add docs/architecture.md progress.md
git commit -m "docs: object search design and approach A sim results"
```

---

## Stage 2: Approach B, semantic frontier scoring

### Task 9: Value map and CLIP scorer

**Files:**

- Create: `scripts/objnav_value.py` (replaces the empty file from Task 7)
- Modify: `scripts/objnav_models.py` (add `SceneScorer`)
- Test: `test/test_objnav_value.py`

**Interfaces:**

- Produces:
  - `ValueMap(size=80.0, res=0.10)` with `.update(x, y, yaw, hfov, score, max_range=4.0)` and `.at(x, y) -> float` (NaN where unseen)
  - `SceneScorer(model: str, pretrained: str, device: str)` with `.set_target(name)` and `.score(rgb) -> float`

- [ ] **Step 1: Write the failing tests**

`test/test_objnav_value.py`:

```python
import math

import pytest

from objnav_value import ValueMap

HFOV = math.radians(90)


def test_view_scores_cells_ahead_only():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.3)
    assert vm.at(2.0, 0.0) == pytest.approx(0.3)
    assert math.isnan(vm.at(-2.0, 0.0))  # behind the camera
    assert math.isnan(vm.at(4.5, 0.0))  # beyond max_range
    assert math.isnan(vm.at(2.0, 3.0))  # outside the field of view


def test_views_blend_on_axis():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.2)
    vm.update(0.0, 0.0, 0.0, HFOV, 0.4)
    assert vm.at(2.0, 0.0) == pytest.approx(0.3, abs=1e-3)


def test_yaw_turns_the_view():
    vm = ValueMap(size=10.0, res=0.1)
    vm.update(0.0, 0.0, math.pi / 2, HFOV, 0.5)
    assert vm.at(0.0, 2.0) == pytest.approx(0.5)
    assert math.isnan(vm.at(2.0, 0.0))


def test_outside_the_map_is_unseen():
    assert math.isnan(ValueMap(size=10.0, res=0.1).at(50.0, 0.0))
```

- [ ] **Step 2: Run the tests and confirm they fail**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_value.py -q`
Expected: `ImportError: cannot import name 'ValueMap'`.

- [ ] **Step 3: Implement the value map**

`scripts/objnav_value.py`:

```python
"""VLFM-style value map: how much each map cell looks like the target, built from CLIP
similarity scores of the camera views that covered it (Yokoyama et al., ICRA 2024).

Fixed square grid centred on the map origin. ponytail: an 80 x 80 m extent covers a hospital
floor wing; grow it with the SLAM map if a building is bigger. Occlusion is ignored (a view
scores cells behind walls too); add a depth-based FOV cut if frontier choice suffers.
"""
import math

import numpy as np


class ValueMap:
    def __init__(self, size=80.0, res=0.10):
        n = int(round(size / res))
        self.res, self.origin, self.n = res, -size / 2, n
        self.value = np.zeros((n, n), np.float32)
        self.conf = np.zeros((n, n), np.float32)
        centres = self.origin + (np.arange(n) + 0.5) * res
        self.xs, self.ys = np.meshgrid(centres, centres)  # xs[row, col] is the x of col

    def update(self, x, y, yaw, hfov, score, max_range=4.0):
        """Blend one view's score into the cells inside its field of view, trusting cells near
        the optical axis most (confidence cos^2 of the scaled off-axis angle, as in VLFM)."""
        dx, dy = self.xs - x, self.ys - y
        ang = np.arctan2(dy, dx) - yaw
        ang = np.arctan2(np.sin(ang), np.cos(ang))
        view = (np.hypot(dx, dy) < max_range) & (np.abs(ang) < hfov / 2)
        c = np.cos(ang[view] / (hfov / 2) * (math.pi / 2)) ** 2
        old_c, old_v = self.conf[view], self.value[view]
        self.value[view] = (old_v * old_c + score * c) / (old_c + c + 1e-9)
        self.conf[view] = (old_c ** 2 + c ** 2) / (old_c + c + 1e-9)

    def at(self, x, y):
        r = math.floor((y - self.origin) / self.res)
        c = math.floor((x - self.origin) / self.res)
        if not (0 <= r < self.n and 0 <= c < self.n) or self.conf[r, c] == 0:
            return float("nan")
        return float(self.value[r, c])
```

- [ ] **Step 4: Run the tests and confirm they pass**

Run: `cd ~/wheelchair_nav/src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test/test_objnav_value.py -q`
Expected: `4 passed`.

- [ ] **Step 5: Install open_clip and pick the MobileCLIP checkpoint**

```bash
source ~/wheelchair_nav/.venv-voice/bin/activate
pip install open_clip_torch "numpy<2"
python -c "import open_clip; print([p for p in open_clip.list_pretrained() if 'MobileCLIP' in p[0]])"
```

Use the `MobileCLIP-S2` entry if it is listed (pretrained tag `datacompdr`). Otherwise use the smallest `MobileCLIP2-*` entry. Record the exact model and pretrained names in the report, then use them in Step 6 and in Task 10.

- [ ] **Step 6: Add SceneScorer to objnav_models.py**

Append to `scripts/objnav_models.py`:

```python
class SceneScorer:
    """CLIP similarity between a camera view and "a photo of a <target>": the cue approach B
    uses to steer exploration toward areas that look like the target."""

    def __init__(self, model="MobileCLIP-S2", pretrained="datacompdr", device="cuda:0"):
        import open_clip
        import torch
        self.torch, self.device = torch, device
        self.model, _, self.preprocess = open_clip.create_model_and_transforms(
            model, pretrained=pretrained, device=device)
        self.model.eval()
        self.tokenizer = open_clip.get_tokenizer(model)
        self.text = None

    def set_target(self, name):
        with self.torch.no_grad():
            t = self.model.encode_text(self.tokenizer([f"a photo of a {name}"]).to(self.device))
        self.text = t / t.norm(dim=-1, keepdim=True)

    def score(self, rgb):
        from PIL import Image
        x = self.preprocess(Image.fromarray(rgb)).unsqueeze(0).to(self.device)
        with self.torch.no_grad():
            f = self.model.encode_image(x)
        f = f / f.norm(dim=-1, keepdim=True)
        return float((f @ self.text.T).item())
```

- [ ] **Step 7: Smoke check the scorer**

```bash
python3 - <<'EOF'
import cv2, sys
sys.path.insert(0, "scripts")
from objnav_models import SceneScorer
rgb = cv2.cvtColor(cv2.imread(__import__("os").environ["TMPDIR"] + "/bus.jpg"), cv2.COLOR_BGR2RGB)
s = SceneScorer()
for t in ("bus", "sofa"):
    s.set_target(t); print(t, round(s.score(rgb), 3))
EOF
```

Expected: the `bus` score is greater than the `sofa` score. If you picked different names in Step 5, pass them to `SceneScorer(...)`.

- [ ] **Step 8: Run all tests and commit**

```bash
PYTHONPATH=scripts /usr/bin/python3 -m pytest test -q
```

Expected: `32 passed`.

```bash
git add src/wheelchair_description/scripts/objnav_value.py src/wheelchair_description/scripts/objnav_models.py src/wheelchair_description/test/test_objnav_value.py
git commit -m "objnav: VLFM-style value map and MobileCLIP scene scorer"
```

### Task 10: Wire approach B into the node and compare it with A

**Files:**

- Modify: `scripts/object_nav.py` (`__init__`, `search`, `look`, `next_frontier`)
- Modify: `~/wheelchair_nav/docs/architecture.md`, `~/wheelchair_nav/progress.md`

**Interfaces:**

- Consumes: `ValueMap` and `SceneScorer` from Task 9, and `pick(..., value=)` from Task 3.
- Produces: the `semantic` parameter (bool, default `false`). B becomes the default only if Step 4 shows it is better.

- [ ] **Step 1: Make the node edits**

Add to the imports:

```python
from objnav_models import Detector, SceneScorer, Verifier
from objnav_value import ValueMap
```

Remove the old `from objnav_models import Detector, Verifier` line.

At the end of `ObjectNav.__init__`, add:

```python
        self.semantic = self.declare_parameter("semantic", False).value
        self.scorer = SceneScorer(self.declare_parameter("clip_model", "MobileCLIP-S2").value,
                                  self.declare_parameter("clip_pretrained", "datacompdr").value,
                                  self.get_parameter("device").value) if self.semantic else None
        self.values, self.scored = None, {}
```

In `search`, directly after `self.detector.set_target(target)`, add:

```python
            if self.semantic:
                self.scorer.set_target(target)
                self.values, self.scored = ValueMap(), {}
```

In `look`, move the TF lookup before the detector call, and add the scoring. The body of the `for cam in CAMERAS:` loop becomes:

```python
            f = dict(self.frames[cam])  # snapshot; callbacks keep replacing entries
            if len(f) < 3:
                continue
            rgb, depth = to_rgb(f["rgb"]), to_metres(f["depth"])
            if rgb.shape[:2] != depth.shape:
                self.get_logger().warn(f"{cam}: aligned depth {depth.shape} != colour "
                                       f"{rgb.shape[:2]}", throttle_duration_sec=10.0)
                continue
            try:
                tf = self.tf.lookup_transform("map", f["depth"].header.frame_id,
                                              Time.from_msg(f["depth"].header.stamp),
                                              timeout=Duration(seconds=0.1))
            except Exception as e:  # tf2 raises several unrelated exception types
                self.get_logger().warn(f"tf {cam}: {e}", throttle_duration_sec=5.0)
                continue
            q, p = tf.transform.rotation, tf.transform.translation
            r, t = quat_to_matrix(q.x, q.y, q.z, q.w), np.array([p.x, p.y, p.z])
            if self.semantic and time.monotonic() - self.scored.get(cam, 0.0) > 1.0:
                self.scored[cam] = time.monotonic()
                k = f["info"].k
                self.values.update(t[0], t[1], math.atan2(r[1, 2], r[0, 2]),  # optical z axis
                                   2 * math.atan(f["info"].width / (2 * k[0])), self.scorer.score(rgb))
            dets = self.detector.detect(rgb)
            for box, score, mask in dets:
                m = mask if mask is not None and mask.shape == depth.shape else box_mask(depth.shape, box)
                z = object_depth(depth, m)
                if z is None:
                    continue
                u, v = mask_centre(m)
                xy = (r @ pixel_to_optical(u, v, z, f["info"].k) + t)[:2]
                hits.add(Hit(xy, view, cam, (rgb, box)))
                self.get_logger().info(f"{cam}: box {[round(b) for b in box]} score {score:.2f} "
                                       f"at map ({xy[0]:.2f}, {xy[1]:.2f}), {z:.1f} m")
```

In `next_frontier`, replace the `return` line with:

```python
        value = self.values.at if self.semantic else None
        return pick(find_frontiers(to_grid(self.map), to_grid(self.costmap)), chair[:2], tried, value)
```

- [ ] **Step 2: Build and run the unit tests**

```bash
cd ~/wheelchair_nav && colcon build --base-paths src --symlink-install --build-base build_src \
  --install-base install_src --packages-select wheelchair_description --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
cd src/wheelchair_description && PYTHONPATH=scripts /usr/bin/python3 -m pytest test -q
```

Expected: `32 passed`.

- [ ] **Step 3: Sim check that B runs**

Start the node as in Task 7 Step 4, adding `-p semantic:=true` (and the clip names from Task 9 Step 5 if they differ). Send "take me to the bed".

Expected: the same success as approach A, and no `ValueMap` or `SceneScorer` traceback.

- [ ] **Step 4: Compare A and B**

Run the four object rows from Task 8 (sofa, refrigerator, bed, television) twice each with `semantic:=false` and twice with `semantic:=true`, restarting the sim before every run. Tabulate the `SEARCH_RESULT` time and path for each run.

The decision rule: make B the default (`declare_parameter("semantic", True)`) only if its mean time over the 8 runs is at least 15 % lower than A's, with no extra failures. Otherwise keep A as the default and record why.

- [ ] **Step 5: Docs and commit**

Add the A/B table and the decision to `docs/architecture.md` section 11 and to `progress.md`.

```bash
git add src/wheelchair_description/scripts/object_nav.py docs/architecture.md progress.md
git commit -m "object_nav: optional VLFM-style semantic frontier scoring, A/B sim results"
```

---

## Stage 3: Laptop on the real chair

### Task 11: Real chair with the laptop

**Files:**

- Modify: `~/wheelchair_nav/src/wc_control/launch/multi_camera.launch.py` (add a `side_color` argument)
- Modify: `~/wheelchair_nav/src/wheelchair_bringup/launch/wheelchair_fusion_nav.launch.py:107-116` (add a `localization` argument)
- Modify: `~/wheelchair_nav/docs/architecture.md`, `~/wheelchair_nav/progress.md`

**Interfaces:**

- Produces:
  - `multi_camera.launch.py side_color:=true` (default `false`, so mapping keeps today's USB load)
  - `wheelchair_fusion_nav.launch.py localization:=false` (default `true`)

- [ ] **Step 1: Add colour to the side cameras behind an argument**

In `multi_camera.launch.py`, next to the `side_resolution` `DeclareLaunchArgument`, add:

```python
        DeclareLaunchArgument(
            'side_color', default_value='false',
            description='Colour on the left and right cameras (object search needs it; '
                        'costs USB bandwidth)'),
```

Match the surrounding style: if the declarations are separate variables rather than list items, declare it as `declare_side_color = DeclareLaunchArgument(...)` and add it to the returned list. In both the `mapping_camera` and `right_camera` parameter dicts, replace:

```python
                    'enable_color': 'false',  # Depth only - saves USB bandwidth
```

with:

```python
                    'enable_color': LaunchConfiguration('side_color'),  # off by default: USB bandwidth
                    'rgb_camera.profile': LaunchConfiguration('side_resolution'),
```

- [ ] **Step 2: Let Nav2 run without the saved-map localization**

In `wheelchair_fusion_nav.launch.py`, add a declaration next to `declare_record_bag`:

```python
    declare_localization = DeclareLaunchArgument(
        'localization', default_value='true',
        description='Start map_server + AMCL on a saved map. false = Nav2 only, for a live '
                    'slam_toolbox map (object search in an unmapped building)'
    )
```

Add `condition=IfCondition(LaunchConfiguration('localization')),` to `localization_launch = IncludeLaunchDescription(...)`, and add `declare_localization,` to the returned list next to `declare_record_bag,`. Add `IfCondition` to the imports if it is missing.

- [ ] **Step 3: Find what the localization launch provides besides AMCL**

```bash
cd ~/wheelchair_nav/src/wheelchair_bringup/launch
rg -n "executable=|IncludeLaunchDescription" wheelchair_fusion_localization.launch.py
rg -n "executable=|IncludeLaunchDescription" wheelchair_slam_mapping.launch.py
```

List every node the localization launch starts that the SLAM launch does not, apart from `map_server`, `amcl` and their lifecycle manager. `odom_velocity_corrector` (around line 433) is known to be one. For each one Nav2 needs, which at least includes whatever publishes the odometry topic that `velocity_smoother` and `controller_server` read in `nav2_params_3cam_v30.yaml` (`rg -n "odom_topic" ../../wheelchair_navigation/config/nav2_params_3cam_v30.yaml`), write down the exact `ros2 run` command with its parameters copied from the localization launch. Put these commands in `progress.md` under "Object search on the chair". Do not edit the localization or SLAM launch files.

- [ ] **Step 4: Bring-up check on the chair (empty chair, wheels off the ground or in open space, hand on the e-stop)**

```bash
run_slam   # terminal 1; this pkills all ROS 2 processes first, so start it first
ros2 launch wc_control multi_camera.launch.py side_color:=true   # only if run_slam does not start the cameras; otherwise pass side_color through the launch that does
ros2 launch wheelchair_bringup wheelchair_fusion_nav.launch.py localization:=false use_rviz:=false \
  record_bag:=false use_imu_diagnostic:=false use_collision_monitor:=true \
  nav2_params:=$HOME/wheelchair_nav/src/wheelchair_navigation/config/nav2_params_3cam_v30.yaml \
  bt_xml:=$HOME/wheelchair_nav/src/wheelchair_navigation/behavior_tree/wheelchair_robust_nav_v5.xml
# plus the ros2 run commands from Step 3
```

Checks:

- `ros2 node list | grep -E "/amcl|/map_server"` prints nothing.
- All Nav2 nodes print `active [3]`.
- `ros2 topic hz /camera/color/image_raw /mapping_camera/color/image_raw /right_camera/color/image_raw` shows 10 Hz or more on each.
- `ros2 topic hz /scan_fused` keeps its usual rate.
- `ros2 topic echo --once /mapping_camera/aligned_depth_to_color/image_raw --field encoding` prints `16UC1`.

If colour on the side cameras drops any stream below 10 Hz, set `side_resolution:=424x240x6` and re-check.

- [ ] **Step 5: Model check on real frames**

Save one frame from each camera with `ros2 run scripts rgb_depth_saver` (or `ros2 topic echo --once` plus cv2), photographing a chair, a table and a door. Then run `objnav_smoke.py` on each with those three targets.

Expected: on each image, at least one target gets a box with VLM `yes`, and one absent target ("refrigerator", if there is none) gets no `yes`.

- [ ] **Step 6: Empty-chair trials**

Use a room plus a corridor, with the collision monitor on. Run `object_nav.py` (no `use_sim_time`). Ten runs, mixing the targets visible from the start (2), behind the chair (2), in the next room (4) and absent (2).

Pass: 9 of 10 runs end correctly, there is zero contact with anything, and "stop" brings `/cmd_vel` to 0 within 1 s in 3 of 3 tries. Log every `SEARCH_RESULT` line.

- [ ] **Step 7: Seated trials (only after Step 6 passes; a second person on the e-stop)**

Five runs with a passenger, targets in the next room. Record the results, plus anything the passenger reports (speed, the announcement wording, waiting during VLM checks).

- [ ] **Step 8: Docs and commit**

Add the chair procedure and results to `progress.md` and `docs/architecture.md`.

```bash
git add src/wc_control/launch/multi_camera.launch.py src/wheelchair_bringup/launch/wheelchair_fusion_nav.launch.py docs/architecture.md progress.md
git commit -m "object search on the chair: side-camera colour and Nav2 without AMCL, trial results"
```

---

## Stage 4: Jetson Orin Nano Super

### Task 12: Port to the Jetson and check the budgets

**Files:**

- Modify: `~/wheelchair_nav/docs/architecture.md` (section 12), `~/wheelchair_nav/progress.md`

**Interfaces:**

- Consumes: everything above. No code changes are expected. Model paths and `device` are already ROS parameters, and the server paths are environment variables (`LLAMA`, `MODELS`).

- [ ] **Step 1: Choose native or container**

Run `cat /etc/nv_tegra_release; lsb_release -a` on the Jetson. If it is Ubuntu 24.04, install ROS 2 Jazzy natively. Otherwise run ROS in a Jazzy container from jetson-containers (`jetson-containers list | grep -i jazzy`, then `jetson-containers run $(autotag <that image>)`). Record the choice.

- [ ] **Step 2: Set max power and measure the baseline memory**

```bash
sudo nvpmodel -m 0 && sudo jetson_clocks   # MAXN SUPER
```

Start the chair stack from Task 11 Step 4 without `object_nav.py`. Let `tegrastats --interval 1000` run for 60 s and record the used and free RAM. This is the baseline.

- [ ] **Step 3: Install the models**

PyTorch comes from the Jetson AI Lab index for the installed JetPack (for example, `pip install torch torchvision --index-url https://pypi.jetson-ai-lab.io/jp6/cu126`; check the index path for your JetPack), then `pip install ultralytics "numpy<2"`. Build llama.cpp as in Task 1 Step 2, with `-DCMAKE_CUDA_ARCHITECTURES=87`. Copy `~/models/` from the laptop.

- [ ] **Step 4: Smoke gates on the Jetson**

```bash
LLAMA=$HOME/llama.cpp/build/bin/llama-server MODELS=$HOME/models/qwen3-vl-2b scripts/objnav_vlm_server.sh &
python3 scripts/objnav_smoke.py <chair frame from Task 11> chair --weights ~/models/yoloe-26s-seg.pt
```

Gates, with the chair stack running:

- The detector is at or under 150 ms/frame, so all three cameras are looked at about twice per second.
- A VLM check is at or under 3000 ms.
- `tegrastats` free RAM stays at or above 1 GB.

If a gate fails, try these fallbacks in order, one at a time, re-measuring after each and stopping at the first that passes. Ask the user before keeping any of them:

1. `yoloe-26n-seg.pt`.
2. `-c 2048 --image-max-tokens 256` on the server.
3. Load the VLM only during a search (start the server on the first command and stop it afterwards). This costs a ~20 s first call.
4. NanoOWL in place of YOLOE.

- [ ] **Step 5: Re-run Task 11 Step 6 with the Jetson in place of the laptop**

Expected: the same pass bar (9 of 10, no contact, stop within 1 s).

- [ ] **Step 6: Docs and commit**

Update `docs/architecture.md` section 12 with the measured memory, the latencies and the chosen fallbacks, and add the Jetson run steps to `progress.md`.

```bash
git add docs/architecture.md progress.md
git commit -m "docs: object search on the Jetson Orin Nano Super, measured budgets"
```

---

## Risks

- **Laptop RAM.** Gazebo and Nav2 already use about 11 of 14 GB. Torch, YOLOE and open_clip add about 2–3 GB, and the laptop will swap. Run Gazebo without RViz. If the sim stutters, run `object_nav.py` with `semantic:=false` and report it.
- **YOLOE recall on uncommon words.** Zero-shot detection is weaker on rare names ("commode", "IV stand"). A miss shows up as `found=False` after the full search, never as a wrong goal. Record misses in the Task 11 trials.
- **SLAM drift during a long search.** Object positions are stored in the map frame, so a loop closure can shift them. The approach goal is computed right after the VLM confirms, from hits at most a few seconds old.
- **Hospital Wi-Fi is irrelevant** (everything is on the device), but the first YOLOE `set_classes` call needs the MobileCLIP text encoder cached. Task 1 Step 7 caches it, and runtime uses `HF_HUB_OFFLINE=1`.
- **The collision monitor is off by default** in the sim. The chair trials in Task 11 turn it on (`use_collision_monitor:=true`).
