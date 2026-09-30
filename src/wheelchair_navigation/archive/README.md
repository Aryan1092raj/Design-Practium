# Archived configs

Superseded `nav2_params_*.yaml` versions, kept for rollback only. Nothing here
is installed or referenced by a launch file.

## Active configs (in `../config/`)

| File | Used by |
|---|---|
| `nav2_params_3cam_v29.yaml` | `wheelchair_fusion_nav.launch.py` (default), `wheelchair_cable_trace.launch.py` |
| `nav2_params_ablation_lidar_only_v2.yaml` | `wheelchair_ablation_lidar_only.launch.py` |
| `nav2_params_ablation_stvl_v2.yaml` | `wheelchair_ablation_stvl.launch.py` |
| `nav2_params_kinoflow.yaml` | `wheelchair_kinoflow_nav.launch.py` |
| `nav2_params_kinoflow_cpp_v2.yaml` | `wheelchair_kinoflow_cpp_nav.launch.py` |

## Rollback

To go back to an archived version, move it into `../config/` and repoint the
`default_nav2_params` line in the relevant launch file (or pass
`nav2_params:=<file>` at launch). Per project convention, never edit a config
in place: copy forward to a new version instead.