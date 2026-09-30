# Archived configs

Superseded SLAM Toolbox and laser-filter configs (`slam_toolbox_fused_v2..v20`,
the retired `slam_toolbox_v14*` and `_v2` series, and older hospital variants).
Kept for rollback only — these are **not** listed in `setup.py` and are not
installed.

## Active configs (in `../config/`)

| File | Used by |
|---|---|
| `slam_toolbox_motion_compensated_v2.yaml` | `wheelchair_slam_mapping.launch.py` — lidar-only default |
| `slam_toolbox_fused_v21.yaml` | `wheelchair_slam_mapping.launch.py` — fused default (`use_fused_slam:=true`) |
| `slam_toolbox_hospital_lidar_v4.yaml` | `hospital_mode:=true`, lidar-only |
| `slam_toolbox_hospital_fused_v2.yaml` | `hospital_mode:=true`, fused |
| `laser_filter_robust.yaml` | standard mapping/localization filter chain |
| `laser_filter_hospital_v2.yaml` | hospital mode (30 m range) |
| `laser_filter.yaml`, `laser_filter_hospital.yaml` | simple / rtabmap filter chains |
| `amcl_fusion.yaml`, `ekf.yaml`, `ekf_global.yaml` | localization and odometry fusion |
| `slam_toolbox_fused_robust.yaml` | fused fallback |

## Rollback

Move the file back into `../config/`, re-add it to the `data_files` list in
`../../setup.py`, rebuild, and repoint the launch file (or pass
`slam_config:=<file>` at launch).