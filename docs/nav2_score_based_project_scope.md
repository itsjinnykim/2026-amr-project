# Nav2 Score-Based Improvement Scope

This document maps the desired project items to concrete files and runtime
checks in this repo.

## What Is Implemented Now

| Item | Implementation |
| --- | --- |
| AMCL | Preserved in `config/nav2_params_dwb_safe.yaml` from the Storagy params. |
| map_server | Preserved in `config/nav2_params_dwb_safe.yaml`; map is still passed by launch argument. |
| Nav2 basic execution | Use `storagy navigation.launch.py params_file:=...` as described in the runbook. |
| Obstacle cost adjustment | `global_costmap` and `local_costmap` inflation settings. |
| Inflation tuning | `inflation_radius` and `cost_scaling_factor` tuning recipes. |
| Costmap before/after comparison | `nav2_score_monitor.py` records inflated/high-cost cell counts and mean cost. |
| Path length score | `scripts/nav2_score_monitor.py` computes path length from `/plan`. |
| Obstacle distance score | `scripts/nav2_score_monitor.py` estimates nearest obstacle from `/local_costmap/costmap`. |
| Rotation score | `scripts/nav2_score_monitor.py` integrates absolute `/cmd_vel.angular.z`. |
| Energy efficiency estimate | `scripts/nav2_score_monitor.py` estimates energy cost from velocity changes, acceleration changes, stops, and restarts. |
| DWB critic weights | `FollowPath` critic weights in `config/nav2_params_dwb_safe.yaml`. |
| Smooth driving | Lower velocity/accel limits plus velocity smoother settings reduce sharp starts, stops, and turns. |
| Blocked path detection | Progress checker plus score monitor blocked-state detection. |
| Replan | Nav2 BT periodic replanning; custom BT in `behavior_trees/score_replanning_recovery.xml`. |
| Recovery improvement | Custom BT clear-costmap, wait, backup, spin recovery order. |

## How The Score Is Calculated

The monitor uses:

```text
score =
  distance_weight * path_length_m
  + time_weight * elapsed_time_s
  + rotation_weight * accumulated_abs_rotation_rad
  + safety_weight * max(0, safety_distance_m - nearest_obstacle_m)
  + energy_weight * estimated_energy_cost
  + blocked_weight if blocked
```

Lower score is better.

Energy cost is an estimate, not a direct battery reading:

```text
estimated_energy_cost =
  velocity_change_weight * accumulated_velocity_change
  + acceleration_change_weight * accumulated_acceleration_change
  + stop_weight * stop_count
  + restart_weight * restart_count
```

Default meaning:

- Shorter global path is better.
- Shorter running time is better.
- Less accumulated turning is better.
- Larger obstacle clearance is better.
- Fewer speed changes, acceleration changes, stops, and restarts are better
  for estimated battery efficiency.
- Blocked-path behavior is penalized.

## Baseline vs Tuned Test

Terminal 1:

```bash
ros2 launch storagy sim.launch.py use_rviz:=false
```

Terminal 2 baseline:

```bash
ros2 launch storagy navigation.launch.py
```

Terminal 3 baseline score monitor:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_baseline.csv
```

Repeat the same goal in RViz with `Nav2 Goal`.

Terminal 2 tuned:

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
```

Terminal 3 tuned score monitor:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_tuned.csv
```

Compare:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/compare_nav2_score_runs.py \
  --baseline /tmp/nav2_baseline.csv \
  --tuned /tmp/nav2_tuned.csv
```

The comparison reports score, path length, minimum obstacle distance, total
rotation, estimated energy cost, velocity/acceleration change, stop/restart
counts, blocked samples, inflated costmap cells, high-cost cells, and mean
costmap cost.

## How To Enable The Custom BT

Copy:

```bash
mkdir -p /opt/storagy-practice-ws-docker/src/storagy/behavior_trees
cp behavior_trees/score_replanning_recovery.xml \
  /opt/storagy-practice-ws-docker/src/storagy/behavior_trees/
```

Then add this parameter under `bt_navigator.ros__parameters` in the copied
Nav2 params file:

```yaml
default_nav_to_pose_bt_xml: /opt/storagy-practice-ws-docker/src/storagy/behavior_trees/score_replanning_recovery.xml
```

For the native robot workspace, use the absolute path under
`/home/storagy/Desktop/storagy_ws/src/storagy/behavior_trees/...`.

## What Still Counts As Future Work

These items are not necessary for the current project scope:

- A compiled custom DWB critic plugin.
- A custom global planner plugin with its own C++ score function.
- Multi-robot traffic management.
- D* Lite, Hybrid A*, or RRT planner integration.

The current implementation is enough to demonstrate score-based navigation
improvement with measurable before/after results.
