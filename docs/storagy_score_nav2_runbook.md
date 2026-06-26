# Storagy Nav2 Score-Based Improvement Runbook

This runbook adapts the completed Storagy SLAM and navigation workflow to the
score-based Nav2 profile in this repo.

## Goal

Improve navigation without writing a custom plugin first:

- Give obstacles a wider safety score through costmap inflation.
- Make the global planner prefer lower-risk cells.
- Make the DWB local planner prefer safer, smoother commands.
- Estimate battery efficiency from distance, rotation, velocity changes,
  acceleration changes, stops, and restarts.
- Keep the original Docker practice flow:
  `ros2 launch storagy navigation.launch.py`.

The public Docker practice repo uses:

- Launch file: `src/storagy/launch/navigation.launch.py`
- Default params: `src/storagy/param/nav2_params.yaml`
- Params override argument: `params_file`

## 1. Connect To The Robot

From Windows PowerShell:

```bash
wsl -d Ubuntu-22.04
```

From WSL or native Ubuntu:

```bash
ssh storagy@<ROBOT_IP> -XC
```

Password:

```text
123412
```

Then, for the native robot workspace:

```bash
cd ~/Desktop/storagy_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. Docker Practice Repo Path

If you are using the public Docker practice repo:

```bash
git clone https://github.com/bluephysi01/storagy-practice-ws-docker.git
cd storagy-practice-ws-docker
```

Copy this repo's tuned profile into Storagy's parameter folder:

```bash
cp /mnt/c/Users/jin/Desktop/2026-amr-project/config/nav2_params_dwb_safe.yaml \
  src/storagy/param/nav2_params_dwb_safe.yaml
```

Start the container:

```bash
docker compose up -d
```

Open `http://localhost:6080`, then in noVNC terminal 1:

```bash
ros2 launch storagy sim.launch.py use_rviz:=false
```

In noVNC terminal 2:

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
```

If you changed files after the container had already built the package, either
use the absolute source path above or run:

```bash
docker compose exec storagy-practice rebuild_ws.sh
```

## 3. Find The Current Nav2 Parameter File

Run this in the Storagy workspace:

```bash
grep -RIn \
  "planner_server\|controller_server\|local_costmap\|global_costmap\|params_file" \
  src install \
  --include "*.yaml" \
  --include "*.yml" \
  --include "*.py"
```

Known Docker practice target:

- `src/storagy/param/nav2_params.yaml`

Other possible native targets:

- `src/storagy/launch/bringup.launch.py`
- `src/storagy/launch/navigation2/navigation2.launch.py`
- `src/storagy/param/navigation2/storagy.yaml`
- `install/storagy/share/storagy/config/*.yaml`
- `install/storagy/share/storagy/param/*.yaml`

Prefer editing the source-space file under `src/`, then rebuild if the package
installs config files into `install/`.

## 4. Copy This Repo's Score Profile To The Native Robot

From WSL, before SSH-ing into the robot:

```bash
scp /mnt/c/Users/jin/Desktop/2026-amr-project/config/nav2_params_dwb_safe.yaml \
  storagy@<ROBOT_IP>:/home/storagy/Desktop/storagy_ws/src/storagy/param/navigation2/
```

If working directly on the native Ubuntu machine, copy the file with a USB
drive or GitHub clone.

## 5. Back Up The Original Params

On the robot:

```bash
cd ~/Desktop/storagy_ws
cp src/storagy/param/navigation2/storagy.yaml \
  src/storagy/param/navigation2/storagy.yaml.bak.$(date +%Y%m%d_%H%M%S)
```

Do not skip this. The tuned profile is intentionally more conservative and may
need adjustment for the actual robot footprint and map.

## 6. Apply Safely

Use one of these paths.

Path A: launch file supports `params_file`.

```bash
ros2 launch storagy bringup.launch.py \
  map:=/home/storagy/maps/1206_new_map.yaml \
  params_file:=/home/storagy/Desktop/storagy_ws/src/storagy/param/navigation2/nav2_params_dwb_safe.yaml
```

Path B: launch file does not support `params_file`.

Merge these sections from `nav2_params_dwb_safe.yaml` into the original Nav2
parameter file:

- `planner_server`
- `controller_server`
- `local_costmap`
- `global_costmap`
- `velocity_smoother`
- `bt_navigator`

Keep original robot-specific values if they exist:

- `robot_radius` or `footprint`
- `robot_base_frame`
- sensor topic names such as `/scan`
- map file paths
- lifecycle or namespace settings

Then rebuild if needed:

```bash
colcon build --symlink-install
source install/setup.bash
```

## 7. Verify Loaded Parameters

After bringup:

```bash
ros2 param get /controller_server FollowPath.BaseObstacle.scale
ros2 param get /controller_server FollowPath.PathDist.scale
ros2 param get /planner_server GridBased.plugin
ros2 param get /global_costmap/global_costmap inflation_layer.inflation_radius
ros2 param get /local_costmap/local_costmap inflation_layer.inflation_radius
```

If any command says the parameter is not set, the tuned file was not loaded or
the node name differs.

## 8. Test Order

Use RViz in this order:

1. Set `2D Pose Estimate`.
2. Send a short `Nav2 Goal` in open space.
3. Send a goal near a wall and compare clearance.
4. Add a temporary obstacle and check local replanning.
5. Try a narrow corridor only after open-space behavior is stable.

Watch:

- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan`
- `/local_plan`
- `/cmd_vel`
- `/nav2_score_report`

## 9. First Tuning Moves

Too close to obstacles:

```text
inflation_radius +0.05
cost_scaling_factor -0.2
BaseObstacle.scale +0.01
```

Cannot pass valid narrow areas:

```text
Check robot_radius or footprint first.
inflation_radius -0.05
cost_scaling_factor +0.2
BaseObstacle.scale -0.01
```

Too much spinning:

```text
RotateToGoal.scale +4.0
Twirling.scale +2.0
max_vel_theta -0.05
```

Estimated energy cost too high:

```text
max_vel_x -0.02
acc_lim_x -0.1
decel_lim_x magnitude -0.1
Twirling.scale +1.0 if repeated turning is visible
```

Too slow:

```text
max_vel_x +0.02
acc_lim_x +0.1
```

## 10. When To Add Behavior Tree Logic

Add a Behavior Tree change after parameter tuning if:

- The robot waits too long behind a blocked path.
- A dynamic obstacle repeatedly triggers recovery instead of replanning.
- You need modes such as safe mode, fast mode, or energy-saving mode.

Until those cases are observed, costmap plus DWB scoring is the fastest useful
improvement path.

## 11. Score Monitor And Before/After Comparison

Copy these files into the Storagy workspace or run them directly from this repo:

- `scripts/nav2_score_monitor.py`
- `scripts/compare_nav2_score_runs.py`
- `behavior_trees/score_replanning_recovery.xml`

For the Docker practice repo, copy them under the mounted `src` tree:

```bash
cd /path/to/storagy-practice-ws-docker
/mnt/c/Users/jin/Desktop/2026-amr-project/scripts/install_score_tools_to_storagy.sh "$PWD"
```

Manual copy version:

```bash
mkdir -p src/storagy/scripts src/storagy/behavior_trees src/storagy/param
cp /mnt/c/Users/jin/Desktop/2026-amr-project/config/nav2_params_dwb_safe.yaml \
  src/storagy/param/
cp /mnt/c/Users/jin/Desktop/2026-amr-project/scripts/nav2_score_monitor.py \
  src/storagy/scripts/
cp /mnt/c/Users/jin/Desktop/2026-amr-project/scripts/compare_nav2_score_runs.py \
  src/storagy/scripts/
cp /mnt/c/Users/jin/Desktop/2026-amr-project/behavior_trees/score_replanning_recovery.xml \
  src/storagy/behavior_trees/
```

Baseline run:

```bash
ros2 launch storagy navigation.launch.py
python3 /mnt/c/Users/jin/Desktop/2026-amr-project/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_baseline.csv
```

Inside the Docker noVNC terminal, use:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_baseline.csv
```

Tuned run:

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
python3 /mnt/c/Users/jin/Desktop/2026-amr-project/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_tuned.csv
```

Inside the Docker noVNC terminal, use:

```bash
python3 /opt/storagy-practice-ws-docker/src/storagy/scripts/nav2_score_monitor.py \
  --ros-args -p csv_path:=/tmp/nav2_tuned.csv
```

Use the same RViz start pose and `Nav2 Goal` for both runs. Then compare:

```bash
python3 /mnt/c/Users/jin/Desktop/2026-amr-project/scripts/compare_nav2_score_runs.py \
  --baseline /tmp/nav2_baseline.csv \
  --tuned /tmp/nav2_tuned.csv
```

The comparison reports score, path length, minimum obstacle distance, total
rotation, estimated energy cost, velocity/acceleration change, stop/restart
counts, blocked samples, inflated costmap cells, high-cost cells, and mean
costmap cost.
