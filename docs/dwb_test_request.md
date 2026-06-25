# Nav2 Score-Based Navigation Test Request

Use this checklist to compare the baseline Nav2 parameters with
`config/nav2_params_dwb_safe.yaml`.

## Launch

Docker simulation example:

```bash
ros2 launch storagy sim.launch.py use_rviz:=false
```

In another noVNC terminal:

```bash
ros2 launch storagy navigation.launch.py \
  params_file:=/opt/storagy-practice-ws-docker/src/storagy/param/nav2_params_dwb_safe.yaml
```

Native/robot example:

```bash
ros2 launch storagy navigation.launch.py \
  map:=/home/storagy/maps/1206_new_map.yaml \
  params_file:=/home/storagy/Desktop/storagy_ws/src/storagy/param/nav2_params_dwb_safe.yaml
```

`navigation.launch.py` in the public Storagy repo already supports
`params_file`, so the original `nav2_params.yaml` does not need to be
overwritten for A/B testing.

## RViz Topics To Watch

- `/global_costmap/costmap`
- `/local_costmap/costmap`
- `/plan`
- `/local_plan`
- `/cmd_vel`
- `/tf`

## Test Cases

### 1. Straight Path Baseline

Goal: verify the robot can still reach a simple goal.

Pass criteria:

- The robot reaches the goal.
- No repeated recovery behavior appears.
- `/cmd_vel` does not show sharp alternating angular commands.

### 2. Narrow Corridor

Goal: check whether the safety score is too conservative.

Pass criteria:

- The robot can enter the corridor if it is physically valid.
- The footprint does not touch inflated obstacle zones in RViz.
- If planning fails, reduce `inflation_radius` before reducing obstacle critic
  weights.

### 3. Obstacle Clearance

Goal: confirm that the new profile prefers safer clearance.

Pass criteria:

- Compared with the baseline, the global path runs farther from obstacle
  edges when space is available.
- Local trajectories do not skim walls or table legs.
- The robot does not take a much longer path unless the clearance improvement
  is visible.

### 4. Dynamic Obstacle

Goal: validate live obstacle updates from `/scan`.

Pass criteria:

- A new obstacle appears in the local costmap quickly.
- The robot slows, replans, or avoids the obstacle.
- After the obstacle is removed, the costmap clears within a few seconds.

### 5. Blocked Path Recovery

Goal: decide whether a Behavior Tree upgrade is needed.

Pass criteria:

- If the path is blocked briefly, the robot waits or replans cleanly.
- If the path stays blocked, record how long it takes before recovery starts.
- If behavior is too passive, add a BT condition for blocked-path replanning.

## Measurements

Record these for both baseline and tuned runs:

- Time to goal
- Minimum obstacle clearance
- Number of recoveries
- Number of replans
- Maximum angular velocity
- Visible oscillation count
- Estimated energy cost
- Accumulated velocity change
- Accumulated acceleration change
- Stop/restart count

## Result Template

```text
Map/scenario:
Baseline params:
Tuned params:
Goal pose:
Time to goal:
Minimum clearance:
Recoveries:
Replans:
Energy score:
Velocity change:
Acceleration change:
Stops/restarts:
Observed issue:
Next tuning change:
```
