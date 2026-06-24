# Nav2 DWB Score-Based Tuning Note

This repo uses a no-custom-plugin first step for score-based navigation.
The idea is to make Nav2's existing scoring surfaces do more work before
writing a new planner:

- Global score: the planner reads inflated costmap values and selects a path
  with lower accumulated cost.
- Local score: DWB samples velocity commands and scores each trajectory with
  weighted critics.
- Smoothness score: the velocity smoother limits acceleration and sudden
  angular changes.

## Implemented Profile

File: `config/nav2_params_dwb_safe.yaml`

This file is a full Storagy replacement profile derived from:

- `bluephysi01/storagy-practice-ws-docker/src/storagy/param/nav2_params.yaml`
- `bluephysi01/storagy-practice-ws-docker/src/storagy/launch/navigation.launch.py`

`navigation.launch.py` already exposes a `params_file` argument, so this profile
can be tested without overwriting the original params.

The profile is safety-first:

- Costmap inflation radius is widened to keep the robot away from obstacle
  edges.
- The original Storagy square footprint is preserved instead of replacing it
  with a guessed `robot_radius`.
- `cost_scaling_factor` is set to `2.0`; this makes the inflated cost decay
  more gently, so clearance is visible farther from obstacles.
- The global planner is changed from NavFn to `SmacPlanner2D` with
  `cost_travel_multiplier: 2.0`; this makes the global path avoid high-cost
  cells more strongly.
- DWB `BaseObstacle` and `ObstacleFootprint` critics are weighted higher than
  the common TurtleBot3 baseline.
- Forward and low-turn behavior is encouraged through `PreferForward`,
  `Twirling`, reduced angular velocity, and gentler acceleration limits.
- The local costmap frame is changed to `odom`, which is the usual Nav2 choice
  for a rolling local costmap.

## Score Mapping

The PDF's score formula can be mapped onto existing Nav2 parameters like this:

```text
Score = a * Distance + b * Time + c * Energy + d * Safety
```

- Distance: `PathDist`, `GoalDist`, `PathAlign`, `GoalAlign`
- Time: `max_vel_x`, `sim_time`, controller frequency, planner frequency
- Energy: `acc_lim_x`, `acc_lim_theta`, `velocity_smoother`, `Twirling`
- Safety: costmap inflation, `BaseObstacle`, `ObstacleFootprint`,
  `cost_travel_multiplier`

This is not a literal custom score function yet. It is a practical first
implementation using Nav2's built-in scoring hooks.

## Tuning Recipes

When the robot drives too close to obstacles:

- Increase `inflation_radius` by `0.05`.
- Decrease `cost_scaling_factor` by `0.2` to spread the cost farther.
- Increase `BaseObstacle.scale` by `0.01`.

When the robot refuses narrow but valid corridors:

- First verify the `footprint` and `footprint_padding`.
- Decrease `inflation_radius` by `0.05`.
- Increase `cost_scaling_factor` by `0.2`.
- Decrease `BaseObstacle.scale` by `0.01`.

When the robot oscillates or spins too much:

- Increase `RotateToGoal.scale` by `4.0`.
- Increase `Twirling.scale` by `2.0`.
- Decrease `max_vel_theta` by `0.05`.
- Decrease `acc_lim_theta` by `0.2`.

When the robot is too slow:

- Increase `max_vel_x` by `0.02`.
- Increase `acc_lim_x` by `0.1`.
- Keep obstacle tests active after each speed change.

When the planner plugin fails to load:

- Replace `nav2_smac_planner::SmacPlanner2D` with the original
  `nav2_navfn_planner/NavfnPlanner`.
- Remove `cost_travel_multiplier` because NavFn does not use it.
- Keep the DWB and costmap scoring changes.

## Next Step: Real Custom Score Plugin

After the parameter-only profile is stable, the next meaningful upgrade is a
custom Nav2 plugin:

- Planner plugin: combine path length, costmap risk, estimated time, and turn
  penalty before returning a global path.
- Controller critic: add a custom DWB critic that penalizes repeated sharp
  turns or passing through crowded regions.
- Behavior Tree condition: switch profiles when the path is blocked or when
  an emergency/energy-saving mode is active.

Recommended order:

1. Validate this parameter profile in RViz.
2. Add a Behavior Tree branch for blocked-path replanning.
3. Add one custom DWB critic for energy or crowd-risk scoring.
4. Add multi-robot traffic scoring only after single-robot behavior is stable.
