# Drone Interceptor

This package is a ROS 2 Humble simulation of a target drone and an interceptor drone.
It supports both a baseline guidance controller and an MPC-based controller so
you can compare how each interceptor behaves against the same target. 
Note that there may be bugs and that the MPC is currently quite bad and untuned.

## How To Use The Package

1. Source ROS 2 Humble.
2. Build the package from the workspace root.
3. Source the workspace overlay.
4. Launch the simulation with the controller and scenario you want.
5. Watch the motion in RViz, inspect distances with `rqt_plot`, and review saved
   result artifacts in `results/drone_interceptor`.

Typical workflow:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_interceptor --symlink-install
source install/setup.bash
ros2 launch drone_interceptor target_sim.launch.py
```

What the package does during a run:

- The launch file resolves a scenario, controller preset, profile, seed, and
  output settings from the YAML configs and any command-line overrides.
- The target behavior node generates desired heading and speed commands.
- The target dynamics node converts those commands into the target state plus
  RViz markers and TF updates.
- The estimator publishes a noisy target measurement and a filtered target
  estimate for downstream control.
- The launch selects either the baseline guidance+dynamics branch or the
  MPC+dynamics branch for the interceptor.
- A shared distance monitor publishes target-to-interceptor separation on
  `/intercept/distance`.
- The results logger records that distance stream and writes CSV, JSON, and PNG
  artifacts when the run ends.

## Build Commands

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_interceptor --symlink-install
source install/setup.bash
```

Run tests:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select drone_interceptor
colcon test-result --verbose
```

## Launch Commands

### Default Launch

```bash
ros2 launch drone_interceptor target_sim.launch.py
```

Default behavior:

- scenario: `target_advantaged_default`
- controller preset: `baseline_lead_pursuit`
- target/interceptor start 30 meters apart
- target threat response is enabled
- RViz opens automatically
- `rqt_plot` opens automatically

### Launch Options


```bash
ros2 launch drone_interceptor target_sim.launch.py <argument>:=<value> <argument>:=<value>
```

The following launch arguments can be passed:

- `experiment:=baseline_pure_pursuit_advantaged | baseline_lead_pursuit_advantaged | baseline_acceleration_aware_advantaged | mpc_advantaged`
- `scenario:=matched_default | target_faster_default | target_more_maneuverable_default | target_advantaged_default | target_advantaged_close_start | target_advantaged_no_threat | target_advantaged_medium_noise`
- `controller_preset:=baseline_pure_pursuit | baseline_lead_pursuit | baseline_acceleration_aware | mpc_default`
- `profile:=matched | target_faster | target_more_maneuverable | target_advantaged`
- `controller_mode:=baseline | mpc`
- `guidance_mode:=pure_pursuit | lead_pursuit | acceleration_aware_lead_pursuit`
- `threat_response:=true | false`
- `spawn_distance:=[float]`
- `random_seed:=[int]`
- `open_rviz:=true | false`
- `open_plot:=true | false`
- `output_dir:=[path/string]`


## Performance

The figure below shows the averaged overlay comparison across five 60-second
runs for each guidance/controller method. The comparison used the
`target_advantaged_default` scenario with `profile:=target_advantaged`,
`spawn_distance:=30.0`, `threat_response:=true`, and five runs per method. The
evaluated presets were `baseline_pure_pursuit`, `baseline_lead_pursuit`,
`baseline_acceleration_aware`, and `mpc_default`, using seeds `0` through `19`
across the full batch.

Command used to generate the comparison data and averaged overlay:

```bash
run_experiment \
  --results-dir results/drone_interceptor_average_runs \
  --duration-s 60 \
  --runs-per-controller 5 \
  --controllers baseline_pure_pursuit baseline_lead_pursuit baseline_acceleration_aware mpc \
  --profile target_advantaged \
  --scenario target_advantaged_default \
  --spawn-distance 30.0 \
  --threat-response true \
  --seed-start 0

plot_results --results-dir results/drone_interceptor_average_runs
```

![Average controller comparison](docs/average_overlay_comparison.png)

## Pipeline Summary

At startup, the launch file configures the scenario, controller mode, noise
settings, estimator topics, and output paths from the config files, then starts
the target, estimator, interceptor, monitoring, logging, and optional RViz and
`rqt_plot` processes together. The target behavior node samples a maneuver,
publishes `/target/desired_heading` and `/target/desired_speed`, and the target
dynamics node turns those commands into `/target/state` while also publishing
markers and TF for visualization.

That target state then flows through the estimator, which publishes both a
noisy measurement and a filtered estimate to simulate actual sensor data. The
interceptor side consumes the estimated target state and follows one of two
branches: the baseline branch uses `interceptor_guidance_node` to produce
`/interceptor/cmd_vel`, while the MPC branch uses `interceptor_mpc_node` to
produce `/interceptor_mpc/cmd_accel`. Their respective dynamics nodes convert
those commands into interceptor motion, the distance monitor publishes
`/intercept/distance`, and the results logger stores the run history plus
summary artifacts for later comparison.

## ROS Graph Overview

The diagram below summarizes the main nodes, topic groups, and data flow in the
baseline interception stack.

![ROS graph overview](docs/node_graph_overview.png)

## What The Nodes And Scripts Do

### Nodes

`target_behavior_node`

- Purpose: generates the target's desired heading and desired speed.
- Inputs: interceptor state topic when threat response is enabled, random seed,
  speed limits, threat radius.
- Outputs: `/target/desired_heading`, `/target/desired_speed`.

`target_dynamics_node`

- Purpose: simulates the target as a point-mass vehicle with speed and
  acceleration limits.
- Inputs: `/target/desired_heading`, `/target/desired_speed`.
- Outputs: `/target/state`, target marker, target path marker, TF frame.

`estimator_node`

- Purpose: builds the target measurement stream and publishes a filtered target
  estimate.
- Inputs: `/target/state`, measurement noise settings, Kalman filter settings.
- Outputs: `/target/state_noisy`, `/target/estimated_state`, measurement marker,
  estimated marker.

`interceptor_guidance_node`

- Purpose: computes velocity commands for the baseline interceptor.
- Inputs: `/target/estimated_state`, `/interceptor/state`, guidance mode,
  interceptor speed, interceptor acceleration limit, capture radius.
- Outputs: `/interceptor/cmd_vel`, `/intercept/marker`.

`interceptor_dynamics_node`

- Purpose: simulates the baseline interceptor as a point-mass vehicle.
- Inputs: `/interceptor/cmd_vel`.
- Outputs: `/interceptor/state`, interceptor marker, interceptor path marker,
  TF frame.

`interceptor_mpc_node`

- Purpose: computes acceleration commands for the MPC interceptor.
- Inputs: `/target/estimated_state`, `/interceptor_mpc/state`, speed and
  acceleration limits.
- Outputs: `/interceptor_mpc/cmd_accel`.

`interceptor_mpc_dynamics_node`

- Purpose: simulates the MPC interceptor with acceleration commands.
- Inputs: `/interceptor_mpc/cmd_accel`.
- Outputs: `/interceptor_mpc/state`, MPC interceptor marker, MPC interceptor
  path marker, TF frame.

`distance_monitor_node`

- Purpose: measures separation between the target and the active interceptor.
- Inputs: `/target/state` and a configured interceptor state topic.
- Outputs: `/intercept/distance` or another configured distance topic.

`results_logger_node`

- Purpose: records each run and saves plots plus summary metrics.
- Inputs: distance topic, controller label, profile name, spawn distance,
  threat-response setting, output directory.
- Outputs: `<run_label>.csv`, `<run_label>.json`, `<run_label>.png`.

### Scripts

`compare_results`

- Purpose: aggregates saved run artifacts and produces comparison figures.
- Inputs: a results directory containing run CSV/JSON/PNG files.
- Outputs: aggregate comparison PNGs and `aggregate_summary.md`.

Run it with:

```bash
compare_results --results-dir results/drone_interceptor
```

or:

```bash
python3 src/drone_interceptor/scripts/compare_results.py --results-dir results/drone_interceptor
```

`organize_results`

- Purpose: helps reorganize generated run artifacts into cleaner result folders.
- Inputs: generated result files.
- Outputs: moved or grouped result artifacts.

`plot_results`

- Purpose: creates averaged overlay plots and summary tables from repeated
  controller runs.
- Inputs: a results directory such as `results/drone_interceptor_average_runs`
  containing run CSV/JSON files.
- Outputs: `average_overlay_comparison.png` and
  `average_overlay_summary.md`.

`run_experiment`

- Purpose: launches predefined experiment configurations for repeatable runs.

`record_showcase`

- Purpose: records showcase runs for demonstration assets.
