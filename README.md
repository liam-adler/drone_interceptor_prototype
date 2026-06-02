# Drone Interceptor

This package is a ROS 2 simulation of a target drone and an interceptor drone.
It supports both a baseline guidance controller and an MPC-based controller so
you can compare how each interceptor behaves against the same target.

## How To Use The Package

1. Source ROS 2.
2. Build the package from the workspace root.
3. Source the workspace overlay.
4. Launch the simulation with the controller and scenario you want.
5. Watch the motion in RViz, inspect distances with `rqt_plot`, and review saved
   result artifacts in `results/drone_interceptor`.

Typical workflow:

```bash
source /opt/ros/humble/setup.bash
cd /home/liam/ros2_drone_intercept_ws
colcon build --packages-select drone_interceptor --symlink-install
source install/setup.bash
ros2 launch drone_interceptor target_sim.launch.py
```

What the package does during a run:

- The target chooses a heading and speed.
- The target dynamics node turns those commands into smooth motion.
- The estimator can add measurement noise and publish a filtered target state.
- The interceptor controller reads the target/interceptor states and commands a
  chase action.
- The interceptor dynamics node simulates the interceptor motion.
- A distance monitor publishes separation over time.
- A results logger saves CSV, JSON, and PNG outputs for later comparison.

## Build Commands

From the workspace root:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select drone_interceptor --symlink-install
source install/setup.bash
```

If you only changed Python code and want to rebuild quickly:

```bash
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

Use the base launch command and add any arguments you want:

```bash
ros2 launch drone_interceptor target_sim.launch.py <argument>:=<value> <argument>:=<value>
```

The following arguments can be passed:

- `controller_mode:=baseline`
- `controller_mode:=mpc`
- `guidance_mode:=pure_pursuit`
- `guidance_mode:=lead_pursuit`
- `guidance_mode:=acceleration_aware_lead_pursuit`
- `controller_preset:=baseline_pure_pursuit`
- `controller_preset:=baseline_lead_pursuit`
- `controller_preset:=baseline_acceleration_aware`
- `controller_preset:=mpc_default`
- `profile:=matched`
- `profile:=target_faster`
- `profile:=target_more_maneuverable`
- `profile:=target_advantaged`
- `threat_response:=false`
- `spawn_distance:=40.0`
- `random_seed:=4`
- `scenario:=target_advantaged_close_start`
- `scenario:=target_advantaged_no_threat`
- `scenario:=target_advantaged_medium_noise`
- `open_rviz:=false`
- `open_plot:=false`
- `output_dir:=results/my_experiments`

Examples:

```bash
ros2 launch drone_interceptor target_sim.launch.py controller_mode:=mpc
ros2 launch drone_interceptor target_sim.launch.py controller_preset:=mpc_default
ros2 launch drone_interceptor target_sim.launch.py controller_mode:=baseline guidance_mode:=lead_pursuit
ros2 launch drone_interceptor target_sim.launch.py controller_preset:=baseline_acceleration_aware
ros2 launch drone_interceptor target_sim.launch.py controller_mode:=baseline guidance_mode:=pure_pursuit profile:=matched
ros2 launch drone_interceptor target_sim.launch.py controller_preset:=mpc_default scenario:=target_advantaged_close_start
ros2 launch drone_interceptor target_sim.launch.py open_rviz:=false
ros2 launch drone_interceptor target_sim.launch.py open_plot:=false
ros2 launch drone_interceptor target_sim.launch.py open_rviz:=false open_plot:=false
ros2 launch drone_interceptor target_sim.launch.py output_dir:=results/my_experiments
```

### Useful ROS 2 Runtime Checks

Show all running nodes:

```bash
ros2 node list
```

Show all topics:

```bash
ros2 topic list
```

Watch baseline controller commands:

```bash
ros2 topic echo /interceptor/cmd_vel
```

Watch the interceptor state:

```bash
ros2 topic echo /interceptor/state
```

Watch the target estimate used by the baseline controller:

```bash
ros2 topic echo /target/estimated_state
```

Watch the live distance:

```bash
ros2 topic echo /intercept/distance
```

Open the distance plot manually:

```bash
ros2 run rqt_plot rqt_plot
```

Useful plot fields:

```text
/intercept/distance/data
/intercept/distance_mpc/data
```

## Screenshot

The figure below shows the averaged overlay comparison across five 60-second
runs for each guidance/controller method.

![Average controller comparison](docs/average_overlay_comparison.png)

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

## Saved Artifacts

Each run writes artifacts into `results/drone_interceptor` by default:

- `<run_label>.csv` with `time_s` and `distance_m`
- `<run_label>.json` with summary metrics
- `<run_label>.png` with the distance trace

Aggregate plotting generates files such as:

- `aggregate/distance_over_time__*.png`
- `aggregate/summary_comparison.png`
- `aggregate/capture_time_comparison.png`
- `aggregate/min_distance_comparison.png`
- `aggregate/capture_rate_comparison.png`
- `aggregate/aggregate_summary.md`

## Notes

- `rviz2` must be installed for the RViz window to open.
- `rqt_plot` must be installed for the plot window to open.
- After code changes, rebuild with `colcon build --packages-select drone_interceptor --symlink-install` and re-source `install/setup.bash`.
