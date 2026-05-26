# Drone Interceptor

This package is a ROS 2 simulation of a target drone and an interceptor drone.

The target:
- moves with simple point-mass dynamics
- changes heading randomly
- can optionally speed up when the interceptor gets close

The interceptor:
- moves with the same point-mass style dynamics
- reads the target state
- chases the target with pursuit guidance

The package also includes:
- an RViz config for visualization
- a live distance monitor topic
- an `rqt_plot` view for target/interceptor separation

## Nodes

`target_behavior_node`
- Publishes `/target/desired_heading`
- Publishes `/target/desired_speed`
- Randomizes target heading and cruise speed
- Optionally boosts target speed when the interceptor enters a threat radius

`target_dynamics_node`
- Subscribes to `/target/desired_heading` and `/target/desired_speed`
- Simulates target position and velocity
- Publishes `/target/state`
- Publishes target marker and target path marker

`interceptor_guidance_node`
- Subscribes to `/target/state`
- Subscribes to `/interceptor/state`
- Publishes `/interceptor/cmd_vel`
- Uses pursuit guidance to chase the target

`interceptor_dynamics_node`
- Subscribes to `/interceptor/cmd_vel`
- Simulates interceptor position and velocity
- Publishes `/interceptor/state`
- Publishes interceptor marker and interceptor path marker

`distance_monitor_node`
- Subscribes to `/target/state`
- Subscribes to `/interceptor/state`
- Publishes `/intercept/distance`

## What The Simulation Does

1. The target behavior node chooses a heading and speed.
2. The target dynamics node turns that into smooth motion with speed and acceleration limits.
3. The interceptor guidance node looks at both drone states and commands a chase velocity.
4. The interceptor dynamics node simulates the interceptor motion.
5. The distance monitor node publishes the current separation between the drones.
6. RViz shows the drones, paths, TF frames, and intercept marker.
7. `rqt_plot` can show the distance over time.

## Build

Run these commands from the workspace root:

```bash
cd /home/liam/ros2_drone_intercept_ws
colcon build --packages-select drone_interceptor
source install/setup.bash
```

If your shell does not already have ROS 2 sourced, do this first:

```bash
source /opt/ros/<your_ros_distro>/setup.bash
```

Then build again:

```bash
cd /home/liam/ros2_drone_intercept_ws
colcon build --packages-select drone_interceptor
source install/setup.bash
```

## Main Launch Command

```bash
ros2 launch drone_interceptor target_sim.launch.py
```

Default behavior:
- target and interceptor start 30 meters apart
- RViz opens automatically
- `rqt_plot` opens automatically
- the plot window opens after a short delay, and you enter the topic manually
- target threat response is enabled
- default profile is `target_advantaged`

## Launch Options

`profile`
- Selects the speed and maneuverability setup.

Available profiles:
- `matched`
- `target_faster`
- `target_more_maneuverable`
- `target_advantaged`

`threat_response`
- `true` or `false`
- When `true`, the target speeds up as the interceptor gets close.

`spawn_distance`
- Initial separation in meters.

`open_rviz`
- `true` or `false`

`open_plot`
- `true` or `false`

## Example Commands

Default run:

```bash
ros2 launch drone_interceptor target_sim.launch.py
```

Matched target and interceptor:

```bash
ros2 launch drone_interceptor target_sim.launch.py profile:=matched
```

Target faster only:

```bash
ros2 launch drone_interceptor target_sim.launch.py profile:=target_faster
```

Target more maneuverable only:

```bash
ros2 launch drone_interceptor target_sim.launch.py profile:=target_more_maneuverable
```

Target both faster and more maneuverable:

```bash
ros2 launch drone_interceptor target_sim.launch.py profile:=target_advantaged
```

Disable threat response:

```bash
ros2 launch drone_interceptor target_sim.launch.py threat_response:=false
```

Spawn 40 meters apart:

```bash
ros2 launch drone_interceptor target_sim.launch.py spawn_distance:=40.0
```

Run without RViz:

```bash
ros2 launch drone_interceptor target_sim.launch.py open_rviz:=false
```

Run without the live plot:

```bash
ros2 launch drone_interceptor target_sim.launch.py open_plot:=false
```

Run with matched profile and no threat response:

```bash
ros2 launch drone_interceptor target_sim.launch.py profile:=matched threat_response:=false
```

## Distance Plot

The distance monitor publishes:

```bash
/intercept/distance
```

The plotted numeric field is:

```bash
/intercept/distance/data
```

If you want to open the plot manually:

```bash
ros2 run rqt_plot rqt_plot
```

If `rqt_plot` opens but shows `Topic/Field to enter something`:

1. Enter `/intercept/distance/data` in the field at the top and press Enter.
2. If that still does not draw, enter `/intercept/distance` and select the `data` field.
3. Verify the topic is publishing with:

```bash
ros2 topic echo /intercept/distance
```

You should see changing values like:

```text
data: 29.8
data: 29.6
data: 29.4
```

## Useful ROS 2 Checks

Show all running nodes:

```bash
ros2 node list
```

Show all topics:

```bash
ros2 topic list
```

Echo the live distance:

```bash
ros2 topic echo /intercept/distance
```

Echo the target state:

```bash
ros2 topic echo /target/state
```

Echo the interceptor state:

```bash
ros2 topic echo /interceptor/state
```

## Notes

- `rviz2` must be installed for the RViz window to open.
- `rqt_plot` must be installed for the plot window to open.
- After code changes, rebuild with `colcon build --packages-select drone_interceptor` and re-source `install/setup.bash`.
