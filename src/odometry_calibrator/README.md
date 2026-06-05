# Odometry Calibrator

ROS 2 Python package for straight-line wheel odometry scale calibration.

The `odom_linear_calibrator` node subscribes to `/odom`, drives along the configured `/cmd_vel` linear axis, waits for an operator-entered measured distance, and calculates:

```text
K_x = D_actual / D_odom_x
K_y = D_actual / D_odom_y
```

The measurement input is behind a `MeasurementProvider` interface. The initial implementation uses `CliMeasurementProvider`; a future marker-based provider can be added without coupling it to the calibration node.

The `/odom` subscription uses ROS 2 sensor-data QoS so it can connect to best-effort odometry publishers commonly used on robots. The `/cmd_vel` publisher keeps the default reliable QoS.

## Nodes

```bash
ros2 run odometry_calibrator odom_linear_calibrator
ros2 run odometry_calibrator mock_odom_publisher
```

## Launch Mock Test

```bash
ros2 launch odometry_calibrator test_calibration.launch.py
```

## Build

```bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
```

## Main Parameters

Defaults are provided in `config/odom_linear_calibration.yaml`.

```yaml
odom_topic: /odom
cmd_vel_topic: /cmd_vel
axis: "x"
target_distance: 1.0
distance_tolerance: 0.005
kp: 0.4
max_velocity: 0.1
min_velocity: 0.01
max_acceleration: 0.05
control_rate_hz: 20.0
stop_publish_rate_hz: 10.0
motion_timeout_sec: 30.0
keep_alive_after_done: true
```

## Expected Flow

1. `odom_linear_calibrator` waits for the first valid `/odom` sample and latches it as the start pose.
2. The node publishes `/cmd_vel.linear.x` when `axis: x`, or `/cmd_vel.linear.y` when `axis: y`, with acceleration limiting.
3. Near the target distance, on timeout, or below minimum commanded velocity, it transitions to `WAIT_FOR_MEASUREMENT`.
4. While waiting, the node continuously publishes zero velocity at least 10 Hz.
5. Enter a measured distance in meters:

```text
Enter actual measured distance [m]: 0.985
```

6. The node prints `D_odom`, `D_actual`, and `K_x` or `K_y`.

For y-axis calibration from the CLI, quote the value or use the shorthand below. ROS 2
parses unquoted `y` as YAML boolean true, and this node maps that value to the y axis:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y
```
