# Odometry Calibrator

ROS 2 Python package for straight-line wheel odometry scale calibration.

The `odom_linear_calibrator` node subscribes to `/odom`, drives forward with `/cmd_vel`, waits for an operator-entered measured distance, and calculates:

```text
K_linear = D_actual / D_odom
```

The measurement input is behind a `MeasurementProvider` interface. The initial implementation uses `CliMeasurementProvider`; a future marker-based provider can be added without coupling it to the calibration node.

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
2. The node publishes forward `/cmd_vel` commands with acceleration limiting.
3. Near the target distance, on timeout, or below minimum commanded velocity, it transitions to `WAIT_FOR_MEASUREMENT`.
4. While waiting, the node continuously publishes zero velocity at least 10 Hz.
5. Enter a measured distance in meters:

```text
Enter actual measured distance [m]: 0.985
```

6. The node prints `D_odom`, `D_actual`, and `K_linear`.
