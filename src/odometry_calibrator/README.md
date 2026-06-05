# Odometry Calibrator

ROS 2 Python package for straight-line odometry scale calibration.

Omni drive robots can calibrate x-axis and y-axis odometry independently.

## Concept

The `odom_linear_calibrator` node moves the robot along one selected linear axis, reads `/odom`, waits for the operator to enter the actual measured travel distance, then calculates:

```text
K_x = D_actual / D_odom_x
K_y = D_actual / D_odom_y
```

`/cmd_vel` is used only to move the robot. `/odom` is used to calculate the odometry-reported travel distance. `D_actual` must come from an external measurement, such as a tape measure, floor marks, marker tracking, or another ground-truth source.

The measurement input is behind a `MeasurementProvider` interface. The initial implementation uses `CliMeasurementProvider`; a future marker-based provider can be added without coupling it to the calibration node.

The `/odom` subscription uses ROS 2 sensor-data QoS so it can connect to best-effort odometry publishers commonly used on robots. The `/cmd_vel` publisher keeps the default reliable QoS.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
```

## Nodes

```bash
ros2 run odometry_calibrator odom_linear_calibrator
ros2 run odometry_calibrator mock_odom_publisher
```

## X-Axis Calibration

Moves with:

```text
/cmd_vel.linear.x
```

Calculates:

```text
K_x = D_actual / D_odom_x
```

Run:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=x
```

## Y-Axis Calibration

Moves with:

```text
/cmd_vel.linear.y
```

Calculates:

```text
K_y = D_actual / D_odom_y
```

Run:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y
```

ROS 2 parameter parsing treats unquoted `y` as YAML boolean true; this node maps that value to the y axis. Quoted values are also supported in YAML files.

## Measurement Input

After the robot stops, the node prompts:

```text
Enter actual measured distance [m]:
```

Enter the measured real-world travel distance in meters:

```text
Enter actual measured distance [m]: 0.985
```

Example output:

```text
D_odom   : 1.000 m
D_actual : 0.985 m
K_x      : 0.985000
```

or:

```text
D_odom   : 1.000 m
D_actual : 0.972 m
K_y      : 0.972000
```

## Parameters

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

## Launch Mock Test

The package includes a mock odometry publisher for smoke testing.

```bash
ros2 launch odometry_calibrator test_calibration.launch.py
```

For y-axis mock testing:

```bash
ros2 run odometry_calibrator mock_odom_publisher --ros-args -p mock_axis:=y
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y
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

## Notes

- Run x and y calibration separately for omni drive robots.
- Calibrate on a flat surface with enough clearance.
- Use a consistent robot reference point when measuring `D_actual`.
- Keep motion slow enough to reduce slip and overshoot.
