# Odometry Calibrator

ROS 2 Python package for straight-line odometry scale calibration.

Omni drive robots can calibrate +x, -x, +y, and -y odometry independently.

## Concept

The `odom_linear_calibrator` node moves the robot along one selected linear axis and direction, reads odometry from the configured odom topic, waits for the operator to enter the actual measured travel distance, then calculates:

```text
K_x = D_actual / D_odom_x
K_y = D_actual / D_odom_y
```

`/cmd_vel` is used only to move the robot. The configured odom topic is used to calculate the odometry-reported travel distance. `D_actual` must come from an external measurement, such as a tape measure, floor marks, marker tracking, or another ground-truth source.

The measurement input is behind a `MeasurementProvider` interface. The initial implementation uses `CliMeasurementProvider`; a future marker-based provider can be added without coupling it to the calibration node.

The configured odom topic subscription uses ROS 2 sensor-data QoS so it can connect to best-effort odometry publishers commonly used on robots. The `/cmd_vel` publisher keeps the default reliable QoS.

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
ros2 run odometry_calibrator motion_data_recorder
```

## Documentation

- Detailed usage and architecture: `docs/usage_and_architecture.md`
- Motion data recording details: `docs/data_recording.md`

## Direction Examples

+x direction:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=x -p direction:=1
```

-x direction:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=x -p direction:=-1
```

+y direction:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=1
```

-y direction:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=-1
```

The command velocity mapping is:

```text
axis: x, direction:  1  -> /cmd_vel.linear.x = +v_cmd
axis: x, direction: -1  -> /cmd_vel.linear.x = -v_cmd
axis: y, direction:  1  -> /cmd_vel.linear.y = +v_cmd
axis: y, direction: -1  -> /cmd_vel.linear.y = -v_cmd
```

ROS 2 parameter parsing treats unquoted `y` as YAML boolean true; this node maps that value to the y axis. Use quoted values in YAML files, for example `axis: "y"`.

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
Axis      : x
Direction : +1
D_odom    : 0.500 m
D_actual  : 0.492 m
K_x(+1)   : 0.984000
```

or:

```text
Axis      : y
Direction : -1
D_odom    : 0.500 m
D_actual  : 0.486 m
K_y(-1)   : 0.972000
```

## Parameters

Defaults are provided in `config/odom_linear_calibration.yaml`.

```yaml
odom_topic: /odom
cmd_vel_topic: /cmd_vel
axis: "x"
direction: 1
target_distance: 0.5
distance_tolerance: 0.005
kp: 0.4
max_velocity: 0.1
min_velocity: 0.01
max_acceleration: 0.05
control_rate_hz: 20.0
stop_publish_rate_hz: 10.0
motion_timeout_sec: 20.0
keep_alive_after_done: true
```

Mock smoke-test defaults:

```yaml
mock_axis: "x"
direction: 1
publish_rate_hz: 20.0
mock_speed: 0.05
```

## Launch Mock Test

The package includes a mock odometry publisher for smoke testing.

```bash
ros2 launch odometry_calibrator test_calibration.launch.py
```

The mock launch overrides `odom_topic` to `/odometry_calibrator/mock_odom` so it does not mix with a real robot `/odom` publisher.

For y-axis mock testing:

```bash
ros2 run odometry_calibrator mock_odom_publisher --ros-args -p mock_axis:=y -p direction:=1
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=1
```

## Motion Data Recording

`motion_data_recorder` is a separate node for recording motion data. It does not command the robot and does not calculate the final calibration scale factor. The role split is:

```text
odom_linear_calibrator
- command one calibration move
- wait for measured distance input
- calculate K_x or K_y

motion_data_recorder
- subscribe to /cmd_vel and the configured odom topic
- optionally subscribe to a PoseStamped reference pose topic
- write CSV rows at a fixed rate
- print a shutdown summary
```

Run with the default config:

```bash
ros2 launch odometry_calibrator data_recording.launch.py
```

Or run directly:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

CSV files are written to `logs/` by default:

```text
logs/motion_20260605_203015.csv
```

Base CSV columns:

```csv
time_sec,cmd_vx,cmd_vy,cmd_wz,odom_x,odom_y,odom_yaw,odom_distance,axis_distance,remaining_distance
```

`time_sec` is elapsed time after CSV recording actually starts. `odom_distance` is calculated from the first latched odometry pose.

Reference pose recording can be enabled with a `geometry_msgs/msg/PoseStamped` topic:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p use_reference_pose:=true \
  -p reference_pose_topic:=/reference_pose
```

When reference pose recording is enabled, the CSV also includes:

```csv
ref_x,ref_y,ref_yaw,ref_distance,ref_axis_distance,odom_ref_error,error_rate,scale_estimate
```

`ref_distance` is calculated from the first latched reference pose.

Example summary:

```text
=== Motion Data Summary ===
Axis                : x
Direction           : +1
Target distance     : 1.000 m
Final odom distance : 1.003 m
Axis distance       : 1.001 m
Remaining distance  : -0.001 m
Duration            : 10.84 s
Max cmd velocity    : 0.100 m/s
Avg cmd velocity    : 0.092 m/s
CSV saved           : logs/motion_20260605_203015.csv
```

The recorder only writes CSV data and prints a summary. Visualization should be done with external tools such as PlotJuggler, spreadsheets, notebooks, or separate optional scripts.

More details are in `docs/data_recording.md`.

## Expected Flow

1. `odom_linear_calibrator` waits for the first valid odometry sample on the configured odom topic and latches it as the start pose.
2. The node publishes signed `/cmd_vel.linear.x` or `/cmd_vel.linear.y` according to `axis` and `direction`, with acceleration limiting.
3. Near the target distance, on timeout, or below minimum commanded velocity, it transitions to `WAIT_FOR_MEASUREMENT`.
4. While waiting, the node continuously publishes zero velocity at least 10 Hz.
5. Enter a measured distance in meters:

```text
Enter actual measured distance [m]: 0.985
```

6. The node prints `Axis`, `Direction`, `D_odom`, `D_actual`, and `K_x(+/-1)` or `K_y(+/-1)`.

## Notes

- Run x and y calibration separately for omni drive robots.
- Calibrate on a flat surface with enough clearance.
- Use a consistent robot reference point when measuring `D_actual`.
- Keep motion slow enough to reduce slip and overshoot.

## Safety Checklist Before Real Robot Test

- Verify behavior with the mock launch before connecting to a real AMR.
- Confirm that `/cmd_vel` matches the real robot control topic.
- Confirm that `/odom` matches the real wheel odometry topic.
- Confirm that the `axis` and `direction` combination matches the intended travel direction.
- Prepare an emergency stop or another manual stop method.
- Secure a straight driving space longer than `target_distance`.
- Start with conservative `max_velocity` and `max_acceleration` values.
- First check command direction with wheels lifted or at very low speed.
- Confirm that `/cmd_vel` is continuously published as zero while in `WAIT_FOR_MEASUREMENT`.
