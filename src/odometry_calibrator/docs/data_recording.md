# Motion Data Recording

For the full package architecture and end-to-end workflows, see `usage_and_architecture.md`.

## Purpose

`motion_data_recorder` records time-aligned command velocity, odometry, and optional reference pose data to CSV. It is intended for real AMR low-speed checks, calibrator runs, mock smoke tests, rosbag replay analysis, and future external reference comparisons.

The recorder only writes CSV data and prints a summary. Visualization should be done with external tools such as PlotJuggler, spreadsheets, notebooks, or separate optional scripts.

## Role Separation

`odom_linear_calibrator` drives the robot, waits for the operator's measured distance input, and calculates the odometry scale factor.

`motion_data_recorder` does not command the robot and does not calculate final calibration constants. It subscribes to `/cmd_vel`, the configured odom topic, and optionally a `PoseStamped` reference pose topic, then writes raw motion data for later analysis.

## Basic Run

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
ros2 launch odometry_calibrator data_recording.launch.py
```

By default, CSV files are saved under:

```text
logs/motion_<timestamp>.csv
```

When only `test_mock_odom_publisher` is running, command velocity columns can remain zero because no node is publishing `/cmd_vel`. The test mock odometry publisher is a smoke-test support executable and fixed-speed odometry source, not a robot simulator or closed-loop robot model, so it keeps increasing odometry until the mock node is stopped.

To override parameters:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5 \
  -p output_prefix:=calibration_x_pos
```

## Recording During Calibration

Run the recorder in one terminal:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

Run the calibrator in another terminal:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

The recorder starts writing rows only after it has latched the first odometry sample. If reference pose recording is enabled, it also waits for the first valid reference pose.

`time_sec` is measured from the moment CSV recording actually starts. Distance origins are separate from the time origin: `odom_distance` is calculated from the first latched odometry pose, and `ref_distance` is calculated from the first latched reference pose.

For mock-based calibration smoke tests, stop the recorder shortly after the calibrator reaches `DONE` if you want the recorder summary to stay close to `target_distance`. If the fixed-speed mock publisher continues running, odometry and summary distance will continue increasing even though `/cmd_vel` has returned to zero.

## CSV Columns

Base columns:

```csv
time_sec,cmd_vx,cmd_vy,cmd_wz,odom_x,odom_y,odom_yaw,odom_distance,axis_distance,remaining_distance
```

- `time_sec`: elapsed time after CSV recording starts
- `cmd_vx`, `cmd_vy`, `cmd_wz`: latest `/cmd_vel` values
- `odom_x`, `odom_y`, `odom_yaw`: latest odometry pose
- `odom_distance`: 2D distance from the odometry start pose
- `axis_distance`: positive distance along `axis` and `direction`
- `remaining_distance`: `target_distance - axis_distance`

With `use_reference_pose=true`, the recorder appends:

```csv
ref_x,ref_y,ref_yaw,ref_distance,ref_axis_distance,odom_ref_error,error_rate,scale_estimate
```

- `ref_distance`: 2D distance from the reference start pose
- `ref_axis_distance`: positive reference distance along `axis` and `direction`
- `odom_ref_error`: `axis_distance - ref_axis_distance`
- `error_rate`: `odom_ref_error / ref_axis_distance`
- `scale_estimate`: `ref_axis_distance / axis_distance`

Ratio fields are empty when the denominator is too small.

## Reference Pose

The first implementation supports only:

```text
geometry_msgs/msg/PoseStamped
```

Example:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p use_reference_pose:=true \
  -p reference_pose_topic:=/reference_pose
```

Use this for marker tracking, simulator ground truth, motion capture, or any external system that publishes a compatible `PoseStamped`.

## Rosbag

Record raw ROS topics for replay or comparison:

```bash
ros2 bag record /cmd_vel /odom
```

With reference pose:

```bash
ros2 bag record /cmd_vel /odom /reference_pose
```

During rosbag replay, start `motion_data_recorder` to regenerate CSV from the replayed topics.

## PlotJuggler

Useful ROS fields to inspect:

```text
/cmd_vel.linear.x
/cmd_vel.linear.y
/odom.pose.pose.position.x
/odom.pose.pose.position.y
```

You can also load the recorder CSV directly into PlotJuggler and inspect `axis_distance`, `remaining_distance`, `odom_ref_error`, and `scale_estimate`.

## Spreadsheet Review

Open the generated CSV in Excel, LibreOffice, Google Sheets, or another spreadsheet. The most useful columns for a quick sanity check are:

```text
time_sec
cmd_vx
cmd_vy
odom_x
odom_y
odom_distance
axis_distance
remaining_distance
```
