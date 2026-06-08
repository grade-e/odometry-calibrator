# Platform Inspector Integration

## Purpose

`odometry-calibrator` can run as a managed external L6 calibration worker for
`platform-inspector` while remaining an independent ROS 2 package.

The integration contract is based on ROS parameters, process exit codes, and
machine-readable artifacts. `platform-inspector` does not need to import Python
modules from this package.

## Execution Model

`platform-inspector` starts `odom_linear_calibrator` with `ros2 run` or
`ros2 launch`, passes a job artifact directory through `output_dir`, and
collects the generated files after the process exits.

```text
platform-inspector
  -> ros2 run / ros2 launch odometry_calibrator
  -> output_dir=/path/to/job/artifacts/odom_x_plus
  -> collect result.json, events.jsonl, optional motion CSV
  -> map process status to JobResult
```

The calibrator publishes `/cmd_vel` as
`geometry_msgs/msg/TwistStamped` and reads the configured odometry topic.

## Required Parameters

| Parameter | Required | Description |
| --- | --- | --- |
| `axis` | yes | Calibration axis: `x` or `y` |
| `direction` | yes | Direction: `1` or `-1` |
| `target_distance` | yes | Requested odometry travel distance in meters |
| `odom_topic` | yes | Odometry input topic |
| `cmd_vel_topic` | yes | TwistStamped command output topic |
| `output_dir` | yes | Job artifact directory |
| `measurement_source` | yes | `cli` or `parameter` |
| `actual_distance_m` | when `parameter` | Actual measured distance in meters |

`measurement_source=cli` preserves the interactive workflow. The robot stops,
then the operator enters the actual measured distance.

`measurement_source=parameter` uses `actual_distance_m` and does not require a
CLI input stream. This is the preferred mode for managed
`platform-inspector` jobs.

`actual_distance_m` must be finite and satisfy:

```text
0.0 < actual_distance_m <= 10.0
```

## Output Artifacts

The calibrator creates `output_dir` when it does not exist.

| Artifact | Description |
| --- | --- |
| `result.json` | Final machine-readable calibration result |
| `events.jsonl` | JSON Lines state, warning, result, and error events |
| `motion.csv` | Optional CSV produced by `motion_data_recorder` |

`result.json` includes:

```text
schema_version, tool, task_type, status, axis, direction,
target_distance_m, odom_distance_m, actual_distance_m, scale_factor,
candidate, topics, control, artifacts, message
```

Candidate parameter names follow this mapping:

| Axis | Direction | Candidate parameter |
| --- | --- | --- |
| `x` | `1` | `wheel_odom_scale_x_positive` |
| `x` | `-1` | `wheel_odom_scale_x_negative` |
| `y` | `1` | `wheel_odom_scale_y_positive` |
| `y` | `-1` | `wheel_odom_scale_y_negative` |

The candidate is never applied automatically. It is marked with
`apply_policy=manual_review_required`.

## Exit Codes

| Exit code | Name | Meaning |
| --- | --- | --- |
| `0` | `SUCCESS` | Calibration completed successfully |
| `1` | `CALIBRATION_FAILED` | Calibration could not compute a valid result |
| `2` | `INVALID_PARAMETER` | Parameter validation failed |
| `3` | `TOPIC_UNAVAILABLE` | Reserved for topic availability failures |
| `4` | `MOTION_TIMEOUT` | Motion timed out before target distance |
| `5` | `MEASUREMENT_CANCELLED` | User cancelled the run |
| `10` | `INTERNAL_ERROR` | Unexpected internal error |

## Example

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p output_dir:=/tmp/platform_inspector/job_001/artifacts/odom_x_plus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.985 \
  -p keep_alive_after_done:=false
```

After the process exits:

```bash
cat /tmp/platform_inspector/job_001/artifacts/odom_x_plus/result.json
cat /tmp/platform_inspector/job_001/artifacts/odom_x_plus/events.jsonl
```

## Motion Data Recorder

`motion_data_recorder` already accepts `output_dir` and `output_prefix`.
For managed jobs, pass an explicit `output_dir` inside the same job artifact
tree.

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p output_dir:=/tmp/platform_inspector/job_001/artifacts/motion \
  -p output_prefix:=motion
```

The recorder keeps its existing shutdown summary and `CSV saved` console output.
Future integrations can add the recorder CSV relative path to
`result.json.artifacts.motion_csv`.

## Safety Notes

Confirm the following before running a managed calibration job:

- The robot has enough clear space for the selected axis and direction.
- `cmd_vel_topic` uses `geometry_msgs/msg/TwistStamped`.
- Twist-based robots or simulations use a relay or adapter.
- The configured odometry topic is the source being calibrated.
- Timeout or cancellation still publishes stop commands before shutdown when
  the calibrator node is alive.
