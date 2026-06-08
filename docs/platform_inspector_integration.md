# Platform Inspector 연동

## 목적

`odometry-calibrator`는 독립 ROS 2 패키지로 유지하면서도,
`platform-inspector`가 L6 calibration job의 외부 worker로 실행하고
결과를 수집할 수 있다.

이 연동은 Python module import 계약이 아니다. `platform-inspector`는 이
패키지 내부 코드를 직접 호출하지 않고, 다음 계약만 사용한다.

- ROS 2 실행 명령: `ros2 run` 또는 `ros2 launch`
- ROS parameter 입력
- process exit code
- `result.json`, `events.jsonl`, optional CSV artifact

즉, `odometry-calibrator`는 사람이 직접 실행할 수 있는 calibration 도구이자,
외부 운영 시스템이 job 단위로 관리할 수 있는 managed external worker다.

## 실행 모델

기본 흐름은 다음과 같다.

```text
platform-inspector
  -> artifact directory 생성
  -> ros2 run / ros2 launch odometry_calibrator 실행
  -> output_dir parameter 전달
  -> result.json / events.jsonl / optional motion CSV 수집
  -> process exit code와 result.json.status를 JobResult로 변환
```

`odom_linear_calibrator`는 configured odometry topic을 구독하고,
configured command velocity topic으로 `geometry_msgs/msg/TwistStamped`를
publish한다.

```text
input : odom_topic      nav_msgs/msg/Odometry
output: cmd_vel_topic   geometry_msgs/msg/TwistStamped
```

`geometry_msgs/msg/Twist` 기반 robot 또는 simulation에 연결하려면
`TwistStamped`를 `Twist`로 변환하는 relay/adapter가 필요하다.

## 사전 확인

패키지 빌드와 overlay source를 먼저 수행한다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --packages-select odometry_calibrator
source install/setup.bash
```

실행 파일을 확인한다.

```bash
ros2 pkg executables odometry_calibrator
```

기대 출력:

```text
odometry_calibrator motion_data_recorder
odometry_calibrator odom_linear_calibrator
odometry_calibrator test_mock_odom_publisher
```

로봇 또는 simulation이 이미 실행 중이면 topic type을 확인한다.

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /odom -v
ros2 topic echo /odom --once
```

확인 기준:

- `/cmd_vel` subscriber가 `geometry_msgs/msg/TwistStamped`를 받을 수 있다.
- `/odom` publisher가 `nav_msgs/msg/Odometry`를 publish한다.
- `/odom` pose 값이 정상적으로 갱신된다.

topic 이름이 다르면 이후 명령에서 `odom_topic`, `cmd_vel_topic`을 실제
이름으로 override한다.

## 필수 파라미터

| 파라미터 | 필수 여부 | 설명 |
| --- | --- | --- |
| `axis` | 필수 | calibration 축. `x` 또는 `y` |
| `direction` | 필수 | 진행 방향. `1` 또는 `-1` |
| `target_distance` | 필수 | odom 기준 목표 이동 거리, meter |
| `odom_topic` | 필수 | odometry 입력 topic |
| `cmd_vel_topic` | 필수 | TwistStamped command 출력 topic |
| `output_dir` | 필수 | job artifact directory |
| `measurement_source` | 필수 | `cli` 또는 `parameter` |
| `actual_distance_m` | `parameter` 모드에서 필수 | 실제 측정 거리, meter |

`measurement_source=cli`는 기존 대화형 동작을 유지한다. 로봇이 정지한 뒤
작업자가 터미널에 실제 이동 거리를 입력한다.

`measurement_source=parameter`는 `actual_distance_m` parameter를 사용한다.
CLI input stream이 필요 없으므로 `platform-inspector`가 관리하는 자동 job에서는
이 모드를 우선 사용한다.

`actual_distance_m` 검증 조건:

```text
0.0 < actual_distance_m <= 10.0
finite value
```

## Artifact 구조

`odom_linear_calibrator`는 `output_dir`가 없으면 자동으로 생성한다.

권장 job artifact 구조:

```text
/tmp/platform_inspector/job_001/artifacts/
  odom_x_plus/
    result.json
    events.jsonl
  motion/
    motion_20260608_221500.csv
```

생성 artifact:

| Artifact | 설명 |
| --- | --- |
| `result.json` | 최종 calibration 결과. 기계가 읽는 표준 결과 파일 |
| `events.jsonl` | 상태 전이, warning, measurement, result, error 이벤트 |
| `motion.csv` | 선택 사항. `motion_data_recorder`가 생성하는 motion 기록 |

`result.json` 주요 필드:

```text
schema_version
tool
task_type
status
axis
direction
target_distance_m
odom_distance_m
actual_distance_m
scale_factor
candidate
topics
control
artifacts
message
```

candidate parameter 이름은 다음 규칙을 따른다.

| Axis | Direction | Candidate parameter |
| --- | --- | --- |
| `x` | `1` | `wheel_odom_scale_x_positive` |
| `x` | `-1` | `wheel_odom_scale_x_negative` |
| `y` | `1` | `wheel_odom_scale_y_positive` |
| `y` | `-1` | `wheel_odom_scale_y_negative` |

candidate는 자동 적용하지 않는다.

```json
{
  "candidate": {
    "parameter_name": "wheel_odom_scale_x_positive",
    "value": 0.982054,
    "apply_policy": "manual_review_required"
  }
}
```

`platform-inspector`는 이 값을 추천 보정값으로 저장하거나 UI에 표시할 수 있지만,
robot 설정 파일이나 live parameter에 자동 적용하면 안 된다.

## Exit Code

| Exit code | 이름 | 의미 |
| --- | --- | --- |
| `0` | `SUCCESS` | calibration 성공 |
| `1` | `CALIBRATION_FAILED` | 유효한 결과를 계산하지 못함 |
| `2` | `INVALID_PARAMETER` | parameter 검증 실패 |
| `3` | `TOPIC_UNAVAILABLE` | topic 사용 불가 예약 코드 |
| `4` | `MOTION_TIMEOUT` | 목표 거리 도달 전 motion timeout |
| `5` | `MEASUREMENT_CANCELLED` | 사용자가 실행 취소 |
| `10` | `INTERNAL_ERROR` | 예상하지 못한 내부 오류 |

권장 처리:

- process exit code가 `0`이면 `result.json.status`도 확인한 뒤 성공 처리한다.
- process exit code가 `0`이 아니면 job은 실패 또는 취소로 처리한다.
- 가능한 경우 `events.jsonl`의 마지막 `error` 또는 `warning` 이벤트를 UI에 표시한다.
- `result.json`이 없으면 process startup 또는 parameter validation 단계에서 실패한 것으로 본다.

## 사용 예시 1: platform-inspector 비대화형 calibration

이 예시는 외부 job runner가 CLI 입력 없이 calibration을 실행하는 기본 형태다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash
source /path/to/odometry_calibrator_ws/install/setup.bash

JOB_DIR=/tmp/platform_inspector/job_001
ARTIFACT_DIR=$JOB_DIR/artifacts/odom_x_plus

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p output_dir:=$ARTIFACT_DIR \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.985 \
  -p keep_alive_after_done:=false
```

실행 후 artifact를 확인한다.

```bash
cat $ARTIFACT_DIR/result.json
cat $ARTIFACT_DIR/events.jsonl
```

JSON 유효성 확인:

```bash
python3 -m json.tool $ARTIFACT_DIR/result.json >/dev/null

python3 - <<PY
import json
from pathlib import Path

path = Path('$ARTIFACT_DIR/events.jsonl')
for line_number, line in enumerate(path.read_text().splitlines(), 1):
    json.loads(line)
print(f'{line_number} events valid')
PY
```

예상 `events.jsonl` 흐름:

```json
{"time_sec":0.000,"event":"started","axis":"x","direction":1}
{"time_sec":0.052,"event":"state_transition","from":"INIT","to":"MOVING_ACCEL_LIMIT"}
{"time_sec":1.060,"event":"state_transition","from":"MOVING_ACCEL_LIMIT","to":"MOVING_P_CONTROL"}
{"time_sec":8.900,"event":"state_transition","from":"MOVING_P_CONTROL","to":"WAIT_FOR_MEASUREMENT"}
{"time_sec":8.950,"event":"state_transition","from":"WAIT_FOR_MEASUREMENT","to":"CALCULATE"}
{"time_sec":9.000,"event":"measurement_received","actual_distance_m":0.985}
{"time_sec":9.002,"event":"result","status":"success","scale_factor":0.982054}
{"time_sec":9.003,"event":"state_transition","from":"CALCULATE","to":"DONE"}
```

## 사용 예시 2: 기존 CLI 측정 방식

사람이 직접 줄자, 바닥 마킹, 외부 ground truth로 실제 거리를 측정하는 경우에는
기존 CLI 방식을 그대로 사용할 수 있다.

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p output_dir:=logs/odom_x_plus \
  -p measurement_source:=cli
```

로봇이 정지하면 다음 prompt가 출력된다.

```text
Enter actual measured distance [m]:
```

실제 이동 거리를 meter 단위로 입력한다.

```text
Enter actual measured distance [m]: 0.985
```

CLI 방식도 계산 완료 후 `output_dir/result.json`과 `output_dir/events.jsonl`을
생성한다.

## 사용 예시 3: 방향별 calibration job 분리

omni drive 로봇은 `+x`, `-x`, `+y`, `-y`를 별도 job으로 실행하는 것을 권장한다.
각 방향은 독립적인 artifact directory를 사용한다.

```bash
BASE_DIR=/tmp/platform_inspector/job_002/artifacts

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x -p direction:=1 \
  -p output_dir:=$BASE_DIR/odom_x_plus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.985 \
  -p keep_alive_after_done:=false

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x -p direction:=-1 \
  -p output_dir:=$BASE_DIR/odom_x_minus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.990 \
  -p keep_alive_after_done:=false

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=y -p direction:=1 \
  -p output_dir:=$BASE_DIR/odom_y_plus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.976 \
  -p keep_alive_after_done:=false

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=y -p direction:=-1 \
  -p output_dir:=$BASE_DIR/odom_y_minus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.981 \
  -p keep_alive_after_done:=false
```

실제 운영에서는 각 방향 실행 전 robot pose와 주변 안전 공간을 다시 확인한다.

## 사용 예시 4: TurtleBot3 Gazebo smoke run

TurtleBot3 Gazebo는 differential drive 로봇이므로 x축 전진/후진 smoke test에
사용한다. y축 이동 검증에는 적합하지 않다.

Terminal 1:

```bash
export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source /opt/ros/$ROS_DISTRO/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Terminal 2:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source /path/to/odometry_calibrator_ws/install/setup.bash

ros2 topic info /cmd_vel -v
ros2 topic echo /odom --once
```

`/cmd_vel`이 `geometry_msgs/msg/TwistStamped`를 받을 수 있으면 calibration을
실행한다.

```bash
ARTIFACT_DIR=/tmp/tb3_odom_calib_gazebo

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.3 \
  -p max_velocity:=0.08 \
  -p max_acceleration:=0.04 \
  -p motion_timeout_sec:=20.0 \
  -p output_dir:=$ARTIFACT_DIR \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.3 \
  -p keep_alive_after_done:=false
```

성공 시 다음과 유사한 console 출력이 나온다.

```text
Axis      : x
Direction : +1
D_odom    : 0.297 m
D_actual  : 0.300 m
K_x(+1)   : 1.010505
```

artifact 확인:

```bash
python3 -m json.tool $ARTIFACT_DIR/result.json
cat $ARTIFACT_DIR/events.jsonl
```

## 사용 예시 5: motion_data_recorder 병행 실행

`motion_data_recorder`는 calibration 결과를 계산하지 않는다. 대신 `/cmd_vel`과
`/odom`을 고정 주기로 CSV에 기록한다. platform-inspector job에서 motion trace를
함께 보관하려면 recorder를 별도 process로 실행한다.

Terminal 1:

```bash
JOB_DIR=/tmp/platform_inspector/job_003

ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p output_dir:=$JOB_DIR/artifacts/motion \
  -p output_prefix:=odom_x_plus
```

Terminal 2:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p output_dir:=$JOB_DIR/artifacts/odom_x_plus \
  -p measurement_source:=parameter \
  -p actual_distance_m:=0.985 \
  -p keep_alive_after_done:=false
```

calibrator가 종료되면 recorder를 `Ctrl+C`로 종료한다.

CSV 확인:

```bash
ls $JOB_DIR/artifacts/motion/
head $JOB_DIR/artifacts/motion/odom_x_plus_*.csv
tail $JOB_DIR/artifacts/motion/odom_x_plus_*.csv
```

현재 `result.json.artifacts`에는 `events`만 포함된다. recorder CSV는 같은 job
artifact tree 안에 별도 artifact로 수집한다. 추후 필요하면
`result.json.artifacts.motion_csv` 필드를 추가할 수 있다.

## 상태 값

`result.json.status`는 다음 값 중 하나다.

```text
success
failed
timeout
invalid_parameter
cancelled
internal_error
```

일반적으로 `success`만 자동 성공으로 처리한다. `timeout`은 scale factor가 계산된
경우에도 robot motion이 목표 거리에 정상 도달하지 못했다는 의미이므로, 운영 UI에서
검토가 필요하다고 표시하는 것이 좋다.

## 안전 주의사항

managed calibration job 실행 전 다음을 확인한다.

- 선택한 축과 방향으로 이동할 충분한 공간이 있다.
- 주변 사람과 장애물이 없다.
- `/cmd_vel` topic type이 `geometry_msgs/msg/TwistStamped`와 호환된다.
- Twist 기반 robot 또는 simulation에는 relay/adapter가 준비되어 있다.
- `odom_topic`이 실제로 보정하려는 odometry source다.
- `target_distance`, `max_velocity`, `max_acceleration` 값이 현장 조건에 맞다.
- timeout, cancel, exception 상황에서도 node가 살아 있으면 stop command를 publish한다.

이 패키지는 scale factor 후보를 계산할 뿐, robot 설정을 자동 변경하지 않는다.
