# Odometry Calibrator

직선 주행 기반 wheel odometry scale 보정을 위한 ROS 2 Python 패키지다.

omni drive 로봇은 +x, -x, +y, -y 방향을 독립적으로 확인하고 보정할 수 있다.

## 개념

`odom_linear_calibrator` 노드는 선택한 축과 방향으로 로봇을 이동시킨다. 이후 configured odom topic에서 odometry 이동 거리를 읽고, 작업자가 입력한 실제 측정 거리와 비교해 scale factor를 계산한다.

```text
K_x = D_actual / D_odom_x
K_y = D_actual / D_odom_y
```

`/cmd_vel`은 로봇 이동 명령에만 사용된다. configured odom topic은 odometry가 보고한 이동 거리 계산에 사용된다. `D_actual`은 줄자, 바닥 마킹, marker tracking, 외부 ground truth 등 별도 측정 수단으로 얻어야 한다.

측정값 입력은 `MeasurementProvider` 인터페이스 뒤에 있다. 현재 구현은 `CliMeasurementProvider`를 사용하며, 나중에 marker 기반 provider를 추가해도 calibration node와 강하게 결합되지 않도록 구성되어 있다.

configured odom topic subscription은 ROS 2 sensor-data QoS를 사용한다. 따라서 로봇에서 흔히 쓰는 best-effort odometry publisher와 연결할 수 있다. `/cmd_vel` publisher는 기본 reliable QoS를 유지한다.

## 빌드

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
```

## 실행 파일

```bash
ros2 run odometry_calibrator odom_linear_calibrator
ros2 run odometry_calibrator test_mock_odom_publisher
ros2 run odometry_calibrator motion_data_recorder
```

`test_mock_odom_publisher`는 smoke test support용 실행 파일이다. 실제 runtime 기능 노드나 robot simulator로 취급하지 않는다.

내부 구현은 `calibration`, `recording`, `common`, `test_support` module로 분리되어 있지만, ROS 2 package는 현재 `odometry_calibrator` 하나로 유지한다. 사용자 실행 명령은 위 command name을 기준으로 사용한다.

## 문서

- 문서 안내: `docs/usage_and_architecture.md`
- 구성과 내부 구조: `docs/architecture.md`
- odometry calibration 절차: `docs/calibration.md`
- motion data recording 상세: `docs/data_recording.md`
- 진단과 문제 해결: `docs/troubleshooting.md`

## 방향별 실행 예시

+x 방향:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=x -p direction:=1
```

-x 방향:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=x -p direction:=-1
```

+y 방향:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=1
```

-y 방향:

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=-1
```

command velocity mapping은 다음과 같다.

```text
axis: x, direction:  1  -> /cmd_vel.linear.x = +v_cmd
axis: x, direction: -1  -> /cmd_vel.linear.x = -v_cmd
axis: y, direction:  1  -> /cmd_vel.linear.y = +v_cmd
axis: y, direction: -1  -> /cmd_vel.linear.y = -v_cmd
```

ROS 2 parameter parsing에서는 quote 없는 `y`가 YAML boolean `true`로 해석될 수 있다. 이 노드는 해당 값을 y축으로 매핑하지만, YAML 파일에서는 `axis: "y"`처럼 quote를 사용하는 것을 권장한다.

## 측정값 입력

로봇이 정지하면 노드는 다음 프롬프트를 출력한다.

```text
Enter actual measured distance [m]:
```

실제 주행 거리를 meter 단위로 입력한다.

```text
Enter actual measured distance [m]: 0.985
```

출력 예시:

```text
Axis      : x
Direction : +1
D_odom    : 0.500 m
D_actual  : 0.492 m
K_x(+1)   : 0.984000
```

다른 예시:

```text
Axis      : y
Direction : -1
D_odom    : 0.500 m
D_actual  : 0.486 m
K_y(-1)   : 0.972000
```

## 파라미터

기본값은 `src/odometry_calibrator/config/odom_linear_calibration.yaml`에 있다.

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

smoke test support용 mock odometry 기본값:

```yaml
mock_axis: "x"
direction: 1
publish_rate_hz: 20.0
mock_speed: 0.05
```

## Mock Smoke Test

패키지는 smoke test support용 `test_mock_odom_publisher`를 포함한다. 이 실행 파일은 fixed-speed mock odometry를 publish한다. `/cmd_vel`을 subscribe하지 않으며 실제 robot simulator가 아니다.

```bash
ros2 launch odometry_calibrator test_calibration.launch.py
```

`test_calibration.launch.py`는 내부적으로 `test_mock_odom_publisher`를 실행한다.

mock launch는 실제 로봇의 `/odom` publisher와 섞이지 않도록 `odom_topic`을 `/odometry_calibrator/mock_odom`으로 override한다.

y축 mock test를 수동으로 실행하려면:

```bash
ros2 run odometry_calibrator test_mock_odom_publisher --ros-args -p mock_axis:=y -p direction:=1
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p axis:=y -p direction:=1
```

## Motion Data 기록

`motion_data_recorder`는 motion data를 기록하는 독립 노드다. 로봇을 움직이지 않고, 최종 calibration scale factor도 계산하지 않는다. 역할 분리는 다음과 같다.

```text
odom_linear_calibrator
- calibration 주행 명령
- 실제 측정 거리 입력 대기
- K_x 또는 K_y 계산

motion_data_recorder
- /cmd_vel과 configured odom topic 구독
- optional PoseStamped reference pose topic 구독
- 고정 주기로 CSV row 저장
- shutdown summary 출력
```

기본 config로 실행:

```bash
ros2 launch odometry_calibrator data_recording.launch.py
```

직접 실행:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

CSV 파일은 기본적으로 `logs/` 아래에 저장된다.

```text
logs/motion_20260605_203015.csv
```

`test_mock_odom_publisher`만 기록하는 경우 `/cmd_vel` publisher가 없으므로 `cmd_vx`, `cmd_vy`, `cmd_wz`는 0으로 유지될 수 있다. test mock odometry publisher는 fixed speed로 odometry를 publish하며 `/cmd_vel`에 따라 정지하지 않는다. summary의 최종 거리를 calibration target 근처로 맞추려면 scenario endpoint 근처에서 recorder를 종료한다.

기본 CSV 컬럼:

```csv
time_sec,cmd_vx,cmd_vy,cmd_wz,odom_x,odom_y,odom_yaw,odom_distance,axis_distance,remaining_distance
```

`time_sec`는 CSV recording이 실제 시작된 뒤의 경과 시간이다. `odom_distance`는 첫 latched odometry pose 기준으로 계산된다.

`geometry_msgs/msg/PoseStamped` topic을 이용해 reference pose recording을 켤 수 있다.

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p use_reference_pose:=true \
  -p reference_pose_topic:=/reference_pose
```

reference pose recording이 켜져 있으면 CSV에 다음 컬럼이 추가된다.

```csv
ref_x,ref_y,ref_yaw,ref_distance,ref_axis_distance,odom_ref_error,error_rate,scale_estimate
```

`ref_distance`는 첫 latched reference pose 기준으로 계산된다.

summary 예시:

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

recorder는 CSV data를 저장하고 summary를 출력하는 역할만 한다. topic 연결 확인은 `rqt_graph`를 사용하고, 데이터 시각화는 PlotJuggler, spreadsheet, notebook, 별도 optional script 같은 외부 도구에서 수행한다.

자세한 내용은 `docs/data_recording.md`를 참고한다.

## 동작 흐름

1. `odom_linear_calibrator`는 configured odom topic에서 첫 유효한 odometry sample을 기다리고, 이를 start pose로 latch한다.
2. 노드는 `axis`와 `direction`에 따라 signed `/cmd_vel.linear.x` 또는 `/cmd_vel.linear.y`를 publish하며 acceleration limit을 적용한다.
3. target distance 근처에 도달하거나, timeout이 발생하거나, command velocity가 minimum 아래로 내려가면 `WAIT_FOR_MEASUREMENT`로 전이한다.
4. 측정값 입력 대기 중에는 최소 10 Hz로 zero velocity를 계속 publish한다.
5. meter 단위 실제 측정 거리를 입력한다.

```text
Enter actual measured distance [m]: 0.985
```

6. 노드는 `Axis`, `Direction`, `D_odom`, `D_actual`, `K_x(+/-1)` 또는 `K_y(+/-1)`를 출력한다.

## 참고 사항

- omni drive 로봇은 x축과 y축을 별도로 calibration한다.
- 충분한 공간이 있는 평평한 바닥에서 수행한다.
- `D_actual` 측정 시 로봇 기준점을 일관되게 유지한다.
- slip과 overshoot를 줄이기 위해 낮은 속도로 시작한다.

## 실제 로봇 테스트 전 안전 체크리스트

- 실제 AMR 연결 전에 mock launch로 흐름을 먼저 검증한다.
- `/cmd_vel`이 실제 로봇 제어 topic과 일치하는지 확인한다.
- `/odom`이 실제 wheel odometry topic과 일치하는지 확인한다.
- `axis`와 `direction` 조합이 의도한 주행 방향과 일치하는지 확인한다.
- emergency stop 또는 수동 정지 수단을 준비한다.
- `target_distance`보다 충분히 긴 직선 주행 공간을 확보한다.
- 처음에는 보수적인 `max_velocity`, `max_acceleration` 값으로 시작한다.
- 처음에는 바퀴를 띄운 상태 또는 매우 낮은 속도에서 command 방향만 확인한다.
- `WAIT_FOR_MEASUREMENT` 상태에서 `/cmd_vel`이 zero로 지속 publish되는지 확인한다.
