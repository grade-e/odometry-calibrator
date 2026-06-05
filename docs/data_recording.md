# Motion Data 기록

전체 패키지 구조는 `architecture.md`, calibration 절차는 `calibration.md`, 문제 해결은 `troubleshooting.md`를 참고한다.

## 목적

`motion_data_recorder`는 command velocity, odometry, optional reference pose data를 시간 기준으로 CSV에 기록한다. 실제 AMR 저속 점검, calibrator 실행 중 기록, mock smoke test, rosbag replay 분석, 향후 외부 기준 pose 비교에 사용할 수 있다.

recorder는 CSV data를 저장하고 summary를 출력하는 역할만 한다. topic 연결 확인은 `rqt_graph`를 사용하고, 데이터 시각화는 PlotJuggler, spreadsheet, notebook, 별도 optional script 같은 외부 도구에서 수행한다.

구현 파일은 내부 `recording` module 아래에 있지만, ROS 2 package는 여전히 `odometry_calibrator` 하나로 배포된다. 실행 명령도 `ros2 run odometry_calibrator motion_data_recorder`를 유지한다.

## 역할 분리

`odom_linear_calibrator`는 로봇을 움직이고, 작업자가 입력한 실제 측정 거리를 받아 odometry scale factor를 계산한다.

`motion_data_recorder`는 로봇을 움직이지 않고 최종 calibration constant도 계산하지 않는다. `/cmd_vel` TwistStamped, configured odom topic, optional `PoseStamped` reference pose topic을 구독하고, 나중에 분석할 raw motion data를 CSV로 저장한다.

## 기본 실행

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
ros2 launch odometry_calibrator data_recording.launch.py
```

기본 CSV 저장 위치:

```text
logs/motion_<timestamp>.csv
```

`test_mock_odom_publisher`만 실행 중인 경우 command velocity 컬럼은 0으로 유지될 수 있다. 이 경우 `/cmd_vel`을 publish하는 노드가 없기 때문이다. `test_mock_odom_publisher`는 smoke test support용 fixed-speed odometry source이며 robot simulator나 closed-loop robot model이 아니다. 따라서 mock node를 멈출 때까지 odometry는 계속 증가한다.

parameter를 override하려면:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5 \
  -p output_prefix:=calibration_x_pos
```

## Calibration 중 기록

터미널 1에서 recorder를 실행한다.

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

터미널 2에서 calibrator를 실행한다.

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=0.5
```

recorder는 첫 odometry sample을 latch한 뒤에만 CSV row를 쓰기 시작한다. reference pose recording이 켜져 있으면 첫 유효한 reference pose까지 수신한 뒤 recording을 시작한다.

`time_sec`는 CSV recording이 실제 시작된 시점부터의 경과 시간이다. 거리 기준점은 시간 기준점과 분리되어 있다. `odom_distance`는 첫 latched odometry pose 기준으로 계산되고, `ref_distance`는 첫 latched reference pose 기준으로 계산된다.

mock 기반 calibration smoke test에서는 calibrator가 `DONE`에 도달한 직후 recorder를 종료하는 것이 좋다. fixed-speed mock publisher를 계속 실행하면 `/cmd_vel`이 zero로 돌아간 뒤에도 odometry와 summary distance는 계속 증가한다.

## CSV 컬럼

기본 컬럼:

```csv
time_sec,cmd_vx,cmd_vy,cmd_wz,odom_x,odom_y,odom_yaw,odom_distance,axis_distance,remaining_distance
```

- `time_sec`: CSV recording 시작 이후 경과 시간
- `cmd_vx`, `cmd_vy`, `cmd_wz`: 최신 `/cmd_vel.twist` 값
- `odom_x`, `odom_y`, `odom_yaw`: 최신 odometry pose
- `odom_distance`: odometry start pose 기준 2D 이동 거리
- `axis_distance`: `axis`와 `direction` 기준 양수 이동 거리
- `remaining_distance`: `target_distance - axis_distance`

`use_reference_pose=true`이면 다음 컬럼이 추가된다.

```csv
ref_x,ref_y,ref_yaw,ref_distance,ref_axis_distance,odom_ref_error,error_rate,scale_estimate
```

- `ref_distance`: reference start pose 기준 2D 이동 거리
- `ref_axis_distance`: `axis`와 `direction` 기준 reference 이동 거리
- `odom_ref_error`: `axis_distance - ref_axis_distance`
- `error_rate`: `odom_ref_error / ref_axis_distance`
- `scale_estimate`: `ref_axis_distance / axis_distance`

ratio 계산에서 분모가 너무 작으면 해당 field는 빈 값으로 기록된다.

## Reference Pose

현재 구현에서 지원하는 reference pose message type은 다음 하나다.

```text
geometry_msgs/msg/PoseStamped
```

실행 예시:

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p use_reference_pose:=true \
  -p reference_pose_topic:=/reference_pose
```

marker tracking, simulator ground truth, motion capture, external localization system 등 `PoseStamped`를 publish하는 외부 기준 pose source와 함께 사용할 수 있다.

## Rosbag

원본 ROS topic을 replay 또는 비교용으로 기록하려면:

```bash
ros2 bag record /cmd_vel /odom
```

reference pose까지 포함하려면:

```bash
ros2 bag record /cmd_vel /odom /reference_pose
```

rosbag replay 중 `motion_data_recorder`를 실행하면 replay된 topic에서 CSV를 다시 만들 수 있다.

## rqt_graph로 연결 확인

`rqt_graph`는 데이터 값을 보는 도구가 아니라 ROS graph의 node/topic 연결 관계를 확인하는 도구다. 다음을 확인할 때 사용한다.

- `odom_linear_calibrator`가 configured `/cmd_vel` TwistStamped를 publish하는지
- `odom_linear_calibrator`가 configured odom topic을 subscribe하는지
- `motion_data_recorder`가 `/cmd_vel`과 configured odom topic을 subscribe하는지
- `test_mock_odom_publisher`가 mock odom topic을 publish하는지
- topic 이름 mismatch가 있는지

실행 예:

```bash
rqt_graph
```

mock launch 기준으로 기대되는 연결:

```text
test_mock_odom_publisher
  -> /odometry_calibrator/mock_odom
      -> odom_linear_calibrator

odom_linear_calibrator
  -> /cmd_vel
```

recorder까지 함께 실행하면 기대 구조는 다음과 같다.

```text
test_mock_odom_publisher
  -> /odometry_calibrator/mock_odom
      -> odom_linear_calibrator
      -> motion_data_recorder

odom_linear_calibrator
  -> /cmd_vel
      -> motion_data_recorder
```

`rqt_graph`에서 연결이 보이지 않으면 먼저 topic 이름과 parameter override를 확인한다. 실제 수치 값은 `rqt_graph`가 아니라 `ros2 topic echo`, PlotJuggler, 또는 recorder CSV로 확인한다.

## PlotJuggler

PlotJuggler는 ROS topic 또는 recorder CSV를 시계열로 확인할 때 사용한다.

ROS topic을 직접 보려면 PlotJuggler를 실행한 뒤 ROS 2 data streamer를 사용해 topic을 subscribe한다.

```bash
plotjuggler
```

ROS topic에서 먼저 보기 좋은 field:

```text
/cmd_vel.twist.linear.x
/cmd_vel.twist.linear.y
/odom.pose.pose.position.x
/odom.pose.pose.position.y
```

recorder CSV를 직접 load하는 경우에는 다음 조합을 우선 확인한다.

```text
time_sec vs axis_distance
time_sec vs remaining_distance
time_sec vs cmd_vx
time_sec vs cmd_vy
time_sec vs odom_distance
time_sec vs ref_axis_distance
time_sec vs odom_ref_error
time_sec vs scale_estimate
```

일반 calibration에서는 `axis_distance`가 증가하고 `remaining_distance`가 0 근처로 감소하는지 확인한다. reference pose를 함께 기록한 경우 `axis_distance`와 `ref_axis_distance`의 차이, `scale_estimate`의 안정성을 확인한다.

## Spreadsheet 확인

생성된 CSV는 Excel, LibreOffice, Google Sheets 같은 spreadsheet 도구에서 열 수 있다. 빠른 sanity check에는 다음 컬럼이 유용하다.

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
