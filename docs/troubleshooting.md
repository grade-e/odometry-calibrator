# 진단과 문제 해결

이 문서는 ROS graph 확인, PlotJuggler/Spreadsheet 분석, 파라미터 요약, 자주 발생하는 문제를 정리한다.

## rqt_graph로 연결 확인

`rqt_graph`는 데이터 값을 확인하는 도구가 아니라 node/topic 연결 관계를 확인하는 도구다. calibration 또는 recording이 정상 동작하지 않을 때 먼저 topic 연결을 확인하는 용도로 사용한다.

```bash
rqt_graph
```

mock launch 기준 기대 연결:

```text
test_mock_odom_publisher
  -> /odometry_calibrator/mock_odom
      -> odom_linear_calibrator

odom_linear_calibrator
  -> /cmd_vel
```

recorder까지 함께 실행한 경우:

```text
test_mock_odom_publisher
  -> /odometry_calibrator/mock_odom
      -> odom_linear_calibrator
      -> motion_data_recorder

odom_linear_calibrator
  -> /cmd_vel
      -> motion_data_recorder
```

`rqt_graph`로 확인할 수 있는 것은 연결 관계다. `odom_x`, `cmd_vx`, `axis_distance` 같은 실제 값은 `ros2 topic echo`, PlotJuggler, 또는 recorder CSV로 확인한다.

## PlotJuggler와 Spreadsheet 분석

PlotJuggler는 ROS topic 또는 recorder CSV를 시계열로 확인할 때 사용한다.

실행:

```bash
plotjuggler
```

ROS topic을 직접 볼 때 먼저 확인할 field:

```text
/cmd_vel.twist.linear.x
/cmd_vel.twist.linear.y
/odom.pose.pose.position.x
/odom.pose.pose.position.y
```

CSV에서 먼저 확인할 컬럼:

```text
time_sec
cmd_vx
cmd_vy
odom_x
odom_y
odom_distance
axis_distance
remaining_distance
odom_ref_error
scale_estimate
```

추천 plot 조합:

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

일반 calibration에서는 `axis_distance`가 증가하고 `remaining_distance`가 0 근처로 감소하는지 확인한다. reference pose가 있으면 `axis_distance`와 `ref_axis_distance`의 차이, `scale_estimate`가 안정적인지 확인한다.

Excel, LibreOffice, Google Sheets에서는 `time_sec`를 x축으로 두고 `axis_distance`, `remaining_distance`, `cmd_vx`, `cmd_vy`를 함께 보면 주행 command와 odometry 변화가 일관적인지 빠르게 확인할 수 있다.

## 파라미터 요약

### odom_linear_calibrator

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `odom_topic` | `/odom` | odometry 입력 topic |
| `cmd_vel_topic` | `/cmd_vel` | command velocity 출력 topic |
| `cmd_vel_frame_id` | `base_link` | TwistStamped header frame_id |
| `axis` | `"x"` | `x` 또는 `y` |
| `direction` | `1` | `1` 또는 `-1` |
| `target_distance` | `0.5` | 목표 이동 거리 |
| `distance_tolerance` | `0.005` | 목표 거리 도달 tolerance |
| `control_mode` | `p_min_clamped` | command velocity 계산 방식 |
| `max_velocity` | `0.1` | 최대 command velocity |
| `min_velocity` | `0.05` | `p_min_clamped` 이동 중 command velocity 하한 |
| `max_acceleration` | `0.05` | acceleration limit |
| `motion_timeout_sec` | `20.0` | 이동 timeout |

### motion_data_recorder

| 파라미터 | 기본값 | 설명 |
| --- | ---: | --- |
| `odom_topic` | `/odom` | odometry 입력 topic |
| `cmd_vel_topic` | `/cmd_vel` | command velocity 입력 topic |
| `use_reference_pose` | `false` | reference pose 구독 여부 |
| `reference_pose_topic` | `""` | PoseStamped reference topic |
| `axis` | `"x"` | `x` 또는 `y` |
| `direction` | `1` | `1` 또는 `-1` |
| `target_distance` | `1.0` | remaining distance 계산 기준 |
| `output_dir` | `logs` | CSV 저장 directory |
| `output_prefix` | `motion` | CSV filename prefix |
| `record_rate_hz` | `20.0` | CSV 기록 주기 |
| `summary_on_shutdown` | `true` | 종료 시 summary 출력 여부 |

## 실제 로봇 테스트 체크리스트

- mock launch로 먼저 흐름을 검증한다.
- 실제 robot의 command topic이 `cmd_vel_topic`과 일치하는지 확인한다.
- 실제 wheel odometry topic이 `odom_topic`과 일치하는지 확인한다.
- `axis`와 `direction` 조합이 의도한 주행 방향과 일치하는지 확인한다.
- emergency stop 또는 수동 정지 수단을 준비한다.
- `target_distance`보다 긴 직선 공간을 확보한다.
- 처음에는 낮은 `max_velocity`, 낮은 `max_acceleration`으로 실행한다.
- 바퀴를 띄운 상태 또는 매우 낮은 속도로 command 방향만 먼저 확인한다.
- `motion_data_recorder`를 함께 실행해 CSV를 남긴다.
- calibration 결과와 CSV summary의 축/방향이 같은지 확인한다.

## 문제 해결

### `axis: y`가 이상하게 들어오는 경우

YAML에서는 다음처럼 quote를 사용한다.

```yaml
axis: "y"
```

### recorder가 CSV를 만들지 않는 경우

다음을 확인한다.

- odom topic이 실제 publish되고 있는가
- `odom_topic` 파라미터가 실제 topic 이름과 같은가
- `use_reference_pose=true`인데 reference pose가 publish되지 않는 상태는 아닌가
- `output_dir`에 write 권한이 있는가

### reference 관련 컬럼이 비어 있는 경우

`ref_axis_distance` 또는 `axis_distance`가 너무 작으면 ratio 계산 분모가 작기 때문에 `error_rate`, `scale_estimate`가 빈 값이 될 수 있다.

### ROS 2 topic 연결 확인

```bash
ros2 topic list
ros2 topic info /odom
ros2 topic info /cmd_vel
ros2 topic echo /odom --once
```

custom topic을 쓰는 경우:

```bash
ros2 topic info /my_robot/wheel_odom
ros2 topic info /my_robot/cmd_vel
```

## 관련 문서

- 전체 구조: `architecture.md`
- 보정 절차: `calibration.md`
- Motion data recording: `data_recording.md`
