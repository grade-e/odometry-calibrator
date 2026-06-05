# odometry_calibrator 문서 안내

이 문서는 `odometry_calibrator` repository의 문서 허브다. 자세한 내용은 주제별 문서로 분리되어 있다.

## 빠른 시작

워크스페이스 루트에서 실행한다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --packages-select odometry_calibrator
source install/setup.bash
```

사용자-facing 실행 명령은 다음 세 가지다.

```bash
ros2 run odometry_calibrator odom_linear_calibrator
ros2 run odometry_calibrator motion_data_recorder
ros2 run odometry_calibrator test_mock_odom_publisher
```

## 문서 구성

| 문서 | 목적 |
| --- | --- |
| `architecture.md` | 패키지 목적, 전체 구성, 노드 역할, 내부 module 구조 |
| `calibration.md` | mock smoke test, 실제 AMR calibration 절차, axis/direction, 결과 해석 |
| `data_recording.md` | motion_data_recorder 사용법, CSV 컬럼, reference pose, rosbag, PlotJuggler |
| `troubleshooting.md` | rqt_graph 확인, 파라미터 요약, 안전 체크리스트, 문제 해결 |

## 권장 읽기 순서

1. 처음 구조를 볼 때: `architecture.md`
2. 보정 절차를 실행할 때: `calibration.md`
3. 주행 데이터를 남길 때: `data_recording.md`
4. topic 연결이나 CSV 값을 확인할 때: `troubleshooting.md`

## 역할 요약

- `odom_linear_calibrator`: 목표 거리 주행, 실측 거리 입력, scale factor 계산
- `motion_data_recorder`: `/cmd_vel`, odometry, optional reference pose를 CSV로 기록
- `test_mock_odom_publisher`: smoke test support용 fixed-speed odometry source

내부 구현은 `calibration`, `recording`, `common`, `test_support` module로 나뉘지만 ROS 2 package는 하나의 `odometry_calibrator`로 유지한다.
