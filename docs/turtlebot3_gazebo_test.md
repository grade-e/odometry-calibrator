# TurtleBot3 Gazebo End-to-End 검증

이 문서는 TurtleBot3 Gazebo simulation을 이용해 `odometry_calibrator`가 실제 ROS 2 `/cmd_vel`과 `/odom` 인터페이스에서 end-to-end로 동작하는지 확인하는 절차를 정리한다.

이 검증의 목적은 wheel odometry scale 정확도를 평가하는 것이 아니다. 오픈소스 simulation 환경에서 다음 흐름이 실제로 연결되는지 확인하는 smoke/e2e test다.

- `odom_linear_calibrator`가 `/cmd_vel` `geometry_msgs/msg/TwistStamped`를 publish한다.
- TurtleBot3가 Gazebo에서 이동한다.
- `/odom` pose가 변화한다.
- calibrator가 `/odom` 기준 이동 거리를 계산한다.
- target distance 근처에서 `WAIT_FOR_MEASUREMENT`로 전이한다.
- `motion_data_recorder`가 `/cmd_vel` TwistStamped, `/odom`을 CSV로 기록한다.

TurtleBot3는 differential drive 로봇이므로 이 문서는 x축 전진/후진 검증만 다룬다.

## 전제 조건

ROS 2 환경은 현재 개발 환경의 배포판을 사용한다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash
echo $ROS_DISTRO
```

TurtleBot3 simulation package는 ROS 배포판과 branch가 맞아야 한다. 예를 들어 Jazzy 환경에서는 TurtleBot3 simulation의 Jazzy 지원 branch를 사용해야 한다.

## TurtleBot3 Simulation 설치

별도 workspace에 TurtleBot3 simulation을 설치한다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws/src

git clone -b $ROS_DISTRO https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
```

`$ROS_DISTRO`와 같은 이름의 branch가 없을 수 있다. 이 경우 공식 TurtleBot3 문서를 확인하고 현재 ROS 2 배포판에서 지원하는 branch를 선택한다.

## TurtleBot3 Gazebo 실행

1차 검증은 가장 단순한 empty world에서 수행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

다른 world는 2차 검증 용도로 사용한다.

```bash
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```

처음 검증은 `empty_world.launch.py`에서 수행한다. `world` 또는 `house` 환경은 장애물, 맵 다운로드, 초기 로딩 시간이 있을 수 있다.

## Topic 확인

TurtleBot3 Gazebo 실행 후 topic을 확인한다.

```bash
ros2 topic list
ros2 topic info /cmd_vel
ros2 topic info /odom
ros2 topic echo /odom --once
```

확인 기준:

- `/cmd_vel` topic이 존재한다.
- `/cmd_vel` message type이 `geometry_msgs/msg/TwistStamped`다.
- `/odom` topic이 존재한다.
- `/odom` message type이 `nav_msgs/msg/Odometry`다.
- `/odom` pose 값이 정상적으로 echo된다.

topic 이름이 다르면 이후 명령에서 `cmd_vel_topic`, `odom_topic` parameter를 실제 topic 이름으로 override한다.

`odom_linear_calibrator`와 `motion_data_recorder`는 `/cmd_vel`을 `geometry_msgs/msg/TwistStamped`로 사용한다. TurtleBot3 Gazebo의 Jazzy bridge도 `/cmd_vel` `TwistStamped` subscriber를 제공하므로 별도 relay 없이 직접 연결된다.

## odometry_calibrator Build/Test

`odometry-calibrator` workspace 루트에서 실행한다.

```bash
export ROS_DISTRO=${ROS_DISTRO:-humble}
source /opt/ros/$ROS_DISTRO/setup.bash

colcon build --packages-select odometry_calibrator
source install/setup.bash

colcon test --packages-select odometry_calibrator
colcon test-result --verbose
```

entry point 확인:

```bash
ros2 pkg executables odometry_calibrator
```

기대 실행 파일:

```text
odometry_calibrator odom_linear_calibrator
odometry_calibrator motion_data_recorder
odometry_calibrator test_mock_odom_publisher
```

시뮬레이터가 꺼져 있으면 아래 명령은 `/odom` 대기 상태로 들어가도 정상이다.

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args -p target_distance:=0.1
```

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args -p target_distance:=0.1
```

## 1차 E2E: TurtleBot3와 odom_linear_calibrator

Terminal 1에서 TurtleBot3 Gazebo를 실행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Terminal 2에서 calibrator를 실행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source <odometry_calibrator_ws>/install/setup.bash

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p max_velocity:=0.08 \
  -p max_acceleration:=0.04 \
  -p motion_timeout_sec:=30.0
```

예상 동작:

1. calibrator가 첫 `/odom`을 latch한다.
2. `/cmd_vel.twist.linear.x`를 publish한다.
3. TurtleBot3가 Gazebo에서 전진한다.
4. `/odom` 기준 `axis_distance`가 증가한다.
5. target distance 근처에서 `WAIT_FOR_MEASUREMENT`로 전이한다.
6. `/cmd_vel` zero가 지속 publish된다.
7. 터미널에 실제 거리 입력 prompt가 뜬다.

Gazebo에서 실제 거리를 정확히 측정하기 어렵다면 1차 검증에서는 `/odom` 기준 거리와 유사한 값을 입력해 흐름만 확인한다.

```text
Enter actual measured distance [m]: 1.000
```

수락 기준:

- TurtleBot3가 Gazebo에서 실제로 움직인다.
- calibrator가 `WAIT_FOR_MEASUREMENT`에 진입한다.
- zero `/cmd_vel` 유지가 확인된다.
- 입력 후 `K_x(+1)` 결과가 출력된다.
- node crash가 없다.

## 2차 E2E: recorder 동시 실행

Terminal 1에서 TurtleBot3 Gazebo를 실행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

Terminal 2에서 recorder를 실행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source <odometry_calibrator_ws>/install/setup.bash

ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p output_prefix:=tb3_x_pos
```

Terminal 3에서 calibrator를 실행한다.

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source <odometry_calibrator_ws>/install/setup.bash

ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=1 \
  -p target_distance:=1.0 \
  -p max_velocity:=0.08 \
  -p max_acceleration:=0.04 \
  -p motion_timeout_sec:=30.0
```

calibrator가 prompt를 띄우면 다음처럼 입력한다.

```text
Enter actual measured distance [m]: 1.000
```

이후 recorder를 `Ctrl+C`로 종료한다.

CSV를 확인한다.

```bash
ls logs/
head logs/tb3_x_pos_*.csv
tail logs/tb3_x_pos_*.csv
```

수락 기준:

- `logs/tb3_x_pos_<timestamp>.csv` 파일이 생성된다.
- `time_sec` 값이 증가한다.
- 주행 중 `cmd_vx`가 0이 아닌 값으로 기록된다.
- `odom_x` 또는 `odom_distance`가 증가한다.
- `axis_distance`가 1.0 m 근처까지 증가한다.
- `remaining_distance`가 0 근처로 감소한다.
- 종료 시 Motion Data Summary가 출력된다.

## -x 방향 후진 테스트

TurtleBot3는 differential drive이므로 후진 motion도 확인할 수 있다.

Terminal 2에서 recorder를 실행한다.

```bash
ros2 run odometry_calibrator motion_data_recorder --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=-1 \
  -p target_distance:=0.5 \
  -p output_prefix:=tb3_x_neg
```

Terminal 3에서 calibrator를 실행한다.

```bash
ros2 run odometry_calibrator odom_linear_calibrator --ros-args \
  -p odom_topic:=/odom \
  -p cmd_vel_topic:=/cmd_vel \
  -p axis:=x \
  -p direction:=-1 \
  -p target_distance:=0.5 \
  -p max_velocity:=0.05 \
  -p max_acceleration:=0.03 \
  -p motion_timeout_sec:=20.0
```

수락 기준:

- TurtleBot3가 후진한다.
- odom x 방향 변화가 음수 또는 후진 방향으로 나타난다.
- `axis_distance`는 양수로 증가한다.
- summary `Direction`이 `-1`로 출력된다.

## y축 테스트 제외

TurtleBot3는 일반적인 differential drive 로봇이며 lateral motion을 지원하지 않는다.

TurtleBot3는 x축 전진/후진 end-to-end check에는 적합하지만, y축 lateral calibration 검증에는 적합하지 않다. y축 테스트는 omni-drive robot model 또는 실제 대상 AMR platform에서 수행한다.

TurtleBot3 Gazebo 검증 범위에서는 `axis:=y` 테스트를 제외한다.

## 문제 해결

### `/odom`이 보이지 않는 경우

```bash
ros2 topic list | grep odom
ros2 topic info /odom
```

확인 항목:

- TurtleBot3 Gazebo가 정상 실행 중인지
- `TURTLEBOT3_MODEL` 환경 변수가 설정되었는지
- `~/turtlebot3_ws/install/setup.bash`를 source 했는지
- TurtleBot3 simulation branch가 현재 `ROS_DISTRO`와 맞는지

### `/cmd_vel`을 publish해도 움직이지 않는 경우

```bash
ros2 topic echo /cmd_vel
```

확인 항목:

- `cmd_vel_topic` 이름이 실제 TurtleBot3 topic과 같은지
- Gazebo가 pause 상태인지
- 로봇이 world에 spawn되었는지
- `/cmd_vel` message가 실제로 publish되고 있는지

### calibrator가 바로 timeout 되는 경우

대응:

- `motion_timeout_sec`를 늘린다.
- `max_velocity`를 조금 높인다.
- `target_distance`를 0.5 m로 낮춰 smoke test한다.
- `/odom` 값이 실제로 증가하는지 확인한다.

### recorder CSV가 생성되지 않는 경우

확인 항목:

- recorder가 첫 `/odom`을 수신했는지
- `output_dir`에 write 권한이 있는지
- `use_reference_pose=true`인데 reference topic이 없는 상태는 아닌지
- `odom_topic` parameter가 TurtleBot3의 실제 odom topic과 일치하는지

## Validation Log

| Date | ROS_DISTRO | TurtleBot3 Model | World | Test | Result | Notes |
| --- | --- | --- | --- | --- | --- | --- |
| TBD | TBD | burger | empty_world | +x 1.0m calibrator | TBD | TBD |
| TBD | TBD | burger | empty_world | +x 1.0m with recorder | TBD | TBD |
| TBD | TBD | burger | empty_world | -x 0.5m with recorder | TBD | TBD |
