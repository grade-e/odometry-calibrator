# Copyright 2026 Odometry Calibrator Maintainer
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare('odometry_calibrator'), 'config', 'odom_linear_calibration.yaml']
    )
    smoke_overrides = {'odom_topic': '/odometry_calibrator/mock_odom'}

    return LaunchDescription(
        [
            Node(
                package='odometry_calibrator',
                executable='test_mock_odom_publisher',
                name='test_mock_odom_publisher',
                output='screen',
                parameters=[params_file, smoke_overrides],
            ),
            TimerAction(
                period=1.0,
                actions=[
                    Node(
                        package='odometry_calibrator',
                        executable='odom_linear_calibrator',
                        name='odom_linear_calibrator',
                        output='screen',
                        emulate_tty=True,
                        parameters=[params_file, smoke_overrides],
                    ),
                ],
            ),
        ]
    )
