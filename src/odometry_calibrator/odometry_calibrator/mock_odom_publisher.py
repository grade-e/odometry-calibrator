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

import math

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class MockOdomPublisher(Node):
    def __init__(self):
        super().__init__('mock_odom_publisher')

        self.odom_topic = self.declare_parameter('odom_topic', '/odom').value
        self.publish_rate_hz = self.declare_parameter('publish_rate_hz', 20.0).value
        self.mock_speed = self.declare_parameter('mock_speed', 0.02).value
        self.x = self.declare_parameter('start_x', 0.0).value
        self.y = self.declare_parameter('start_y', 0.0).value

        self._validate_parameters()

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.last_publish_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_odom)

        self.get_logger().info(
            "Publishing mock odometry on '%s' at %.3f Hz"
            % (self.odom_topic, self.publish_rate_hz)
        )

    def _validate_parameters(self):
        if self.publish_rate_hz <= 0.0 or not math.isfinite(self.publish_rate_hz):
            raise RuntimeError('publish_rate_hz must be positive and finite')
        if not math.isfinite(self.mock_speed):
            raise RuntimeError('mock_speed must be finite')
        if not math.isfinite(self.x) or not math.isfinite(self.y):
            raise RuntimeError('start_x and start_y must be finite')

    def _publish_odom(self):
        current_time = self.get_clock().now()
        dt = max((current_time - self.last_publish_time).nanoseconds * 1.0e-9, 0.0)
        self.last_publish_time = current_time

        self.x += self.mock_speed * dt

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.twist.twist.linear.x = self.mock_speed

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = MockOdomPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
