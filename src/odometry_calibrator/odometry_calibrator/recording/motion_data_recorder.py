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

import csv
from dataclasses import dataclass
from datetime import datetime
import math
from pathlib import Path
from threading import Lock

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from odometry_calibrator.common.axis import normalize_axis
from odometry_calibrator.common.axis import normalize_direction
from odometry_calibrator.common.axis import VALID_AXES
from odometry_calibrator.common.motion_metrics import compute_axis_distance
from odometry_calibrator.common.motion_metrics import compute_remaining_distance
from odometry_calibrator.common.motion_metrics import safe_ratio
from odometry_calibrator.common.parameters import normalize_reference_pose_topic
from odometry_calibrator.common.parameters import validate_positive_finite
from odometry_calibrator.common.pose_utils import distance_2d
from odometry_calibrator.common.pose_utils import yaw_from_quaternion
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.exceptions import ParameterUninitializedException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

BASE_FIELDS = [
    'time_sec',
    'cmd_vx',
    'cmd_vy',
    'cmd_wz',
    'odom_x',
    'odom_y',
    'odom_yaw',
    'odom_distance',
    'axis_distance',
    'remaining_distance',
]
REFERENCE_FIELDS = [
    'ref_x',
    'ref_y',
    'ref_yaw',
    'ref_distance',
    'ref_axis_distance',
    'odom_ref_error',
    'error_rate',
    'scale_estimate',
]


@dataclass
class Parameters:
    odom_topic: str = '/odom'
    cmd_vel_topic: str = '/cmd_vel'
    use_reference_pose: bool = False
    reference_pose_topic: str = ''
    axis: str = 'x'
    direction: int = 1
    target_distance: float = 1.0
    output_dir: str = 'logs'
    output_prefix: str = 'motion'
    record_rate_hz: float = 20.0
    summary_on_shutdown: bool = True


@dataclass
class CmdSample:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


@dataclass
class PoseSample:
    x: float
    y: float
    yaw: float


class MotionDataRecorder(Node):
    def __init__(self):
        super().__init__('motion_data_recorder')

        self._params = self._declare_and_load_parameters()
        self._validate_parameters()

        self._lock = Lock()
        self._latest_cmd = CmdSample()
        self._latest_odom = None
        self._latest_ref = None
        self._start_odom = None
        self._start_ref = None
        self._start_time = None
        self._recording_started = False
        self._last_nonfinite_odom_warn_ns = 0
        self._last_nonfinite_ref_warn_ns = 0

        self._csv_file = None
        self._csv_writer = None
        self._csv_path = None
        self._sample_count = 0
        self._max_cmd_velocity = 0.0
        self._sum_cmd_velocity = 0.0
        self._last_summary = None
        self._closed = False

        self._cmd_sub = self.create_subscription(
            Twist,
            self._params.cmd_vel_topic,
            self._cmd_vel_callback,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._params.odom_topic,
            self._odom_callback,
            ODOM_QOS,
        )
        self._ref_sub = None
        if self._params.use_reference_pose:
            self._ref_sub = self.create_subscription(
                PoseStamped,
                self._params.reference_pose_topic,
                self._reference_pose_callback,
                10,
            )

        self._record_timer = self.create_timer(
            1.0 / self._params.record_rate_hz,
            self._record_timer_callback,
        )

        self.get_logger().info(
            "Recording motion data from '%s' and '%s'"
            % (self._params.cmd_vel_topic, self._params.odom_topic)
        )
        if self._params.use_reference_pose:
            self.get_logger().info(
                "Reference pose recording enabled on '%s'"
                % self._params.reference_pose_topic
            )

    def _declare_and_load_parameters(self):
        defaults = Parameters()
        values = {}
        for name, default in defaults.__dict__.items():
            descriptor = None
            if name in ('axis', 'direction'):
                descriptor = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameter(name, default, descriptor)
            try:
                values[name] = self.get_parameter(name).value
            except ParameterUninitializedException:
                values[name] = default
        return Parameters(**values)

    def _validate_parameters(self):
        try:
            validate_positive_finite('record_rate_hz', self._params.record_rate_hz)
            validate_positive_finite('target_distance', self._params.target_distance)
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

        self._params.axis = normalize_axis(self._params.axis)
        if self._params.axis not in VALID_AXES:
            raise RuntimeError("axis must be either 'x' or 'y'")

        try:
            self._params.direction = normalize_direction(self._params.direction)
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

        try:
            self._params.reference_pose_topic = normalize_reference_pose_topic(
                self._params.use_reference_pose,
                self._params.reference_pose_topic,
            )
        except ValueError as exc:
            raise RuntimeError(str(exc)) from exc

    def _cmd_vel_callback(self, msg):
        with self._lock:
            self._latest_cmd = CmdSample(
                vx=msg.linear.x,
                vy=msg.linear.y,
                wz=msg.angular.z,
            )

    def _odom_callback(self, msg):
        pose = msg.pose.pose
        if not self._is_finite_pose(pose.position.x, pose.position.y, pose.orientation):
            self._warn_nonfinite_odom()
            return

        sample = PoseSample(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw_from_quaternion(pose.orientation),
        )
        with self._lock:
            self._latest_odom = sample
            if self._start_odom is None:
                self._start_odom = sample
                self.get_logger().info(
                    'Latched odometry start pose: x=%.6f, y=%.6f'
                    % (sample.x, sample.y)
                )

    def _reference_pose_callback(self, msg):
        pose = msg.pose
        if not self._is_finite_pose(pose.position.x, pose.position.y, pose.orientation):
            self._warn_nonfinite_reference_pose()
            return

        sample = PoseSample(
            x=pose.position.x,
            y=pose.position.y,
            yaw=yaw_from_quaternion(pose.orientation),
        )
        with self._lock:
            self._latest_ref = sample
            if self._start_ref is None:
                self._start_ref = sample
                self.get_logger().info(
                    'Latched reference start pose: x=%.6f, y=%.6f'
                    % (sample.x, sample.y)
                )

    def _record_timer_callback(self):
        with self._lock:
            if not self._ready_to_record_locked():
                return
            if not self._recording_started:
                self._start_recording_locked()
            row, metrics = self._build_row_locked()

        self._csv_writer.writerow(row)
        self._csv_file.flush()
        self._update_summary_metrics(metrics)

    def _ready_to_record_locked(self):
        if self._start_odom is None or self._latest_odom is None:
            return False
        if self._params.use_reference_pose:
            return self._start_ref is not None and self._latest_ref is not None
        return True

    def _start_recording_locked(self):
        output_dir = Path(self._params.output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{self._params.output_prefix}_{timestamp}.csv'
        self._csv_path = output_dir / filename
        self._csv_file = self._csv_path.open('w', newline='')
        fieldnames = list(BASE_FIELDS)
        if self._params.use_reference_pose:
            fieldnames.extend(REFERENCE_FIELDS)
        self._csv_writer = csv.DictWriter(self._csv_file, fieldnames=fieldnames)
        self._csv_writer.writeheader()
        self._start_time = self.get_clock().now()
        self._recording_started = True
        self.get_logger().info("Started recording CSV: '%s'" % self._csv_path)

    def _build_row_locked(self):
        odom = self._latest_odom
        cmd = self._latest_cmd
        start_odom = self._start_odom
        time_sec = self._elapsed_since_start()
        odom_distance = distance_2d(start_odom.x, start_odom.y, odom.x, odom.y)
        axis_distance = compute_axis_distance(
            start_odom.x,
            start_odom.y,
            odom.x,
            odom.y,
            self._params.axis,
            self._params.direction,
        )
        remaining_distance = compute_remaining_distance(
            self._params.target_distance,
            axis_distance,
        )
        row = {
            'time_sec': self._format_float(time_sec),
            'cmd_vx': self._format_float(cmd.vx),
            'cmd_vy': self._format_float(cmd.vy),
            'cmd_wz': self._format_float(cmd.wz),
            'odom_x': self._format_float(odom.x),
            'odom_y': self._format_float(odom.y),
            'odom_yaw': self._format_float(odom.yaw),
            'odom_distance': self._format_float(odom_distance),
            'axis_distance': self._format_float(axis_distance),
            'remaining_distance': self._format_float(remaining_distance),
        }
        metrics = {
            'time_sec': time_sec,
            'cmd_speed': math.hypot(cmd.vx, cmd.vy),
            'odom_distance': odom_distance,
            'axis_distance': axis_distance,
            'remaining_distance': remaining_distance,
        }

        if self._params.use_reference_pose:
            self._add_reference_values_locked(row, metrics)

        return row, metrics

    def _add_reference_values_locked(self, row, metrics):
        ref = self._latest_ref
        start_ref = self._start_ref
        ref_distance = distance_2d(start_ref.x, start_ref.y, ref.x, ref.y)
        ref_axis_distance = compute_axis_distance(
            start_ref.x,
            start_ref.y,
            ref.x,
            ref.y,
            self._params.axis,
            self._params.direction,
        )
        odom_ref_error = metrics['axis_distance'] - ref_axis_distance
        error_rate = safe_ratio(odom_ref_error, ref_axis_distance)
        scale_estimate = safe_ratio(ref_axis_distance, metrics['axis_distance'])

        row.update({
            'ref_x': self._format_float(ref.x),
            'ref_y': self._format_float(ref.y),
            'ref_yaw': self._format_float(ref.yaw),
            'ref_distance': self._format_float(ref_distance),
            'ref_axis_distance': self._format_float(ref_axis_distance),
            'odom_ref_error': self._format_float(odom_ref_error),
            'error_rate': self._format_optional_float(error_rate),
            'scale_estimate': self._format_optional_float(scale_estimate),
        })
        metrics.update({
            'ref_distance': ref_distance,
            'ref_axis_distance': ref_axis_distance,
            'odom_ref_error': odom_ref_error,
            'error_rate': error_rate,
            'scale_estimate': scale_estimate,
        })

    def _elapsed_since_start(self):
        if self._start_time is None:
            return 0.0
        return (self.get_clock().now() - self._start_time).nanoseconds * 1.0e-9

    def _warn_nonfinite_odom(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_nonfinite_odom_warn_ns >= 2_000_000_000:
            self.get_logger().warning('Ignoring non-finite odometry pose')
            self._last_nonfinite_odom_warn_ns = now_ns

    def _warn_nonfinite_reference_pose(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_nonfinite_ref_warn_ns >= 2_000_000_000:
            self.get_logger().warning('Ignoring non-finite reference pose')
            self._last_nonfinite_ref_warn_ns = now_ns

    def _update_summary_metrics(self, metrics):
        self._sample_count += 1
        self._max_cmd_velocity = max(self._max_cmd_velocity, metrics['cmd_speed'])
        self._sum_cmd_velocity += metrics['cmd_speed']
        self._last_summary = metrics

    def close(self):
        if self._closed:
            return
        self._closed = True

        if self._params.summary_on_shutdown:
            print(self._format_summary())

        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None

    def _format_summary(self):
        lines = ['=== Motion Data Summary ===']
        if self._sample_count == 0 or self._last_summary is None:
            lines.append('No samples recorded')
            return '\n'.join(lines)

        avg_cmd_velocity = self._sum_cmd_velocity / self._sample_count
        lines.extend([
            f'Axis                : {self._params.axis}',
            f'Direction           : {self._params.direction:+d}',
            f'Target distance     : {self._params.target_distance:.3f} m',
            f"Final odom distance : {self._last_summary['odom_distance']:.3f} m",
            f"Axis distance       : {self._last_summary['axis_distance']:.3f} m",
            f"Remaining distance  : {self._last_summary['remaining_distance']:.3f} m",
            f"Duration            : {self._last_summary['time_sec']:.2f} s",
            f'Max cmd velocity    : {self._max_cmd_velocity:.3f} m/s',
            f'Avg cmd velocity    : {avg_cmd_velocity:.3f} m/s',
        ])

        if self._params.use_reference_pose:
            self._append_reference_summary(lines)

        lines.append(f'CSV saved           : {self._csv_path}')
        return '\n'.join(lines)

    def _append_reference_summary(self, lines):
        summary = self._last_summary
        lines.extend([
            f"Reference distance  : {summary['ref_distance']:.3f} m",
            f"Reference axis dist : {summary['ref_axis_distance']:.3f} m",
            f"Odom-ref error      : {summary['odom_ref_error']:+.3f} m",
        ])
        error_rate = summary['error_rate']
        scale_estimate = summary['scale_estimate']
        lines.append(
            'Odom-ref error rate : '
            + ('' if error_rate is None else f'{error_rate * 100.0:+.2f} %')
        )
        lines.append(
            'Scale estimate      : '
            + ('' if scale_estimate is None else f'{scale_estimate:.4f}')
        )

    @staticmethod
    def _is_finite_pose(x, y, orientation):
        return (
            math.isfinite(x) and
            math.isfinite(y) and
            math.isfinite(orientation.x) and
            math.isfinite(orientation.y) and
            math.isfinite(orientation.z) and
            math.isfinite(orientation.w)
        )

    @staticmethod
    def _format_float(value):
        return f'{value:.9f}'

    @staticmethod
    def _format_optional_float(value):
        if value is None:
            return ''
        return f'{value:.9f}'


def main(args=None):
    rclpy.init(args=args)
    node = MotionDataRecorder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
