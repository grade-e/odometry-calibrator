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

from dataclasses import dataclass
from enum import Enum
import math
from threading import Lock

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from odometry_calibrator.calibration.cli_measurement_provider import CliMeasurementProvider
from odometry_calibrator.calibration.parameter_measurement_provider import (
    ParameterMeasurementProvider,
)
from odometry_calibrator.common.axis import normalize_axis
from odometry_calibrator.common.axis import normalize_direction
from odometry_calibrator.common.axis import OdomDistanceCalculator
from odometry_calibrator.common.axis import signed_linear_components
from odometry_calibrator.common.axis import VALID_AXES
from odometry_calibrator.common.axis import VALID_DIRECTIONS
from odometry_calibrator.common.result import format_calibration_result
from odometry_calibrator.integration import exit_codes
from odometry_calibrator.integration.artifact_writer import ArtifactWriter
from odometry_calibrator.integration.event_writer import EventWriter
from odometry_calibrator.integration.result_schema import build_calibration_result
from odometry_calibrator.integration.result_schema import STATUS_FAILED
from odometry_calibrator.integration.result_schema import STATUS_SUCCESS
from odometry_calibrator.integration.result_schema import STATUS_TIMEOUT
from rcl_interfaces.msg import ParameterDescriptor
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy
from rclpy.qos import HistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import ReliabilityPolicy


MINIMUM_ODOM_DISTANCE_FOR_SCALE = 1.0e-6
CONTROL_MODE_CONSTANT = 'constant'
CONTROL_MODE_P = 'p'
CONTROL_MODE_P_MIN_CLAMPED = 'p_min_clamped'
CONTROL_MODE_P_STOP_THRESHOLD = 'p_stop_threshold'
CONTROL_MODE_ALIASES = {
    'normal': CONTROL_MODE_CONSTANT,
    'p_min_velocity': CONTROL_MODE_P_MIN_CLAMPED,
}
MEASUREMENT_SOURCE_CLI = 'cli'
MEASUREMENT_SOURCE_PARAMETER = 'parameter'
VALID_MEASUREMENT_SOURCES = (
    MEASUREMENT_SOURCE_CLI,
    MEASUREMENT_SOURCE_PARAMETER,
)
VALID_CONTROL_MODES = (
    CONTROL_MODE_CONSTANT,
    CONTROL_MODE_P,
    CONTROL_MODE_P_MIN_CLAMPED,
    CONTROL_MODE_P_STOP_THRESHOLD,
)
ODOM_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class State(Enum):
    INIT = 'INIT'
    MOVING_ACCEL_LIMIT = 'MOVING_ACCEL_LIMIT'
    MOVING_P_CONTROL = 'MOVING_P_CONTROL'
    WAIT_FOR_MEASUREMENT = 'WAIT_FOR_MEASUREMENT'
    CALCULATE = 'CALCULATE'
    DONE = 'DONE'


@dataclass
class Parameters:
    axis: str = 'x'
    direction: int = 1
    odom_topic: str = '/odom'
    cmd_vel_topic: str = '/cmd_vel'
    cmd_vel_frame_id: str = 'base_link'
    target_distance: float = 1.0
    distance_tolerance: float = 0.005
    control_mode: str = CONTROL_MODE_P_MIN_CLAMPED
    kp: float = 0.4
    max_velocity: float = 0.1
    min_velocity: float = 0.05
    max_acceleration: float = 0.05
    control_rate_hz: float = 20.0
    stop_publish_rate_hz: float = 10.0
    motion_timeout_sec: float = 30.0
    keep_alive_after_done: bool = True
    output_dir: str = 'logs'
    result_filename: str = 'result.json'
    events_filename: str = 'events.jsonl'
    measurement_source: str = MEASUREMENT_SOURCE_CLI
    actual_distance_m: float = 0.0
    result_status_mode: str = 'calculate_only'


class InvalidCalibrationParameterError(RuntimeError):
    """Raised when ROS parameters do not satisfy the calibrator contract."""


class OdomLinearCalibrator(Node):
    def __init__(self, measurement_provider=None):
        super().__init__('odom_linear_calibrator')
        self._params = self._declare_and_load_parameters()
        self._artifact_writer = ArtifactWriter(
            self._params.output_dir,
            self._params.result_filename,
            self._params.events_filename,
        )
        self._event_writer = EventWriter(self._artifact_writer.events_path)
        self._validate_parameters()
        self._measurement_provider = (
            measurement_provider or self._create_measurement_provider()
        )

        self._state = State.INIT
        self._distance_calculator = OdomDistanceCalculator()
        self._odom_lock = Lock()
        self._current_odom_distance = 0.0
        self._final_odom_distance_latched = False
        self._final_odom_distance = 0.0
        self._motion_start_time = None
        self._last_control_time = None
        self._last_cmd_velocity = 0.0
        self._calculation_done = False
        self._exit_code = exit_codes.SUCCESS
        self._motion_timed_out = False
        self._last_nonfinite_warn_ns = 0
        self._last_reverse_motion_warn_ns = 0

        self._artifact_writer.ensure_output_dir()
        self._event_writer.write(
            'started',
            axis=self._params.axis,
            direction=self._params.direction,
        )

        self._cmd_vel_pub = self.create_publisher(
            TwistStamped,
            self._params.cmd_vel_topic,
            10,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self._params.odom_topic,
            self._odom_callback,
            ODOM_QOS,
        )
        self._control_timer = self.create_timer(
            1.0 / self._params.control_rate_hz,
            self._control_timer_callback,
        )
        self._stop_timer = self.create_timer(
            1.0 / self._params.stop_publish_rate_hz,
            self._stop_timer_callback,
        )

        self.get_logger().info(
            f"Waiting for first odometry message on '{self._params.odom_topic}' "
            f"with control_mode='{self._params.control_mode}'"
        )

    def _declare_and_load_parameters(self):
        defaults = Parameters()
        values = {}
        for name, default in defaults.__dict__.items():
            descriptor = None
            if name in ('axis', 'direction'):
                descriptor = ParameterDescriptor(dynamic_typing=True)
            self.declare_parameter(name, default, descriptor)
            values[name] = self.get_parameter(name).value
        return Parameters(**values)

    def _validate_parameters(self):
        for name in (
            'target_distance',
            'distance_tolerance',
            'kp',
            'max_velocity',
            'min_velocity',
            'max_acceleration',
            'control_rate_hz',
            'stop_publish_rate_hz',
            'motion_timeout_sec',
        ):
            value = getattr(self._params, name)
            if value <= 0.0 or not math.isfinite(value):
                raise InvalidCalibrationParameterError(
                    f'{name} must be positive and finite'
                )

        if self._params.min_velocity > self._params.max_velocity:
            raise InvalidCalibrationParameterError(
                'min_velocity must be less than or equal to max_velocity'
            )

        self._params.control_mode = normalize_control_mode(self._params.control_mode)

        self._params.axis = normalize_axis(self._params.axis)
        if self._params.axis not in VALID_AXES:
            raise InvalidCalibrationParameterError("axis must be either 'x' or 'y'")

        try:
            self._params.direction = normalize_direction(self._params.direction)
        except ValueError as exc:
            raise InvalidCalibrationParameterError(str(exc)) from exc
        if self._params.direction not in VALID_DIRECTIONS:
            raise InvalidCalibrationParameterError('direction must be 1 or -1')

        self._params.measurement_source = (
            str(self._params.measurement_source).strip().lower()
        )
        if self._params.measurement_source not in VALID_MEASUREMENT_SOURCES:
            valid_sources = ', '.join(VALID_MEASUREMENT_SOURCES)
            raise InvalidCalibrationParameterError(
                f'measurement_source must be one of: {valid_sources}'
            )

        if (
            self._params.measurement_source == MEASUREMENT_SOURCE_PARAMETER and
            not ParameterMeasurementProvider.is_valid_measurement(
                self._params.actual_distance_m
            )
        ):
            raise InvalidCalibrationParameterError(
                'actual_distance_m must be finite and in the range '
                '0.0 < actual_distance_m <= 10.0 when measurement_source=parameter'
            )

        if self._params.distance_tolerance >= self._params.target_distance:
            self.get_logger().warning(
                'distance_tolerance is greater than or equal to target_distance; '
                'calibration may stop early'
            )

        if self._params.stop_publish_rate_hz < 10.0:
            self.get_logger().warning(
                'stop_publish_rate_hz %.3f is below 10 Hz; clamping to 10 Hz'
                % self._params.stop_publish_rate_hz
            )
            self._params.stop_publish_rate_hz = 10.0

    def _create_measurement_provider(self):
        if self._params.measurement_source == MEASUREMENT_SOURCE_PARAMETER:
            return ParameterMeasurementProvider(self._params.actual_distance_m)
        return CliMeasurementProvider()

    def _odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if not math.isfinite(x) or not math.isfinite(y):
            self._warn_nonfinite_odom()
            return

        with self._odom_lock:
            if self._distance_calculator.has_start():
                odom_displacement = self._distance_calculator.displacement_from_start(
                    x,
                    y,
                    self._params.axis,
                )
                directed_displacement = self._params.direction * odom_displacement
                if directed_displacement < -self._params.distance_tolerance:
                    self._warn_reverse_motion(directed_displacement)
                self._current_odom_distance = max(directed_displacement, 0.0)

        if self._state == State.INIT:
            with self._odom_lock:
                self._distance_calculator.set_start(x, y)
                self._current_odom_distance = 0.0
            self._motion_start_time = self.get_clock().now()
            self._last_control_time = self._motion_start_time
            self._transition_to(State.MOVING_ACCEL_LIMIT)

    def _warn_nonfinite_odom(self):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_nonfinite_warn_ns >= 2_000_000_000:
            message = 'Ignoring non-finite odometry pose'
            self.get_logger().warning(message)
            self._event_writer.warning(message)
            self._last_nonfinite_warn_ns = now_ns

    def _warn_reverse_motion(self, directed_displacement):
        now_ns = self.get_clock().now().nanoseconds
        if now_ns - self._last_reverse_motion_warn_ns >= 2_000_000_000:
            message = (
                'Odometry moved opposite to requested direction on %s axis: %.6f m'
                % (self._params.axis, directed_displacement)
            )
            self.get_logger().warning(message)
            self._event_writer.warning(
                message,
                directed_displacement_m=directed_displacement,
            )
            self._last_reverse_motion_warn_ns = now_ns

    def _control_timer_callback(self):
        current_time = self.get_clock().now()

        if self._state == State.INIT:
            return
        if self._state == State.MOVING_ACCEL_LIMIT:
            self._update_moving_state(current_time)
            return
        if self._state == State.MOVING_P_CONTROL:
            self._update_moving_state(current_time)
            return
        if self._state == State.WAIT_FOR_MEASUREMENT:
            if self._measurement_provider.has_measurement():
                self._transition_to(State.CALCULATE)
            return
        if self._state == State.CALCULATE:
            self._calculate_result()
            self._transition_to(State.DONE)

    def _stop_timer_callback(self):
        if self._state in (State.WAIT_FOR_MEASUREMENT, State.DONE):
            self._publish_stop()

    def _transition_to(self, next_state):
        if self._state == next_state:
            return

        self.get_logger().info(f'State transition: {self._state.value} -> {next_state.value}')
        previous_state = self._state
        self._event_writer.state_transition(previous_state.value, next_state.value)
        self._state = next_state

        if self._state == State.WAIT_FOR_MEASUREMENT:
            self._final_odom_distance = self._current_odom_distance_value()
            self._final_odom_distance_latched = True
            self._last_cmd_velocity = 0.0
            self._publish_stop()
            self._measurement_provider.start()
        elif self._state == State.DONE:
            self._publish_stop()
            if not self._params.keep_alive_after_done:
                self.get_logger().info(
                    'Calibration done; shutting down because keep_alive_after_done=false'
                )
                rclpy.shutdown()

    def _update_moving_state(self, current_time):
        odom_distance = self._current_odom_distance_value()
        remaining = self._params.target_distance - odom_distance
        elapsed = self._seconds_between(current_time, self._motion_start_time)

        if (
            remaining <= self._params.distance_tolerance or
            elapsed >= self._params.motion_timeout_sec
        ):
            if elapsed >= self._params.motion_timeout_sec:
                self._motion_timed_out = True
                message = 'Motion timeout reached before target distance'
                self.get_logger().warning(message)
                self._event_writer.warning(
                    message,
                    elapsed_sec=elapsed,
                    target_distance_m=self._params.target_distance,
                    odom_distance_m=odom_distance,
                )
            self._transition_to(State.WAIT_FOR_MEASUREMENT)
            return

        raw_velocity = compute_target_velocity(
            self._params.control_mode,
            remaining,
            self._params.kp,
            self._params.min_velocity,
            self._params.max_velocity,
        )
        if (
            self._params.control_mode == CONTROL_MODE_P_STOP_THRESHOLD
            and raw_velocity <= 0.0
        ):
            self._transition_to(State.WAIT_FOR_MEASUREMENT)
            return

        dt = max(self._seconds_between(current_time, self._last_control_time), 0.0)
        self._last_control_time = current_time
        limited_velocity = self._rate_limit(raw_velocity, dt)

        self._last_cmd_velocity = limited_velocity
        self._publish_velocity(limited_velocity)

        if self._state == State.MOVING_ACCEL_LIMIT:
            command_near_limit = limited_velocity >= self._params.max_velocity * 0.95
            acceleration_window_elapsed = elapsed >= 1.0
            if command_near_limit or acceleration_window_elapsed:
                self._transition_to(State.MOVING_P_CONTROL)

    def _calculate_result(self):
        if self._calculation_done:
            return
        self._calculation_done = True

        odom_distance = (
            self._final_odom_distance
            if self._final_odom_distance_latched
            else self._current_odom_distance_value()
        )
        if odom_distance <= MINIMUM_ODOM_DISTANCE_FOR_SCALE:
            message = (
                f'Cannot calculate K_{self._params.axis} because D_odom is too small: '
                f'{odom_distance:.9f} m'
            )
            self.get_logger().error(message)
            self._exit_code = exit_codes.CALIBRATION_FAILED
            self._write_result(
                STATUS_FAILED,
                odom_distance,
                None,
                None,
                message,
            )
            self._event_writer.error(message, odom_distance_m=odom_distance)
            return

        actual_distance = self._measurement_provider.get_measurement()
        k_axis = actual_distance / odom_distance
        status = STATUS_TIMEOUT if self._motion_timed_out else STATUS_SUCCESS
        message = 'Linear odometry scale factor calculated.'
        if self._motion_timed_out:
            self._exit_code = exit_codes.MOTION_TIMEOUT
            message = 'Linear odometry scale factor calculated after motion timeout.'

        self._event_writer.write(
            'measurement_received',
            actual_distance_m=actual_distance,
        )

        print(format_calibration_result(
            self._params.axis,
            self._params.direction,
            odom_distance,
            actual_distance,
            k_axis,
        ))

        self.get_logger().info(
            'Linear odometry scale factor calculated: '
            f'axis={self._params.axis}, direction={self._params.direction}, K={k_axis:.6f}'
        )
        self._write_result(status, odom_distance, actual_distance, k_axis, message)
        self._event_writer.write('result', status=status, scale_factor=k_axis)

    def _write_result(
        self,
        status,
        odom_distance_m,
        actual_distance_m,
        scale_factor,
        message,
    ):
        artifacts = {
            'events': self._artifact_writer.relative_artifact_path(
                self._artifact_writer.events_path,
            ),
        }
        result = build_calibration_result(
            self._params,
            status,
            odom_distance_m,
            actual_distance_m,
            scale_factor,
            artifacts,
            message,
        )
        self._artifact_writer.write_result(result)

    def _publish_velocity(self, linear_velocity):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = self._params.cmd_vel_frame_id
        cmd.twist.linear.x, cmd.twist.linear.y = signed_linear_components(
            self._params.axis,
            self._params.direction,
            linear_velocity,
        )
        cmd.twist.angular.z = 0.0
        self._cmd_vel_pub.publish(cmd)

    def _publish_stop(self):
        self._publish_velocity(0.0)

    def _rate_limit(self, target_velocity, dt):
        if dt <= 0.0:
            return self._last_cmd_velocity

        max_delta = self._params.max_acceleration * dt
        lower = max(0.0, self._last_cmd_velocity - max_delta)
        upper = self._last_cmd_velocity + max_delta
        return min(max(target_velocity, lower), upper)

    def _current_odom_distance_value(self):
        with self._odom_lock:
            return self._current_odom_distance

    @staticmethod
    def _seconds_between(later, earlier):
        return (later - earlier).nanoseconds * 1.0e-9


def normalize_control_mode(value):
    mode = str(value).strip().lower()
    mode = CONTROL_MODE_ALIASES.get(mode, mode)
    if mode not in VALID_CONTROL_MODES:
        valid_modes = ', '.join(VALID_CONTROL_MODES)
        raise InvalidCalibrationParameterError(
            f'control_mode must be one of: {valid_modes}'
        )
    return mode


def compute_target_velocity(control_mode, remaining_distance, kp, min_velocity, max_velocity):
    if remaining_distance <= 0.0:
        return 0.0

    if control_mode == CONTROL_MODE_CONSTANT:
        return max_velocity

    proportional_velocity = kp * remaining_distance
    p_velocity = min(max(proportional_velocity, 0.0), max_velocity)

    if control_mode == CONTROL_MODE_P:
        return p_velocity
    if control_mode == CONTROL_MODE_P_MIN_CLAMPED:
        return min(max(p_velocity, min_velocity), max_velocity)
    if control_mode == CONTROL_MODE_P_STOP_THRESHOLD:
        if p_velocity < min_velocity:
            return 0.0
        return p_velocity

    raise RuntimeError(f'unsupported control_mode: {control_mode}')


def main(args=None):
    node = None
    exit_code = exit_codes.SUCCESS
    try:
        rclpy.init(args=args)
        node = OdomLinearCalibrator()
        rclpy.spin(node)
        exit_code = node._exit_code
    except KeyboardInterrupt:
        exit_code = exit_codes.MEASUREMENT_CANCELLED
    except InvalidCalibrationParameterError as exc:
        print(f'Invalid calibration parameter: {exc}')
        exit_code = exit_codes.INVALID_PARAMETER
    except ExternalShutdownException:
        if node is not None:
            exit_code = node._exit_code
    except Exception as exc:
        print(f'Unhandled odometry calibration error: {exc}')
        exit_code = exit_codes.INTERNAL_ERROR
    finally:
        if node is not None:
            try:
                node._publish_stop()
            except Exception:
                pass
        if node is not None and hasattr(node._measurement_provider, 'stop'):
            node._measurement_provider.stop()
        if node is not None:
            node._event_writer.close()
            try:
                node.destroy_node()
            except KeyboardInterrupt:
                pass
        if rclpy.ok():
            rclpy.shutdown()
    return exit_code
