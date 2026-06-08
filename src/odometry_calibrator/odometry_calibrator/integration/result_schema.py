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

SCHEMA_VERSION = '1.0'
TOOL_NAME = 'odometry-calibrator'
TASK_TYPE = 'odometry_linear_calibration'
CMD_VEL_TYPE = 'geometry_msgs/msg/TwistStamped'
APPLY_POLICY_MANUAL_REVIEW = 'manual_review_required'

STATUS_SUCCESS = 'success'
STATUS_FAILED = 'failed'
STATUS_TIMEOUT = 'timeout'
STATUS_INVALID_PARAMETER = 'invalid_parameter'
STATUS_CANCELLED = 'cancelled'
STATUS_INTERNAL_ERROR = 'internal_error'
VALID_STATUSES = (
    STATUS_SUCCESS,
    STATUS_FAILED,
    STATUS_TIMEOUT,
    STATUS_INVALID_PARAMETER,
    STATUS_CANCELLED,
    STATUS_INTERNAL_ERROR,
)


def candidate_parameter_name(axis, direction):
    direction_label = 'positive' if int(direction) == 1 else 'negative'
    return f'wheel_odom_scale_{axis}_{direction_label}'


def build_calibration_result(
    params,
    status,
    odom_distance_m,
    actual_distance_m,
    scale_factor,
    artifacts,
    message,
):
    if status not in VALID_STATUSES:
        raise ValueError(f'unsupported result status: {status}')

    return {
        'schema_version': SCHEMA_VERSION,
        'tool': TOOL_NAME,
        'task_type': TASK_TYPE,
        'status': status,
        'axis': params.axis,
        'direction': params.direction,
        'target_distance_m': params.target_distance,
        'odom_distance_m': odom_distance_m,
        'actual_distance_m': actual_distance_m,
        'scale_factor': scale_factor,
        'candidate': {
            'parameter_name': candidate_parameter_name(
                params.axis,
                params.direction,
            ),
            'value': scale_factor,
            'apply_policy': APPLY_POLICY_MANUAL_REVIEW,
        },
        'topics': {
            'odom_topic': params.odom_topic,
            'cmd_vel_topic': params.cmd_vel_topic,
            'cmd_vel_type': CMD_VEL_TYPE,
        },
        'control': {
            'control_mode': params.control_mode,
            'max_velocity': params.max_velocity,
            'min_velocity': params.min_velocity,
            'max_acceleration': params.max_acceleration,
            'motion_timeout_sec': params.motion_timeout_sec,
        },
        'artifacts': artifacts,
        'message': message,
    }
