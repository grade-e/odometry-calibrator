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

from types import SimpleNamespace

from odometry_calibrator.integration.result_schema import build_calibration_result
from odometry_calibrator.integration.result_schema import candidate_parameter_name
from odometry_calibrator.integration.result_schema import STATUS_SUCCESS
import pytest


@pytest.mark.parametrize(
    'axis,direction,expected',
    [
        ('x', 1, 'wheel_odom_scale_x_positive'),
        ('x', -1, 'wheel_odom_scale_x_negative'),
        ('y', 1, 'wheel_odom_scale_y_positive'),
        ('y', -1, 'wheel_odom_scale_y_negative'),
    ],
)
def test_candidate_parameter_name_uses_axis_and_direction(
    axis,
    direction,
    expected,
):
    assert candidate_parameter_name(axis, direction) == expected


def test_build_calibration_result_contains_external_contract_fields():
    params = SimpleNamespace(
        axis='x',
        direction=1,
        target_distance=1.0,
        odom_topic='/odom',
        cmd_vel_topic='/cmd_vel',
        control_mode='p_min_clamped',
        max_velocity=0.1,
        min_velocity=0.05,
        max_acceleration=0.05,
        motion_timeout_sec=30.0,
    )

    result = build_calibration_result(
        params,
        STATUS_SUCCESS,
        1.003,
        0.985,
        0.982054,
        {'events': 'events.jsonl'},
        'Linear odometry scale factor calculated.',
    )

    assert result['schema_version'] == '1.0'
    assert result['tool'] == 'odometry-calibrator'
    assert result['task_type'] == 'odometry_linear_calibration'
    assert result['candidate']['parameter_name'] == 'wheel_odom_scale_x_positive'
    assert result['candidate']['apply_policy'] == 'manual_review_required'
    assert result['topics']['cmd_vel_type'] == 'geometry_msgs/msg/TwistStamped'
    assert result['artifacts']['events'] == 'events.jsonl'
