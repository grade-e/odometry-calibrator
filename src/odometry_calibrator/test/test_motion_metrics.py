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

from odometry_calibrator.common.motion_metrics import compute_axis_distance
from odometry_calibrator.common.motion_metrics import compute_remaining_distance
from odometry_calibrator.common.motion_metrics import safe_ratio


def test_compute_axis_distance_for_positive_x_motion():
    assert compute_axis_distance(0.0, 0.0, 1.0, 0.0, 'x', 1) == 1.0


def test_compute_axis_distance_for_negative_x_motion():
    assert compute_axis_distance(0.0, 0.0, -1.0, 0.0, 'x', -1) == 1.0


def test_compute_axis_distance_for_positive_y_motion():
    assert compute_axis_distance(0.0, 0.0, 0.0, 1.0, 'y', 1) == 1.0


def test_compute_axis_distance_for_negative_y_motion():
    assert compute_axis_distance(0.0, 0.0, 0.0, -1.0, 'y', -1) == 1.0


def test_compute_axis_distance_clamps_opposite_direction_motion():
    assert compute_axis_distance(0.0, 0.0, -1.0, 0.0, 'x', 1) == 0.0


def test_compute_remaining_distance():
    assert compute_remaining_distance(1.0, 0.25) == 0.75


def test_safe_ratio_returns_ratio_for_normal_denominator():
    assert safe_ratio(2.0, 4.0) == 0.5


def test_safe_ratio_returns_none_for_small_denominator():
    assert safe_ratio(2.0, 1.0e-9) is None
