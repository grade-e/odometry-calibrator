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

from odometry_calibrator.common.result import format_calibration_result


def test_format_calibration_result_includes_positive_x_label():
    result = format_calibration_result('x', 1, 0.5, 0.492, 0.984)

    assert 'Axis      : x' in result
    assert 'Direction : +1' in result
    assert 'K_x(+1)   : 0.984000' in result


def test_format_calibration_result_includes_negative_y_label():
    result = format_calibration_result('y', -1, 0.5, 0.486, 0.972)

    assert 'Axis      : y' in result
    assert 'Direction : -1' in result
    assert 'K_y(-1)   : 0.972000' in result


def test_format_calibration_result_rounds_distances_and_scale():
    result = format_calibration_result('x', 1, 0.4999, 0.4916, 0.9833333)

    assert 'D_odom    : 0.500 m' in result
    assert 'D_actual  : 0.492 m' in result
    assert 'K_x(+1)   : 0.983333' in result
