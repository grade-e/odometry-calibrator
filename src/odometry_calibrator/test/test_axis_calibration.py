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

from odometry_calibrator.axis import normalize_axis
from odometry_calibrator.axis import OdomDistanceCalculator


def test_odom_distance_calculator_uses_selected_axis():
    calculator = OdomDistanceCalculator()
    calculator.set_start(1.0, 2.0)

    assert calculator.displacement_from_start(1.7, 5.0, 'x') == 0.7
    assert calculator.displacement_from_start(1.7, 5.0, 'y') == 3.0


def test_axis_normalization_accepts_ros_yaml_boolean_y():
    assert normalize_axis(True) == 'y'
    assert normalize_axis(False) == 'x'
    assert normalize_axis('Y') == 'y'
