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

from odometry_calibrator.parameters import validate_positive_finite
import pytest


def test_validate_positive_finite_accepts_positive_finite_value():
    validate_positive_finite('mock_speed', 0.05)


@pytest.mark.parametrize('value', [0.0, -0.05, float('inf'), float('nan')])
def test_validate_positive_finite_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        validate_positive_finite('mock_speed', value)
