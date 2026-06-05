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

from odometry_calibrator.axis import normalize_direction
from odometry_calibrator.axis import signed_linear_components
import pytest


def test_normalize_direction_accepts_int_and_string_values():
    assert normalize_direction(1) == 1
    assert normalize_direction(-1) == -1
    assert normalize_direction('1') == 1
    assert normalize_direction('-1') == -1


@pytest.mark.parametrize('value', [0, 2, 'x', True, False])
def test_normalize_direction_rejects_invalid_values(value):
    with pytest.raises(ValueError):
        normalize_direction(value)


@pytest.mark.parametrize(
    ('axis', 'direction', 'expected'),
    [
        ('x', 1, (0.1, 0.0)),
        ('x', -1, (-0.1, 0.0)),
        ('y', 1, (0.0, 0.1)),
        ('y', -1, (0.0, -0.1)),
    ],
)
def test_signed_linear_components(axis, direction, expected):
    assert signed_linear_components(axis, direction, 0.1) == expected
