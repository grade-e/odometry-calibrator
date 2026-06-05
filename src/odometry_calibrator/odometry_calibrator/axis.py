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

VALID_AXES = ('x', 'y')
VALID_DIRECTIONS = (1, -1)


def normalize_axis(axis):
    if isinstance(axis, bool):
        return 'y' if axis else 'x'
    return str(axis).lower()


def normalize_direction(value):
    if isinstance(value, bool):
        raise ValueError('direction must be 1 or -1, not bool')
    try:
        direction = int(value)
    except (TypeError, ValueError):
        raise ValueError('direction must be 1 or -1') from None
    if direction not in VALID_DIRECTIONS:
        raise ValueError('direction must be 1 or -1')
    return direction


def signed_linear_components(axis, direction, velocity):
    signed_velocity = direction * velocity
    if axis == 'x':
        return signed_velocity, 0.0
    return 0.0, signed_velocity


class OdomDistanceCalculator:
    def __init__(self):
        self._has_start = False
        self._start_x = 0.0
        self._start_y = 0.0

    def set_start(self, x, y):
        self._start_x = x
        self._start_y = y
        self._has_start = True

    def has_start(self):
        return self._has_start

    def displacement_from_start(self, x, y, axis):
        if not self._has_start:
            return 0.0
        if axis == 'x':
            return x - self._start_x
        return y - self._start_y
