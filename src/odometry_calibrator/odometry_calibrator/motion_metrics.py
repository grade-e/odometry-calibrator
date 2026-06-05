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

MIN_DISTANCE_FOR_RATIO = 1.0e-6


def compute_axis_distance(start_x, start_y, x, y, axis, direction):
    """Return positive axis/direction distance."""
    if axis == 'x':
        raw_displacement = x - start_x
    else:
        raw_displacement = y - start_y
    return max(direction * raw_displacement, 0.0)


def compute_remaining_distance(target_distance, axis_distance):
    """Return remaining distance."""
    return target_distance - axis_distance


def safe_ratio(numerator, denominator, min_denominator=MIN_DISTANCE_FOR_RATIO):
    """Return numerator / denominator, or None when the denominator is too small."""
    if abs(denominator) < min_denominator:
        return None
    return numerator / denominator
