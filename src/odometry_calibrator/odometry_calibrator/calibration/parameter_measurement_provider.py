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

import math

from odometry_calibrator.calibration.cli_measurement_provider import (
    MAX_REASONABLE_MEASUREMENT_METERS,
)
from odometry_calibrator.calibration.measurement_provider import MeasurementProvider


class ParameterMeasurementProvider(MeasurementProvider):
    """Provide an actual-distance measurement from a ROS parameter value."""

    def __init__(self, actual_distance_m):
        if not self.is_valid_measurement(actual_distance_m):
            raise ValueError(
                'actual_distance_m must be finite and in the range '
                f'0.0 < actual_distance_m <= {MAX_REASONABLE_MEASUREMENT_METERS}'
            )
        self._actual_distance_m = float(actual_distance_m)

    def start(self):
        pass

    def has_measurement(self):
        return True

    def get_measurement(self):
        return self._actual_distance_m

    @staticmethod
    def is_valid_measurement(value):
        try:
            measurement = float(value)
        except (TypeError, ValueError):
            return False
        return (
            math.isfinite(measurement) and
            0.0 < measurement <= MAX_REASONABLE_MEASUREMENT_METERS
        )
