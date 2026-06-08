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

from odometry_calibrator.calibration.parameter_measurement_provider import (
    ParameterMeasurementProvider,
)
import pytest


def test_parameter_measurement_provider_has_measurement_immediately():
    provider = ParameterMeasurementProvider(0.985)

    provider.start()

    assert provider.has_measurement()
    assert provider.get_measurement() == pytest.approx(0.985)


@pytest.mark.parametrize('value', [0.0, -1.0, 11.0, float('nan'), float('inf'), 'abc'])
def test_parameter_measurement_provider_rejects_invalid_measurements(value):
    with pytest.raises(ValueError):
        ParameterMeasurementProvider(value)
