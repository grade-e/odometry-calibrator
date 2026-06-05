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

from odometry_calibrator.cli_measurement_provider import CliMeasurementProvider
import pytest


def test_parse_measurement_accepts_valid_distances():
    assert CliMeasurementProvider.parse_measurement('1.0') == 1.0
    assert CliMeasurementProvider.parse_measurement('0.985') == 0.985


@pytest.mark.parametrize('value', ['abc', '-1.0', '0', '', 'nan', 'inf', '11.0'])
def test_parse_measurement_rejects_invalid_distances(value):
    assert CliMeasurementProvider.parse_measurement(value) is None
