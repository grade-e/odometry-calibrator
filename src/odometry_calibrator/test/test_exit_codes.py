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

from odometry_calibrator.integration import exit_codes


def test_exit_code_contract_values():
    assert exit_codes.SUCCESS == 0
    assert exit_codes.CALIBRATION_FAILED == 1
    assert exit_codes.INVALID_PARAMETER == 2
    assert exit_codes.TOPIC_UNAVAILABLE == 3
    assert exit_codes.MOTION_TIMEOUT == 4
    assert exit_codes.MEASUREMENT_CANCELLED == 5
    assert exit_codes.INTERNAL_ERROR == 10
