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

from odometry_calibrator.calibration.odom_linear_calibrator import (
    compute_target_velocity,
    CONTROL_MODE_CONSTANT,
    CONTROL_MODE_P,
    CONTROL_MODE_P_MIN_CLAMPED,
    CONTROL_MODE_P_STOP_THRESHOLD,
    normalize_control_mode,
)
import pytest


def test_compute_target_velocity_returns_zero_for_non_positive_remaining():
    assert compute_target_velocity(
        CONTROL_MODE_P_MIN_CLAMPED,
        0.0,
        0.4,
        0.01,
        0.1,
    ) == 0.0
    assert compute_target_velocity(
        CONTROL_MODE_P_MIN_CLAMPED,
        -0.1,
        0.4,
        0.01,
        0.1,
    ) == 0.0


def test_constant_control_mode_uses_max_velocity():
    assert compute_target_velocity(CONTROL_MODE_CONSTANT, 0.01, 0.4, 0.05, 0.1) == 0.1


def test_p_control_mode_allows_velocity_below_min_velocity():
    assert compute_target_velocity(
        CONTROL_MODE_P,
        0.01,
        0.4,
        0.05,
        0.1,
    ) == pytest.approx(0.004)


def test_p_min_clamped_control_mode_clamps_to_min_velocity_while_moving():
    assert compute_target_velocity(
        CONTROL_MODE_P_MIN_CLAMPED,
        0.01,
        0.4,
        0.05,
        0.1,
    ) == 0.05


def test_p_stop_threshold_control_mode_returns_zero_below_min_velocity():
    assert compute_target_velocity(
        CONTROL_MODE_P_STOP_THRESHOLD,
        0.01,
        0.4,
        0.05,
        0.1,
    ) == 0.0


def test_p_control_modes_use_proportional_velocity_between_limits():
    assert compute_target_velocity(
        CONTROL_MODE_P,
        0.2,
        0.4,
        0.05,
        0.1,
    ) == pytest.approx(0.08)
    assert compute_target_velocity(
        CONTROL_MODE_P_MIN_CLAMPED,
        0.2,
        0.4,
        0.05,
        0.1,
    ) == pytest.approx(0.08)
    assert compute_target_velocity(
        CONTROL_MODE_P_STOP_THRESHOLD,
        0.2,
        0.4,
        0.05,
        0.1,
    ) == pytest.approx(0.08)


def test_compute_target_velocity_clamps_to_max_velocity():
    assert compute_target_velocity(
        CONTROL_MODE_P_MIN_CLAMPED,
        1.0,
        0.4,
        0.05,
        0.1,
    ) == 0.1


def test_normalize_control_mode_accepts_aliases():
    assert normalize_control_mode('normal') == CONTROL_MODE_CONSTANT
    assert normalize_control_mode(' p_min_velocity ') == CONTROL_MODE_P_MIN_CLAMPED


def test_normalize_control_mode_rejects_unknown_mode():
    with pytest.raises(RuntimeError):
        normalize_control_mode('unknown')
