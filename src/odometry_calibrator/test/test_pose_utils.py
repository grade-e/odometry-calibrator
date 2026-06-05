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

from types import SimpleNamespace

from odometry_calibrator.pose_utils import distance_2d
from odometry_calibrator.pose_utils import yaw_from_quaternion


def test_yaw_from_identity_quaternion_is_zero():
    quaternion = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)

    assert yaw_from_quaternion(quaternion) == 0.0


def test_distance_2d_returns_euclidean_distance():
    assert distance_2d(0.0, 0.0, 3.0, 4.0) == 5.0
