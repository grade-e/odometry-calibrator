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


def validate_positive_finite(name, value):
    if value <= 0.0 or not math.isfinite(value):
        raise ValueError(f'{name} must be positive and finite')


def normalize_reference_pose_topic(use_reference_pose, topic):
    topic = '' if topic is None else str(topic).strip()
    if use_reference_pose and not topic:
        raise ValueError(
            'reference_pose_topic must not be empty when use_reference_pose=true'
        )
    return topic
