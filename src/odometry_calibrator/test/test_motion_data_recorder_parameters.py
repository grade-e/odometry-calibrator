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

from odometry_calibrator.common.parameters import normalize_reference_pose_topic
import pytest


def test_reference_pose_topic_allows_empty_topic_when_disabled():
    assert normalize_reference_pose_topic(False, '') == ''


def test_reference_pose_topic_accepts_topic_when_enabled():
    assert normalize_reference_pose_topic(True, '/reference_pose') == '/reference_pose'


def test_reference_pose_topic_strips_whitespace_when_enabled():
    assert normalize_reference_pose_topic(True, '  /reference_pose  ') == '/reference_pose'


@pytest.mark.parametrize('topic', ['', '   '])
def test_reference_pose_topic_rejects_empty_topic_when_enabled(topic):
    with pytest.raises(ValueError):
        normalize_reference_pose_topic(True, topic)
