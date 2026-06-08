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

import json

from odometry_calibrator.integration.artifact_writer import ArtifactWriter
from odometry_calibrator.integration.event_writer import EventWriter


def test_event_writer_records_json_lines(tmp_path):
    event_path = tmp_path / 'events.jsonl'
    writer = EventWriter(event_path)

    writer.write('started', axis='x', direction=1)
    writer.state_transition('INIT', 'MOVING_ACCEL_LIMIT')
    writer.close()

    lines = event_path.read_text(encoding='utf-8').splitlines()
    events = [json.loads(line) for line in lines]

    assert events[0]['event'] == 'started'
    assert events[0]['axis'] == 'x'
    assert events[1]['event'] == 'state_transition'
    assert events[1]['from'] == 'INIT'
    assert events[1]['to'] == 'MOVING_ACCEL_LIMIT'


def test_artifact_writer_creates_output_dir_and_writes_result(tmp_path):
    output_dir = tmp_path / 'missing' / 'artifacts'
    writer = ArtifactWriter(output_dir, 'result.json', 'events.jsonl')

    writer.write_result({'status': 'success'})

    assert output_dir.exists()
    assert json.loads(writer.result_path.read_text(encoding='utf-8')) == {
        'status': 'success',
    }
    assert writer.relative_artifact_path(writer.events_path) == 'events.jsonl'
