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
import time


class EventWriter:
    """Write JSON Lines events for external job orchestration."""

    def __init__(self, path):
        self._path = path
        self._start_time = time.monotonic()
        self._file = None

    def _open(self):
        if self._file is not None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self._path.open('w', encoding='utf-8')

    def write(self, event, **fields):
        self._open()
        payload = {
            'time_sec': round(time.monotonic() - self._start_time, 6),
            'event': event,
        }
        payload.update(fields)
        self._file.write(json.dumps(payload, sort_keys=True, separators=(',', ':')))
        self._file.write('\n')
        self._file.flush()

    def state_transition(self, from_state, to_state):
        self.write('state_transition', **{'from': from_state, 'to': to_state})

    def warning(self, message, **fields):
        self.write('warning', message=message, **fields)

    def error(self, message, **fields):
        self.write('error', message=message, **fields)

    def close(self):
        if self._file is not None:
            self._file.close()
            self._file = None
