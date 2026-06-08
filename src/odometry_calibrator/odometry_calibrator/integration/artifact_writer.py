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
from pathlib import Path


class ArtifactWriter:
    """Manage calibration output artifact paths."""

    def __init__(self, output_dir, result_filename, events_filename):
        self.output_dir = Path(output_dir)
        self.result_filename = result_filename
        self.events_filename = events_filename

    def ensure_output_dir(self):
        self.output_dir.mkdir(parents=True, exist_ok=True)

    @property
    def result_path(self):
        return self.output_dir / self.result_filename

    @property
    def events_path(self):
        return self.output_dir / self.events_filename

    def relative_artifact_path(self, path):
        try:
            return str(Path(path).relative_to(self.output_dir))
        except ValueError:
            return str(Path(path))

    def write_result(self, result):
        self.ensure_output_dir()
        with self.result_path.open('w', encoding='utf-8') as file_obj:
            json.dump(result, file_obj, indent=2, sort_keys=True)
            file_obj.write('\n')
