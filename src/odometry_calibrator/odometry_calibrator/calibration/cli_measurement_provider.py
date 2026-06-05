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
import sys
import threading
import time

from odometry_calibrator.calibration.measurement_provider import MeasurementProvider


MAX_REASONABLE_MEASUREMENT_METERS = 10.0


class CliMeasurementProvider(MeasurementProvider):
    """Read the actual measured distance from standard input."""

    def __init__(self):
        self._started = False
        self._stop_requested = threading.Event()
        self._has_measurement = threading.Event()
        self._lock = threading.Lock()
        self._measurement = 0.0

    def start(self):
        if self._started:
            return
        self._started = True
        thread = threading.Thread(target=self._input_loop, daemon=True)
        thread.start()

    def stop(self):
        self._stop_requested.set()

    def has_measurement(self):
        return self._has_measurement.is_set()

    def get_measurement(self):
        with self._lock:
            return self._measurement

    @staticmethod
    def parse_measurement(text):
        try:
            value = float(text.strip())
        except ValueError:
            return None

        if math.isfinite(value) and 0.0 < value <= MAX_REASONABLE_MEASUREMENT_METERS:
            return value
        return None

    def _input_loop(self):
        eof_reported = False

        while not self._stop_requested.is_set() and not self._has_measurement.is_set():
            print('Enter actual measured distance [m]: ', end='', flush=True)
            line = sys.stdin.readline()

            if line == '':
                if not eof_reported:
                    print(
                        'No CLI input stream is available. '
                        'Waiting for an input stream to recover.',
                        file=sys.stderr,
                    )
                    eof_reported = True
                time.sleep(1.0)
                continue

            eof_reported = False
            measurement = self.parse_measurement(line)
            if measurement is None:
                print(
                    'Invalid distance. Enter a numeric value in the range '
                    f'0.0 < D_actual <= {MAX_REASONABLE_MEASUREMENT_METERS} meters.',
                    file=sys.stderr,
                )
                continue

            with self._lock:
                self._measurement = measurement
            self._has_measurement.set()
