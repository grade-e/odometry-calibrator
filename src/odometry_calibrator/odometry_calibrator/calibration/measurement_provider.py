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

from abc import ABC, abstractmethod


class MeasurementProvider(ABC):
    """Interface for actual-distance measurement sources."""

    @abstractmethod
    def start(self):
        """Begin collecting a measurement."""

    @abstractmethod
    def has_measurement(self):
        """Return whether a measurement is available."""

    @abstractmethod
    def get_measurement(self):
        """Return the measured distance in meters."""
