# Copyright 2026 Dimensional Inc.
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


from __future__ import annotations

import functools

from turbojpeg import TurboJPEG


@functools.cache
def get_turbojpeg() -> TurboJPEG:
    """Return the shared TurboJPEG codec handle."""
    return TurboJPEG()


@functools.cache
def turbojpeg_available() -> bool:
    """Whether the libturbojpeg shared library can be loaded on this machine."""
    try:
        get_turbojpeg()
    except RuntimeError:
        return False
    return True
