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

TASK_FACTORIES = {
    "m20_locomotion": "dimos.control.tasks.m20_locomotion_task.m20_locomotion_task:create_task",
}
TASK_CONSUMES = {"m20_locomotion": {"twist_command": ("on_twist_command", "broadcast")}}
TASK_EXPOSES = {"m20_locomotion": ["start", "stop", "reset_runtime_state"]}
