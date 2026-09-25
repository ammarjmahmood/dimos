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

"""Call the R1 Pro classical skills over MCP and wait for their motion to finish.

Those skills reply at once with ``{"accepted": ...}`` and move in the
background; ``wait_for_action`` then reports ``state``: running, completed,
failed or cancelled.
"""

from __future__ import annotations

import json
import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.agents.mcp.mcp_adapter import McpAdapter

WAIT_S = 20.0  # the longest wait_for_action blocks


def call_json(
    mcp: McpAdapter, tool: str, arguments: dict[str, Any] | None = None
) -> dict[str, Any]:
    """Call a skill and decode its JSON reply; plain-text replies come back as a refusal."""
    text = mcp.call_tool_text(tool, arguments or {})
    try:
        value = json.loads(text)
    except ValueError:
        return {"accepted": False, "reason": text}
    return value if isinstance(value, dict) else {"value": value}


def run_action(
    mcp: McpAdapter, tool: str, arguments: dict[str, Any], *, timeout_s: float
) -> dict[str, Any]:
    """Start a skill and return how it ended.

    Args:
        mcp: client for the robot's MCP server.
        tool: skill name, e.g. ``pick_object``.
        arguments: the skill's arguments.
        timeout_s: seconds to wait for the motion before stopping it; the
            reply then has ``state`` ``timeout``.

    Returns:
        The final action status (``state``, ``success``, ``error``, ...);
        ``state`` is ``refused`` when the skill did not start. Skills that do
        not move, such as ``get_scene``, return their reply unchanged.
    """
    started = call_json(mcp, tool, arguments)
    if "accepted" not in started:
        return started
    if not started["accepted"]:
        return {**started, "state": "refused", "success": False}
    deadline = time.monotonic() + timeout_s
    while True:
        status = call_json(mcp, "wait_for_action", {"seconds": WAIT_S})
        if status.get("state") != "running":
            return status
        if time.monotonic() > deadline:
            call_json(mcp, "stop_action")
            return {**status, "state": "timeout", "success": False}


def action_running(scene: dict[str, Any]) -> bool:
    """Whether a ``get_scene`` reply shows motion still in progress."""
    return bool(scene.get("action", {}).get("state") == "running")
