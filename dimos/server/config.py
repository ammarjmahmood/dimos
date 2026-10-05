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

"""Where the dimos server keeps things, Desktop's config.yaml it shares, and the checkout's info."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
import shutil
from typing import Any

from dimos.constants import STATE_DIR

# what Desktop's dimos.yaml `requires.dimos` says, when Desktop passes it (else every version is in range)
RANGE_ENV = "DESKTOP_DIMOS_RANGE"


def dimos_home() -> Path:
    """Desktop's home: DIMOS_HOME, else ~/.dimos."""
    home = os.environ.get("DIMOS_HOME")
    return Path(home) if home else Path.home() / ".dimos"


def server_dir() -> Path:
    """The server's own state (upload queue, launch record, logs)."""
    return STATE_DIR / "server"


def logs_dir() -> Path:
    return server_dir() / "logs"


def legacy_dir() -> Path:
    """Where Desktop's Rust dimos server kept the same files."""
    return dimos_home() / "desktop"


def state_file(name: str) -> Path:
    """`server_dir()/name`, first copied from Desktop's Rust server when only it has one."""
    path = server_dir() / name
    legacy = legacy_dir() / name
    if not path.exists() and legacy.is_file():
        path.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(legacy, path)
    return path


def write_atomic(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text)
    temporary.replace(path)


def expand(path: str) -> Path:
    return Path(path).expanduser()


def config_file() -> Path:
    return dimos_home() / "config.yaml"


def load_desktop_config() -> dict[str, Any]:
    """Desktop's config.yaml (it owns the file; the server reads `dimos:` and `recordings:`)."""
    try:
        import yaml

        data = yaml.safe_load(config_file().read_text())
    except (OSError, ImportError):
        return {}
    return data if isinstance(data, dict) else {}


def save_desktop_config(config: dict[str, Any]) -> None:
    import yaml

    header = "# dimOS Desktop settings. Edited by Desktop's Settings app too.\n"
    write_atomic(config_file(), header + yaml.safe_dump(config, sort_keys=False))


def _section(config: dict[str, Any], name: str) -> dict[str, Any]:
    value = config.get(name)
    return value if isinstance(value, dict) else {}


def global_config_overrides() -> dict[str, Any]:
    overrides = _section(load_desktop_config(), "dimos").get("global_config")
    return dict(overrides) if isinstance(overrides, dict) else {}


def set_global_config_overrides(overrides: dict[str, Any]) -> None:
    config = load_desktop_config()
    dimos = _section(config, "dimos")
    dimos["global_config"] = {
        key: value for key, value in sorted(overrides.items()) if value is not None
    }
    config["dimos"] = dimos
    save_desktop_config(config)


def ignore_version_range() -> bool:
    return bool(_section(load_desktop_config(), "dimos").get("ignore_version_range"))


def recordings_dir() -> Path:
    configured = _section(load_desktop_config(), "recordings").get("dir")
    return expand(configured) if configured else dimos_home() / "recordings"


def dimos_bin(dimos_dir: Path) -> Path:
    return dimos_dir / ".venv" / "bin" / "dimos"


@dataclass
class Info:
    dir: str
    found: bool
    installed: bool
    version: str | None
    range: str
    in_range: bool

    def to_json(self) -> dict[str, Any]:
        return {
            "dir": self.dir,
            "found": self.found,
            "installed": self.installed,
            "version": self.version,
            "range": self.range,
            "inRange": self.in_range,
        }


def checkout_version(dimos_dir: Path) -> tuple[bool, str | None]:
    """(a dimos checkout is there, its pyproject version)."""
    try:
        text = (dimos_dir / "pyproject.toml").read_text()
    except OSError:
        return False, None
    if 'name = "dimos"' not in text:
        return False, None
    for line in text.splitlines():
        key, equals, value = line.partition("=")
        if equals and key.strip() == "version":
            return True, value.strip().strip('"')
    return True, None


def satisfies(version: str, range_text: str) -> bool:
    """`>=0.0.14b1 <0.1` (Desktop's space-separated form) or comma-separated; prereleases count."""
    from packaging.specifiers import InvalidSpecifier, SpecifierSet
    from packaging.version import InvalidVersion, Version

    try:
        specifiers = SpecifierSet(",".join(range_text.split()), prereleases=True)
        return Version(version) in specifiers
    except (InvalidSpecifier, InvalidVersion):
        return False


def info(dimos_dir: Path) -> Info:
    found, version = checkout_version(dimos_dir)
    range_text = os.environ.get(RANGE_ENV, "")
    in_range = version is not None and (not range_text or satisfies(version, range_text))
    return Info(
        dir=str(dimos_dir),
        found=found,
        installed=found and dimos_bin(dimos_dir).exists(),
        version=version,
        range=range_text,
        in_range=in_range,
    )


def global_config_flags(overrides: dict[str, Any]) -> list[str]:
    """`--key value` flags for dimos's GlobalConfig, as typer reads them."""
    import json

    flags: list[str] = []
    for key, value in sorted(overrides.items()):
        flag = "--" + key.replace("_", "-")
        if value is None:
            continue
        if value is True:
            flags.append(flag)
        elif value is False:
            flags.append("--no-" + key.replace("_", "-"))
        elif isinstance(value, str):
            flags += [flag, value]
        else:
            flags += [flag, json.dumps(value)]
    return flags
