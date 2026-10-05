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

"""dimos's optional-dependency extras: which there are (the checkout's pyproject.toml, else the installed dimos's
metadata), which are installed in the checkout's python, a download-size hint from uv.lock, and the command that adds
some (scripts/install.sh's: `uv sync` in a checkout, `uv pip install 'dimos[...]'` for a library install).

Which packages are installed is asked of the checkout's own python in a child (`discover packages`), since the server
may run on another one.
"""

from __future__ import annotations

from collections.abc import Awaitable, Callable
import json
import os
from pathlib import Path
import shlex
import shutil
import sys
from typing import Any

from packaging.markers import Marker
from packaging.requirements import InvalidRequirement, Requirement
from packaging.utils import canonicalize_name

from dimos.server.config import venv_dir
from dimos.server.jobs import MissingForJobError

if sys.version_info >= (3, 11):
    import tomllib
else:
    import tomli as tomllib


def is_checkout(dimos_dir: Path) -> bool:
    """A source checkout `uv sync` can install into: dimos's pyproject.toml and a uv.lock."""
    from dimos.server import config

    found, _ = config.checkout_version(dimos_dir)
    return found and (dimos_dir / "uv.lock").is_file()


def declared(dimos_dir: Path, probe: dict[str, Any]) -> dict[str, list[str]]:
    """Extra -> its requirement strings: pyproject.toml's optional-dependencies, else the installed dimos's metadata
    (`probe`, from `discover packages`): each Provides-Extra and the Requires-Dist whose marker holds for that extra and
    not without it, on this machine (packaging evaluates the marker; one for another platform isn't listed)."""
    try:
        project = tomllib.loads((dimos_dir / "pyproject.toml").read_text())["project"]
        if project.get("name") == "dimos":
            return {k: list(v) for k, v in project.get("optional-dependencies", {}).items()}
    except (OSError, KeyError, tomllib.TOMLDecodeError):
        pass
    environment = probe.get("environment", {})
    parsed = []
    for text in probe.get("dimos_requires", []):
        try:
            parsed.append((text, Requirement(text)))
        except InvalidRequirement:
            continue
    return {
        extra: [
            text
            for text, requirement in parsed
            if requirement.marker is not None
            and applies(requirement, environment, extra)
            and not applies(requirement, environment, "")
        ]
        for extra in probe.get("dimos_extras", [])
    }


def applies(requirement: Requirement, environment: dict[str, str], extra: str = "") -> bool:
    if requirement.marker is None:
        return True
    try:
        return bool(requirement.marker.evaluate({**environment, "extra": extra}))
    except Exception:
        return True


def status(dimos_dir: Path, probe: dict[str, Any], lock_sizes: bool = True) -> list[dict[str, Any]]:
    """Each extra: what it requires, which other extras it includes, whether it's installed and what's missing.

    Installed = every requirement that applies on this machine is installed at a version it allows, and every
    included extra is installed. An extra none of whose requirements apply here (cuda on a Mac) is `applicable:
    false` (and counts as installed: there's nothing to add)."""
    environment = probe.get("environment", {})
    packages = probe.get("packages", {})
    extras = declared(dimos_dir, probe)
    parsed: dict[str, tuple[list[Requirement], list[str]]] = {}
    for name, texts in extras.items():
        requirements, includes = [], []
        for text in texts:
            try:
                requirement = Requirement(text)
            except InvalidRequirement:
                continue
            if canonicalize_name(requirement.name) == "dimos":
                includes += sorted(requirement.extras)
            elif applies(requirement, environment, name):
                requirements.append(requirement)
        parsed[name] = (requirements, includes)

    def missing(name: str, seen: frozenset[str] = frozenset()) -> list[str]:
        requirements, includes = parsed.get(name, ([], []))
        found: list[str] = []
        for requirement in requirements:
            version = packages.get(canonicalize_name(requirement.name))
            if version is None or not requirement.specifier.contains(version, prereleases=True):
                found.append(canonicalize_name(requirement.name))
        for include in includes:
            if include not in seen:
                found += [m for m in missing(include, seen | {name}) if m not in found]
        return found

    sizes = lock_download_sizes(dimos_dir, environment, packages) if lock_sizes else {}
    answer = []
    for name in extras:
        requirements, includes = parsed[name]
        lacking = missing(name)
        answer.append(
            {
                "name": name,
                "installed": not lacking,
                "applicable": bool(requirements or includes),
                "requires": extras[name],
                "includes": includes,
                "missing": lacking,
                "download_bytes": sizes.get(name) if lacking else 0,
            }
        )
    return answer


def wheel_sizes(entry: dict[str, Any], environment: dict[str, str]) -> list[int]:
    """The sizes of a uv.lock package's wheels built for this machine's python, OS and CPU (or any)."""
    system = {"Darwin": "macosx", "Linux": "linux"}.get(
        environment.get("platform_system", ""), "win"
    )
    machine = environment.get("platform_machine", "").lower()
    cpus = {"arm64": ("arm64", "universal2"), "aarch64": ("aarch64",)}.get(machine, (machine,))
    python = "cp" + "".join(environment.get("python_version", "3.12").split(".")[:2])
    sizes: list[int] = []
    for wheel in entry.get("wheels", []):
        file = str(wheel.get("url", wheel.get("filename", ""))).rsplit("/", 1)[-1]
        tags = file[: -len(".whl")].split("-")[-3:] if file.endswith(".whl") else []
        if len(tags) != 3 or not isinstance(wheel.get("size"), int):
            continue
        py_ok = tags[0].startswith(("py3", "py2.py3")) or python in tags[0] or tags[1] == "abi3"
        plat_ok = tags[2] == "any" or (system in tags[2] and any(cpu in tags[2] for cpu in cpus))
        if py_ok and plat_ok:
            sizes.append(wheel["size"])
    return sizes


def wheel_size(entry: dict[str, Any], environment: dict[str, str]) -> int | None:
    """A guess at the download for this machine: the largest wheel built for its OS and CPU (or any), else the
    sdist."""
    sizes = wheel_sizes(entry, environment)
    if sizes:
        return max(sizes)
    size = entry.get("sdist", {}).get("size")
    return size if isinstance(size, int) else None


def lock_entry(dimos_dir: Path, name: str) -> dict[str, Any] | None:
    try:
        lock = tomllib.loads((dimos_dir / "uv.lock").read_text())
    except (OSError, tomllib.TOMLDecodeError):
        return None
    return next(
        (e for e in lock.get("package", []) if canonicalize_name(e["name"]) == name),
        None,
    )


def builds_cyclonedds(
    dimos_dir: Path, statuses: list[dict[str, Any]], wanted: list[str], environment: dict[str, str]
) -> bool:
    """Whether installing `wanted` builds the cyclonedds package from source: one of them adds it and uv.lock has no
    wheel of it for this python and machine (0.10.5 has wheels for python 3.10 only). Its build needs the CycloneDDS C
    library (CYCLONEDDS_HOME)."""
    if not any("cyclonedds" in e["missing"] for e in statuses if e["name"] in wanted):
        return False
    entry = lock_entry(dimos_dir, "cyclonedds")
    return entry is None or not wheel_sizes(entry, environment)


def find_nix() -> str | None:
    return shutil.which("nix") or next(
        (
            str(p)
            for p in (
                Path("/nix/var/nix/profiles/default/bin/nix"),
                Path("/run/current-system/sw/bin/nix"),
            )
            if p.is_file()
        ),
        None,
    )


def nixpkgs_ref(dimos_dir: Path) -> str:
    """The nixpkgs dimos's flake.lock pins (its shell's cyclonedds), read from the lock file alone (evaluating the
    flake would copy the whole checkout into the nix store); else the registry's nixpkgs."""
    try:
        lock = json.loads((dimos_dir / "flake.lock").read_text())
        nodes = lock["nodes"]
        locked = nodes[nodes[lock["root"]]["inputs"]["nixpkgs"]]["locked"]
        if locked.get("type") == "github":
            return f"github:{locked['owner']}/{locked['repo']}/{locked['rev']}"
    except (OSError, ValueError, KeyError, TypeError):
        pass
    return "nixpkgs"


# where Homebrew keeps `brew install cyclonedds` (Apple silicon, Intel)
BREWED_CYCLONEDDS = (Path("/opt/homebrew/opt/cyclonedds"), Path("/usr/local/opt/cyclonedds"))


def cyclonedds_env(home: Path) -> dict[str, str]:
    return {
        "CYCLONEDDS_HOME": str(home),
        "CMAKE_PREFIX_PATH": os.pathsep.join(
            p for p in (str(home), os.environ.get("CMAKE_PREFIX_PATH", "")) if p
        ),
    }


async def prepare_cyclonedds(
    dimos_dir: Path, step: Callable[[list[str]], Awaitable[int]]
) -> dict[str, str]:
    """A CycloneDDS C library for the cyclonedds build: $CYCLONEDDS_HOME, else nix's (from the nixpkgs dimos's
    flake.lock pins, the same one `nix develop` gives; its out-link in the venv keeps it from garbage collection, and the
    built package links it by its store path), else Homebrew's. None: MissingForJobError `cyclonedds_missing`."""
    given = os.environ.get("CYCLONEDDS_HOME")
    if given and (Path(given) / "lib").is_dir():
        return cyclonedds_env(Path(given))
    link = venv_dir(dimos_dir) / "cyclonedds"
    nix = find_nix()
    if nix is not None:
        command = [
            nix,
            "--extra-experimental-features",
            "nix-command flakes",
            "build",
            "--out-link",
            str(link),
            f"{nixpkgs_ref(dimos_dir)}#cyclonedds",
        ]
        if await step(command) == 0 and (link / "lib").is_dir():
            return cyclonedds_env(link.resolve())
    for brewed in BREWED_CYCLONEDDS:
        if (brewed / "lib").is_dir():
            return cyclonedds_env(brewed.resolve())
    raise MissingForJobError(
        "cyclonedds_missing",
        "The cyclonedds package is built here against the CycloneDDS C library, and none was found"
        + (" (nix couldn't build it, see above)" if nix else "")
        + ": install nix (Desktop's installer does), or `brew install cyclonedds`, or set CYCLONEDDS_HOME to an"
        " install of CycloneDDS 0.10, then install again.",
    )


def lock_download_sizes(
    dimos_dir: Path, environment: dict[str, str], packages: dict[str, str]
) -> dict[str, int]:
    """Extra -> bytes to download for the packages it would add (uv.lock's wheel sizes; markers it can't read count
    as applying, so it's an upper bound). Empty without a uv.lock."""
    try:
        lock = tomllib.loads((dimos_dir / "uv.lock").read_text())
    except (OSError, tomllib.TOMLDecodeError):
        return {}
    by_name: dict[str, dict[str, Any]] = {}
    for entry in lock.get("package", []):
        by_name.setdefault(canonicalize_name(entry["name"]), entry)
    dimos = by_name.get("dimos")
    if dimos is None:
        return {}

    def wanted(dependency: dict[str, Any]) -> bool:
        marker = dependency.get("marker")
        if not marker:
            return True
        try:
            return bool(Marker(marker).evaluate({**environment, "extra": ""}))
        except Exception:
            return True

    answer = {}
    for extra, dependencies in dimos.get("optional-dependencies", {}).items():
        seen: set[str] = set()
        stack = [d for d in dependencies if wanted(d)]
        while stack:
            dependency = stack.pop()
            name = canonicalize_name(dependency["name"])
            entry = by_name.get(name)
            if entry is None:
                continue
            if name == "dimos":
                for sub in dependency.get("extra", []):
                    stack += [
                        d for d in dimos.get("optional-dependencies", {}).get(sub, []) if wanted(d)
                    ]
                continue
            if name in seen:
                continue
            seen.add(name)
            stack += [d for d in entry.get("dependencies", []) if wanted(d)]
            for sub in dependency.get("extra", []):
                stack += [
                    d for d in entry.get("optional-dependencies", {}).get(sub, []) if wanted(d)
                ]
        answer[extra] = sum(
            wheel_size(by_name[name], environment) or 0 for name in seen if name not in packages
        )
    return answer


def find_uv() -> str | None:
    found = shutil.which("uv")
    if found:
        return found
    for candidate in (Path.home() / ".local/bin/uv", Path.home() / ".cargo/bin/uv"):
        if candidate.is_file():
            return str(candidate)
    return None


def install_command(
    dimos_dir: Path, extras: list[str], python: str, dimos_version: str | None, uv: str
) -> list[str]:
    """scripts/install.sh's command for these extras. In a checkout `--inexact` keeps every extra and group already
    installed (plain `uv sync` removes what isn't asked for)."""
    if is_checkout(dimos_dir):
        command = [uv, "sync", "--locked", "--inexact", "--no-progress"]
        for extra in extras:
            command += ["--extra", extra]
        return command
    backend = "cu128" if "cuda" in extras else "cpu"
    pin = f"=={dimos_version}" if dimos_version else ""
    return [
        uv,
        "pip",
        "install",
        "--no-progress",
        "--python",
        python,
        "--torch-backend",
        backend,
        f"dimos[{','.join(extras)}]{pin}",
    ]


def shell_commands(
    dimos_dir: Path,
    extras: list[str],
    python: str,
    dimos_version: str | None,
    uv: str,
    cyclonedds: bool,
) -> list[dict[str, Any]]:
    """The commands Desktop's shell tool runs for these extras, each with a note for the user: with `cyclonedds`
    (the cyclonedds package builds here) first its C library ($CYCLONEDDS_HOME, else nix's from the nixpkgs dimos's
    flake.lock pins, linked into the venv, else Homebrew's), then install_command. Raises MissingForJobError
    `cyclonedds_missing` when there's no way to get the library."""
    install = shlex.join(install_command(dimos_dir, extras, python, dimos_version, uv))
    env = {"VIRTUAL_ENV": str(venv_dir(dimos_dir))} if is_checkout(dimos_dir) else {}
    commands: list[dict[str, Any]] = []
    if cyclonedds:
        given = os.environ.get("CYCLONEDDS_HOME")
        brewed = next((b for b in BREWED_CYCLONEDDS if (b / "lib").is_dir()), None)
        nix = find_nix()
        if given and (Path(given) / "lib").is_dir():
            env.update(cyclonedds_env(Path(given)))
        elif nix is not None:
            link = venv_dir(dimos_dir) / "cyclonedds"
            build = [
                nix,
                "--extra-experimental-features",
                "nix-command flakes",
                "build",
                "--out-link",
                str(link),
                f"{nixpkgs_ref(dimos_dir)}#cyclonedds",
            ]
            commands.append(
                {
                    "run": shlex.join(build),
                    "note": "Get the CycloneDDS C library (the cyclonedds package builds against it)",
                    "cwd": str(dimos_dir),
                }
            )
            # its store path, resolved when the install runs
            install = (
                f'export CYCLONEDDS_HOME="$(cd {shlex.quote(str(link))} && pwd -P)"\n'
                'export CMAKE_PREFIX_PATH="$CYCLONEDDS_HOME${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"\n'
                + install
            )
        elif brewed is not None:
            env.update(cyclonedds_env(brewed.resolve()))
        else:
            raise MissingForJobError(
                "cyclonedds_missing",
                "The cyclonedds package is built here against the CycloneDDS C library, and none was found: "
                "install nix (Desktop's installer does), or `brew install cyclonedds`, or set CYCLONEDDS_HOME to "
                "an install of CycloneDDS 0.10, then install again.",
            )
    commands.append(
        {
            "run": install,
            "note": f"Install the {', '.join(extras)} extra{'s' if len(extras) > 1 else ''} with uv",
            "cwd": str(dimos_dir),
            "env": env,
        }
    )
    return commands
