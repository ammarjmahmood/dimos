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

"""R1 Pro open-space tasks: what to ask, a skill plan that does it, and how to grade it.

A task is written for one seeded layout: which object starts on which platform.
Grading reads ``scene_truth.jsonl``, the simulator's own view of every object
(which hand holds it, what it rests on, whether it is in the tray) sampled
about once a second while the agent works.

Milestones must be reached in order, anywhere in the run; final checks must
hold in the last sample. The score is the fraction of both that passed.
"""

from __future__ import annotations

from collections.abc import Callable, Sequence
from dataclasses import dataclass, field
import json
import math
import os
from pathlib import Path
from typing import Any

from dimos.evals.environments.r1pro_scene import R1ProScene
from dimos.evals.types import EvalCase, Grade, Outcome

Sample = dict[str, Any]  # one get_scene reply, plus the wall time ``t`` it was read
ARMS = ("left", "right")
# The carried tray clears only platforms below about 80 cm.
TRAY_PLATFORMS = ("worktable", "low_bench", "display_table")
NEAR_PLATFORM_M = 1.4  # base centre to platform centre after go_to docks beside it
# Set to 1 to watch cases in the MuJoCo window instead of running headless.
VIEWER_ENV = "R1PRO_EVAL_VIEWER"
# The blueprint and simulator class of each scene, as dimos run knows them.
SCENES = {
    "open_space": ("r1pro-classical-open-space-sim", "R1ProOpenSpaceSim"),
    "apartment": ("r1pro-classical-apartment-sim", "R1ProClassicalSim"),
}


@dataclass(frozen=True, kw_only=True)
class SceneObject:
    id: str  # simulator instance ID, e.g. "object_3"
    kind: str  # e.g. "toy_block"
    color: str  # the name get_scene reports, e.g. "orange"
    platform: str  # where it starts

    @property
    def name(self) -> str:
        """How a person would say it: "orange toy block"."""
        return f"{self.color} {self.kind.replace('_', ' ')}"


@dataclass(frozen=True, kw_only=True)
class Platform:
    name: str
    xy: tuple[float, float]  # centre of the top, world metres
    height_m: float


@dataclass(frozen=True, kw_only=True)
class Layout:
    """One seed of the open-space scene: five objects, five platforms, the tray."""

    seed: int
    objects: tuple[SceneObject, ...]
    platforms: tuple[Platform, ...]
    tray_platform: str = "worktable"
    scene: str = "open_space"  # or "apartment"

    def on(self, platform: str) -> SceneObject:
        return next(o for o in self.objects if o.platform == platform)

    def platform(self, name: str) -> Platform:
        return next(p for p in self.platforms if p.name == name)

    def expected_objects(self) -> dict[str, dict[str, str]]:
        """What the live scene must show before the agent starts."""
        return {o.id: {"kind": o.kind, "color": o.color, "on": o.platform} for o in self.objects}

    @classmethod
    def from_json(cls, data: dict[str, Any]) -> Layout:
        return cls(
            seed=int(data["seed"]),
            objects=tuple(SceneObject(**o) for o in data["objects"]),
            platforms=tuple(
                Platform(name=p["name"], xy=tuple(p["xy"]), height_m=p["height_m"])  # type: ignore[arg-type]
                for p in data["platforms"]
            ),
            tray_platform=data.get("tray_platform", "worktable"),
            scene=data.get("scene", "open_space"),
        )


@dataclass(frozen=True, kw_only=True)
class Check:
    label: str
    test: Callable[[Sample], bool]


@dataclass(frozen=True, kw_only=True)
class Task:
    id: str
    instruction: str
    # Skill calls that do the task; only the scripted-plan agent reads them.
    plan: tuple[dict[str, Any], ...]
    milestones: tuple[Check, ...]  # reached in this order, anywhere in the run
    final: tuple[Check, ...]  # true in the last sample
    timeout_s: float
    tags: frozenset[str] = field(default_factory=frozenset)


# -- checks over one sample ------------------------------------------------------


def _row(sample: Sample, obj: SceneObject) -> dict[str, Any]:
    for row in sample["objects"]:
        if row["id"] == obj.id:
            return row  # type: ignore[no-any-return]
    raise KeyError(obj.id)


def _held_by(sample: Sample, obj: SceneObject) -> str | None:
    return next((arm for arm, held in sample["held_objects"].items() if held == obj.id), None)


def held(obj: SceneObject, arm: str | None = None) -> Check:
    """The object is in ``arm`` (either hand when None)."""
    which = f"{arm} hand" if arm else "a hand"
    return Check(
        label=f"{obj.name} in {which}",
        test=lambda s: _held_by(s, obj) == arm if arm else _held_by(s, obj) is not None,
    )


def released(obj: SceneObject) -> Check:
    """Out of both hands and resting on something (a platform or the tray)."""
    return Check(
        label=f"{obj.name} put down",
        test=lambda s: _held_by(s, obj) is None and _row(s, obj)["on"] is not None,
    )


def resting_on(obj: SceneObject, platform: str) -> Check:
    return Check(
        label=f"{obj.name} upright on {platform}",
        test=lambda s: (
            _held_by(s, obj) is None
            and _row(s, obj)["on"] == platform
            and bool(_row(s, obj)["upright"])
        ),
    )


def in_tray(*objs: SceneObject, at_least: int | None = None) -> Check:
    """``at_least`` of the objects (all by default) inside the tray and out of the hands."""
    need = len(objs) if at_least is None else at_least
    names = " and ".join(o.name for o in objs)
    label = f"{names} in the tray" if need == len(objs) else f"{need} of {names} in the tray"
    return Check(
        label=label,
        test=lambda s: (sum(_row(s, o)["inside"] and _held_by(s, o) is None for o in objs) >= need),
    )


def tray_held() -> Check:
    return Check(label="tray lifted in both hands", test=lambda s: bool(s["tray"]["held"]))


def tray_on(platform: str) -> Check:
    return Check(
        label=f"tray set down on {platform}",
        test=lambda s: not s["tray"]["held"] and s["tray"]["station"] == platform,
    )


def hands_free() -> Check:
    return Check(
        label="both hands empty",
        test=lambda s: not s["tray"]["held"] and not any(s["held_objects"].values()),
    )


def base_near(platform: Platform) -> Check:
    x, y = platform.xy
    return Check(
        label=f"robot beside {platform.name}",
        test=lambda s: math.hypot(s["base_pose"][0] - x, s["base_pose"][1] - y) <= NEAR_PLATFORM_M,
    )


# -- grading ---------------------------------------------------------------------


def read_truth(outcome: Outcome) -> list[Sample]:
    """The ground-truth samples the environment wrote while the agent worked."""
    path = Path(outcome.artifacts["scene_truth"])
    return [json.loads(line) for line in path.read_text().splitlines() if line.strip()]


def grade_samples(task: Task, samples: Sequence[Sample]) -> Grade:
    """Score milestones in order, then the final checks on the last sample."""
    if not samples:
        return Grade(score=0.0, details={"error": "no ground-truth samples"})
    t0 = float(samples[0]["t"])
    reached: list[dict[str, Any]] = []
    k = 0
    for sample in samples:
        # One sample can complete several milestones when actions finish between reads.
        while k < len(task.milestones) and _safe(task.milestones[k], sample):
            reached.append(
                {"milestone": task.milestones[k].label, "at_s": round(float(sample["t"]) - t0, 1)}
            )
            k += 1
    last = samples[-1]
    held_at_end = [_safe(c, last) for c in task.final]
    finals = [
        {"check": c.label, "passed": ok} for c, ok in zip(task.final, held_at_end, strict=True)
    ]
    passed = len(reached) + sum(held_at_end)
    total = len(task.milestones) + len(task.final)
    details: dict[str, Any] = {
        "milestones_reached": len(reached),
        "milestones_total": len(task.milestones),
        "reached": reached,
        "missed": [c.label for c in task.milestones[k:]],
        "final": finals,
        "dropped": _dropped(last),
        "samples": len(samples),
        "duration_s": round(float(last["t"]) - t0, 1),
        "sim_speed": sim_speed(samples),
    }
    return Grade(score=passed / total if total else 0.0, details=details)


def sim_speed(samples: Sequence[Sample], window_s: float = 20.0) -> dict[str, float | None]:
    """Simulated seconds per wall second: over the run, and in its slowest ``window_s``.

    The controllers run on wall time, so a slow stretch (0.3 means physics ran
    at 30% of real time) can fail docking or tracking checks that pass at 1.0.
    """
    timed = [
        (float(s["t"]), float(s["sim_time"])) for s in samples if s.get("sim_time") is not None
    ]
    if len(timed) < 2 or timed[-1][0] <= timed[0][0]:
        return {"mean": None, "slowest": None}
    mean = (timed[-1][1] - timed[0][1]) / (timed[-1][0] - timed[0][0])
    slowest: float | None = None
    j = 0
    for i, (wall, sim) in enumerate(timed):
        j = max(j, i + 1)
        while j < len(timed) and timed[j][0] - wall < window_s:
            j += 1
        if j == len(timed):
            break
        rate = (timed[j][1] - sim) / (timed[j][0] - wall)
        slowest = rate if slowest is None else min(slowest, rate)
    return {"mean": round(mean, 2), "slowest": None if slowest is None else round(slowest, 2)}


def grader(task: Task) -> Callable[[Outcome], Grade]:
    def grade(outcome: Outcome) -> Grade:
        return grade_samples(task, read_truth(outcome))

    return grade


def _safe(check: Check, sample: Sample) -> bool:
    try:
        return bool(check.test(sample))
    except (KeyError, TypeError, IndexError):
        return False  # a sample taken mid-reset or with the object missing proves nothing


def _dropped(sample: Sample) -> list[str]:
    """Objects out of the hands but resting on no platform and not in the tray: on the floor."""
    held_ids = {v for v in sample["held_objects"].values() if v}
    return [
        r["id"]
        for r in sample["objects"]
        if r["id"] not in held_ids and r["on"] is None and not r["inside"]
    ]


# -- tasks -----------------------------------------------------------------------


def _step(tool: str, **arguments: str) -> dict[str, Any]:
    return {"tool": tool, "arguments": arguments}


def say(platform: str) -> str:
    """A platform as a person names it: "low bench", "kitchen counter"."""
    return {"kitchen": "kitchen counter"}.get(platform, platform.replace("_", " "))


def go_to_task(layout: Layout, platform: str) -> Task:
    target = layout.platform(platform)
    return Task(
        id=f"s{layout.seed}_go_to_{platform}",
        instruction=f"Go to the {say(platform)}.",
        plan=(_step("go_to", destination=platform),),
        milestones=(base_near(target),),
        final=(base_near(target),),
        timeout_s=600.0,
        tags=frozenset({"nav", "smoke"}),
    )


def pick_task(layout: Layout, platform: str, arm: str) -> Task:
    """Go to a platform and pick its object with one named hand; hold it."""
    obj = layout.on(platform)
    where = say(platform)
    return Task(
        id=f"s{layout.seed}_pick_{obj.kind}_{arm}",
        instruction=f"Go to the {where} and pick up the {obj.name} with your {arm} hand.",
        plan=(
            _step("go_to", destination=platform),
            _step("pick_object", object=obj.id, arm=arm),
        ),
        milestones=(held(obj, arm),),
        final=(held(obj, arm),),
        timeout_s=900.0,
        tags=frozenset({"pick"}),
    )


def move_task(layout: Layout, source: str, destination: str) -> Task:
    """Carry one object from one platform to another and put it down."""
    obj = layout.on(source)
    return Task(
        id=f"s{layout.seed}_move_{obj.kind}_to_{destination}",
        instruction=(
            f"Take the {obj.name} from the {say(source)} "
            f"to the {say(destination)} and put it down there."
        ),
        plan=(
            _step("go_to", destination=source),
            _step("pick_object", object=obj.id, arm="auto"),
            _step("go_to", destination=destination),
            _step("place_object", region=destination, arm="auto"),
        ),
        milestones=(held(obj), resting_on(obj, destination)),
        final=(resting_on(obj, destination), hands_free()),
        timeout_s=1500.0,
        tags=frozenset({"pick", "place"}),
    )


def occupied_hand_task(layout: Layout, first: str, second: str) -> Task:
    """Ask for a second pick with a hand that is already full: the agent must free it first."""
    a, b = layout.on(first), layout.on(second)
    return Task(
        id=f"s{layout.seed}_occupied_right_hand",
        instruction=(
            f"Pick up the {a.name} on the {say(first)} with your right hand "
            f"and bring it to the {say(second)}. Then pick up the {b.name} "
            "there with your right hand."
        ),
        plan=(
            _step("go_to", destination=first),
            _step("pick_object", object=a.id, arm="right"),
            _step("go_to", destination=second),
            _step("place_object", region=second, arm="right"),
            _step("pick_object", object=b.id, arm="right"),
        ),
        milestones=(held(a, "right"), released(a), held(b, "right")),
        final=(held(b, "right"), released(a)),
        timeout_s=1800.0,
        tags=frozenset({"pick", "place", "reasoning"}),
    )


def tray_task(layout: Layout, destination: str | None) -> Task:
    """Put the tray table's object in the tray, then carry the tray to ``destination``.

    With no destination (the apartment, where the tray fits only on the
    worktable) the tray stays where it is.
    """
    obj = layout.on(layout.tray_platform)
    if destination is None:
        return Task(
            id=f"s{layout.seed}_{obj.kind}_into_tray",
            instruction=f"Put the {obj.name} into the tray.",
            plan=(
                _step("pick_object", object=obj.id, arm="auto"),
                _step("place_object", region="tray", arm="auto"),
            ),
            milestones=(held(obj), in_tray(obj)),
            final=(in_tray(obj), tray_on(layout.tray_platform), hands_free()),
            timeout_s=1200.0,
            tags=frozenset({"pick", "place", "tray"}),
        )
    return Task(
        id=f"s{layout.seed}_tray_to_{destination}",
        instruction=(
            f"Put the {obj.name} into the tray, then carry the tray to the "
            f"{say(destination)} and put it down there."
        ),
        plan=(
            _step("pick_object", object=obj.id, arm="auto"),
            _step("place_object", region="tray", arm="auto"),
            _step("pick_up_tray"),
            _step("put_down_tray", region=destination),
        ),
        milestones=(held(obj), in_tray(obj), tray_held(), tray_on(destination)),
        final=(tray_on(destination), in_tray(obj), hands_free()),
        timeout_s=1800.0,
        tags=frozenset({"pick", "place", "tray"}),
    )


def roles(layout: Layout) -> dict[str, str | None]:
    """Which platform plays which part in the long task.

    Y gives the left-hand object, Z the right-hand one, W is where the hands
    swap, D is where the tray ends up (only platforms the carried tray clears;
    None in the apartment, where the tray fits only on the worktable). In the
    open space the seed rotates the parts over the four platforms without the
    tray; the apartment loops dining table, kitchen, worktable.
    """
    if layout.scene == "apartment":
        return {"Y": "dining_table", "Z": "kitchen", "W": layout.tray_platform, "D": None}
    others = sorted(p.name for p in layout.platforms if p.name != layout.tray_platform)
    order = [others[(layout.seed + i) % len(others)] for i in range(len(others))]
    y, z, w = order[:3]
    d = next(p for p in ("low_bench", "display_table") if p != w)
    return {"Y": y, "Z": z, "W": w, "D": d}


def full_task(layout: Layout) -> Task:
    """The long task: two picks, a forced hand swap, loading the tray and carrying it.

    The instruction asks for a right-hand pick while the right hand is full and
    never says to put that item down: the agent has to work it out.
    """
    r = roles(layout)
    y, z, w, d = str(r["Y"]), str(r["Z"]), str(r["W"]), r["D"]
    a, b, c = layout.on(y), layout.on(z), layout.on(w)

    instruction = (
        f"Go to the {say(y)} and pick up the {a.name} with your left hand. "
        f"Then go to the {say(z)} and pick up the {b.name} with your right hand. "
        f"Then go to the {say(w)} and put down what is in your left hand there. "
        f"Pick up the {c.name} with your right hand, then pick up the {b.name} again "
        "with your left hand. Bring both items to the tray and put them in it"
        + (f", then carry the tray to the {say(d)} and put it down there." if d else ".")
    )
    plan = (
        _step("go_to", destination=y),
        _step("pick_object", object=a.id, arm="left"),
        _step("go_to", destination=z),
        _step("pick_object", object=b.id, arm="right"),
        _step("go_to", destination=w),
        _step("place_object", region=w, arm="left"),
        _step("place_object", region=w, arm="right"),
        _step("pick_object", object=c.id, arm="right"),
        _step("pick_object", object=b.id, arm="left"),
        _step("place_object", region="tray", arm="right"),
        _step("place_object", region="tray", arm="left"),
        *((_step("pick_up_tray"), _step("put_down_tray", region=d)) if d else ()),
    )
    tray_at = d or layout.tray_platform
    milestones = (
        held(a, "left"),
        held(b, "right"),
        resting_on(a, w),
        released(b),
        held(c, "right"),
        held(b, "left"),
        in_tray(b, c, at_least=1),
        in_tray(b, c),
        *((tray_held(), tray_on(d)) if d else ()),
    )
    final = (tray_on(tray_at), in_tray(b, c), resting_on(a, w), hands_free())
    return Task(
        id=f"s{layout.seed}_full_" + ("tray_delivery" if d else "tray_loading"),
        instruction=instruction,
        plan=plan,
        milestones=milestones,
        final=final,
        timeout_s=5400.0,
        tags=frozenset({"pick", "place", "tray", "reasoning", "long_horizon"}),
    )


def curriculum(layout: Layout) -> list[Task]:
    """From one skill to the whole job, so a failure shows which piece broke."""
    r = roles(layout)
    y, z, w = str(r["Y"]), str(r["Z"]), str(r["W"])
    return [
        go_to_task(layout, z if layout.scene == "apartment" else y),
        pick_task(layout, y, "left"),
        move_task(layout, z, w),
        occupied_hand_task(layout, z, w),
        tray_task(layout, r["D"]),
        full_task(layout),
    ]


def load_layouts(path: Path) -> list[Layout]:
    return [Layout.from_json(d) for d in json.loads(path.read_text())["layouts"]]


def eval_case(layout: Layout, task: Task, *, headless: bool | None = None) -> EvalCase:
    """The task as a case in its seeded scene; apartment case IDs start with ``apt_``.

    ``headless`` None opens the MuJoCo window only when ``R1PRO_EVAL_VIEWER=1``.
    """
    blueprint, sim_module = SCENES[layout.scene]
    if headless is None:
        headless = os.environ.get(VIEWER_ENV) != "1"
    return EvalCase(
        id=("apt_" if layout.scene == "apartment" else "") + task.id,
        inputs=task.instruction,
        environment=R1ProScene(
            blueprint=[blueprint],
            sim_module=sim_module,
            seed=layout.seed,
            expected_objects=layout.expected_objects(),
            reference_plan=list(task.plan),
            headless=headless,
        ),
        grade=grader(task),
        timeout_s=task.timeout_s,
        tags=frozenset(
            {"r1pro", "mujoco", "manipulation", layout.scene, f"seed{layout.seed}", *task.tags}
        ),
    )
