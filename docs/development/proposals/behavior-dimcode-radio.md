# Dimcode Python SDK radio development pilot

The stationary SDK baseline completed official `turning_on_radio` instance 0 in
`house_double_floor_lower`. This used a custom near-field spawn and privileged
geometry. It proves the motion/contact plumbing; it does not establish autonomous
perception, planning, or fair benchmark performance. The successful code revision
is `69c44485baf66f7fe2495712ca2e3eaad2d1e96d`; the independent integration base
remains PR #4342 (`2e09b03c1125c101441d5e16248771177969a9da`).

## Existing interfaces

Dimcode `0.1.0-next.7` uses Pi `0.85.1`, with `write`, `edit`, `read`, and `bash`
tools. Its Bash spawn hook prepends the configured Python interpreter directory
to PATH. A policy can write Python and import `dimos.manipulation.sdk.Arm`.
No MCP endpoint is required; the previous fixture used `mcp: []`.

`Arm.from_app(app, group="left_arm", instance_name="ManipulationModule")` selects
an existing motion module. `state()` and `pose()` return robot feedback;
`move_pose`, `move_linear`, and `set_gripper_position` issue SDK actions.
`Dimos.connect(timeout=5)` connects to the configured transport bus. It has no
`run_id` argument: configure and verify the intended single bus before attaching,
and select the module instance explicitly. Closing a connected app does not stop
the remote runtime.

`BehaviorProbe.observation()` exposes timestamped RGB/depth, calibration, joint
state, and robot TF, with optional wrist-camera streams. It contains no BDDL goal
or task-object pose dictionary. Streams are asynchronous. The current simulator
odometry is perfect simulator localization, so world-frame SDK feedback carries
that development assumption; it is not a validated realistic localization model.

## First real-model invocation: no simulator or motion

The installed isolated Node/Pi runtime exists, but the inspected desktop has no
real Dimcode provider configuration. The only configured provider is the local
fixture. Neither default Dimcode auth location exists, and no inspected provider
key environment variable is present. Credential files were not read. Select an
authorized provider/model and approve the call budget before sending a request.
Use Dimcode's masked login locally; never paste credentials into the agent prompt
or copy another application's credential store. Do not invent a public model ID
from the internal GPT-6.1 Sol model label.

Configure an explicit scratch workspace and the existing Python interpreter,
with `mcp: []`. Launch that dedicated gateway with
`PYTHONPATH=/tmp/behavior-validation.oNeVq8` so imports resolve to the
tested child snapshot, not the interpreter's older editable checkout. Use `setup --skip-dimos` to avoid starting another installation.
The first prompt should ask the actual model to write `sdk_check.py`, execute it,
and interpret its stdout:

```python
import inspect
import json
import sys
from dimos.manipulation.sdk import Arm

print(json.dumps({
    "sdk": Arm.__module__,
    "sdk_file": inspect.getfile(Arm),
    "python": sys.executable,
    "move_pose": str(inspect.signature(Arm.move_pose)),
    "move_linear": str(inspect.signature(Arm.move_linear)),
}))
```

Allow at most one prompt, two tool invocations (write and bash), three model
responses, and 60 seconds overall. The Bash command uses the explicit configured
Python interpreter, with its own 20-second timeout. Capture tool calls, generated
file hash, interpreter/import location, actual stdout, and provider-reported token
usage/cost. These are proposed limits, not a claim that `dimcode run` implements
a turn-count flag. A controller must enforce them using Pi/gateway events.

Dimcode's gateway persists when a CLI detaches. A shell timeout around
`dimcode run` alone is insufficient: on a deadline, send the supported gateway
`abort` command for the owned session, wait for idle, and stop only the pilot's
own dedicated gateway if needed. Model-turn cancellation does not confirm that
a robot action has stopped. The first pilot has no robot action to cancel.

This tests actual-model code-as-policy tool use, repeating a known SDK script.
It does not demonstrate observation-driven planning or task completion.

## Next interaction, after provider and pilot review

A read-only live example, only on the supervisor-selected development bus:

```python
from dimos.porcelain.dimos import Dimos
from dimos.manipulation.sdk import Arm

app = Dimos.connect(timeout=5)
try:
    arm = Arm.from_app(app, group="left_arm", instance_name="ManipulationModule")
    print(arm.state().joints)
    observation = app.get_module("BehaviorProbe").observation()
    image = observation["color_image"]
    print({"timestamp": image.ts, "frame": image.frame_id, "shape": image.data.shape})
finally:
    app.stop()  # Disconnect this client, not the simulation owner.
```

For a later physical policy, the model should produce an action intent (approach,
end-effector pose, press displacement, gripper target) from permitted observations.
The existing SDK/controllers realize that intent. Preserve the conceptual
intent -> action intent -> embodied execution flow; these are not rigid module
assignments. The official BDDL checker remains evaluator-only.

There is one concrete adapter gap before that physical agent pilot: the current
`DevelopmentRadioMotion` client reads object truth to validate a fixed scene.
Run that checker under the development supervisor, and return only allowed
robot/sensor feedback and motion/error/cancellation results to the policy.
The policy must not run `demo_radio --development-diagnostics` or receive the
successful evaluator report as context. Do not supply `BehaviorConnection` truth
or symbolic primitive methods. Direct `Arm.move_pose` against the currently
validation-enabled coordinator is not enough; it would lack the required checked
plan authorization. A minimal task-local module/RPC adapter can preserve the
same stored plan ID and command-digest checks using the existing transport;
this adapter is proposed, not implemented.

Generic filesystem/process/network isolation and cheating audit remain deferred.
With Pi Bash access, prompt instructions and a narrow Python import list do not
prevent introspection or access to other modules/files. This development pilot
must not claim an enforced fairness boundary.

## Oracle inputs that remain

- Custom initial base `[3.6, 4.15, 0.005]`, yaw pi/2, selected for reachability.
- Asset identity, toggle-marker location, and radio/support poses/bounding boxes.
- Manual leading-fingertip contact estimate and 5.3 mm model-height calibration.
- Fixed scene checking from privileged pose snapshots and perfect simulator base
  localization used by world-frame FK.
- Diagnostic contact flags and BDDL results in supervisor reports, not policy input.

Robot joint feedback, robot kinematics/collision meshes, camera calibration, RGB,
depth, and robot TF are distinct from task-object/evaluator truth. Any deliberate
oracle action intent given to the model must retain its provenance and be labeled
script reproduction. Agent value requires a separate observation-driven change
of instruction, target, or recovery decision; another replay is not that evidence.

## Completed real-model CPU pilot

The authorized pilot used the public OpenAI API model `gpt-6-luna` through the
existing official Dimcode gateway. The pinned Pi catalog lacked Luna, so a
scratch, session-local extension registered verified model metadata using Pi's
supported provider API. No package upgrade or fallback model was used.

The actual model wrote `sdk_check.py`, invoked the specified Python interpreter,
and interpreted the resulting stdout. The SDK resolved to
`/tmp/behavior-validation.oNeVq8/dimos/manipulation/sdk.py`, confirming the child
snapshot import rather than the older editable checkout. Both tool calls
succeeded; the third model response correctly described the SDK signatures and
reported that no motion method was called. This reproduces the supplied script;
it demonstrates real-model code execution and feedback, not autonomous task
planning or physical agent success.

The pilot finished idle in 10.04 seconds with exactly two tool invocations and
three model responses. A native provider wrapper enforced at most three HTTP
requests, no retries, 1,024 output tokens per request, standard service tier,
no hosted tools, a 120,000-byte payload limit, and a 60-second deadline. A
credential-free offline fetch-stub preflight verified the output/model limits
and rejection of a fourth request before any network call. Tool hooks permitted
only the supplied SDK introspection write and its bounded Python command.

The public model/pricing source is
<https://developers.openai.com/api/docs/models>: input $0.10 / million tokens,
output $0.50 / million tokens, context 1.05M tokens. Even charging all three
requests for the full context window plus their output cap gives a conservative
$0.316536 ceiling. Pi's cost metadata estimated $0.0016794 for the actual pilot;
that estimate is not a billing invoice and conservatively prices cached input at
the full input rate.

The safe trace, request-bound records, generated script, and offline preflight
are preserved in the task workspace's `dimcode-pilot/` directory. The credential
file was never read or copied by the agent. The user-managed gateway remains
running; this pilot session is idle. Its existing Bash environment inheritance
still provides no enforced credential or evaluator isolation. No simulator,
sensor, robot motion, model installation, or dataset operation occurred.

## Prepared supervisor adapter and next-stage proposal (not run)

`radio_blueprint(..., policy_supervisor=True)` now adds `RadioPolicyModule` and
opts into render-only hiding of OmniGibson's diagnostic toggle markers. Default
SDK baseline behavior remains unchanged. The owner initializes the module with
only the protected collision scene via `initialize_development_scene`; the
policy receives `RadioPolicy.from_app(app)`, not that initialization data.
`RadioPolicy` uses the existing Python SDK and DimOS module RPC, without MCP.
It exposes `observe`, `ground`, `state`, `pose`, `move_pose`, `press`, `status`,
and `cancel`. The module's owner initialization remains discoverable through
ordinary Python/DimOS introspection: this is a supported-interface boundary,
not the deferred security sandbox.

Observations contain only actual head or left-wrist RGB, optical-Z depth,
intrinsics, camera-to-base robot TF, and an observation ID. Robot state exposes
selected-arm joints, gripper feedback and base-frame encoder/FK pose. World
odometry, object poses/identities, segmentation labels, contact flags, toggle
state, BDDL results and spectator-camera images are absent. RGB/depth/calibration
must have matching frames/dimensions, <=100 ms timestamp skew and <=1 second age.
Grounding retains the exact sensor bundle, expires after 10 seconds, and rejects
replaced IDs, insufficient valid depth and depth discontinuities. It unprojects
a model-selected pixel using measured depth and camera intrinsics; it does not
select a radio or infer a button normal.

One important prerequisite emerged from installed OmniGibson 3.9.2 source:
`ToggledOn._initialize` makes `visual_marker` visible and `_set_value` recolors it
red/green from task state. The earlier development wrist recording therefore
contains a simulator hint. Opt-in marker hiding now changes only USD visibility,
checks that overlap extent is unchanged, and never writes the Boolean state,
pose, scale, mass, friction or collision flag. The installed `GeomPrim.extent`
uses local geometry points, independently of visibility. CPU tests verify that
contract with doubles, but live marker removal and sensor readability have not
been tested. A marker may be an annotated control mesh; hiding it can also make
the relevant surface harder to see. The adapter refuses initialization unless
runtime capabilities report hidden markers. Do not claim the new RGB envelope
is verified until a live render check passes.

`move_pose` accepts base-frame EE XYZ/XYZW action intent. `press` accepts a
straight EE displacement <=20 mm. The supervisor keeps the existing oracle
radio/support collision checker, frozen-joint check, stored plan ID and command
payload admission. It converts base-frame intent into the SDK world frame,
using the existing simulator localization and explicit model-height calibration.
These remain development assumptions. Pose execution uses the same checked
plan ID, nonblocking SDK dispatch and authoritative execution wait. Planning and
cancellation share admission coordination; cancellation during checking cannot
later dispatch. Each action has an ID and watchdog (timeout <=30 seconds).
`TIMED_OUT` is not proof of stop: the watchdog requests cancellation, confirmed
SDK terminal status is required, and an uncertain stop blocks subsequent
commands. Cancelling a model turn or killing its Python client is separate from
`policy.cancel(action_id)`. Network RPC stalls remain a limitation requiring
owner cleanup of the simulator on an unconfirmed stop.

### Recommended next bounded run: vision grounding first

Approval is pending. Proposed model is still `gpt-6-luna`, standard tier,
maximum six provider requests, six tools, 2,048 output tokens per request,
no retries/hosted tools, <=60 seconds model/tool loop and <=180 seconds total
simulator slot including startup and cleanup. Full-window conservative pricing
bound is `6 * (1.05M * $0.10/M + 2048 * $0.50/M) = $0.636144`, below a proposed
$1 ceiling. The existing pilot controller/extension must be updated and
preflight-tested for this stage before requesting inference. Do not silently
reuse the earlier exact-script-only two-tool limits or assume they enable this
new flow.

Use official `turning_on_radio`, scene `house_double_floor_lower`, definition 0,
instance 0, seed 0; restart/reset the simulator for a fresh episode. Preserve
custom base `[3.6,4.15,0.005]`, yaw pi/2, unchanged dynamics/goal. Owner closes
the gripper and stages a collision-checked inspection pose before policy input.
That pose can be borrowed from the known baseline for this first experiment,
but it is explicitly oracle-assisted initialization, not autonomous perception.
Initial head/wrist views were unusable. If marker-free inspection RGB does not
show a usable radio/control surface, stop before a paid call; never substitute
the evaluator camera. Owner collision scene, baseline targets, evaluator logs,
and marker geometry must not enter the model context.

Prompt: "Using only the supplied RGB/depth observation and robot feedback,
identify the radio control you would operate. Return the selected pixel and
uncertainty. If the control is not visible, ask for another permitted view;
do not guess from hidden task state. Write Python using RadioPolicy to ground
that pixel, and report the measured base-frame point. Do not command motion."

The concrete sequence is model write -> Python observe/save RGB and observation
ID -> Pi read of that actual RGB image -> model write of selected-pixel Python
-> Python `policy.ground(observation_id, u, v)` -> model interprets XYZ/error.
Both generated Python programs use `Dimos.connect(timeout=5)` and the task-local
facade. `Image.save(path)` is an existing method; do not create screenshots from
GT. The last response must distinguish observed target localization from task
success. The observation ID prevents pairing a pixel with a newer camera frame.

Example first program:

```python
import json
from dimos.porcelain.dimos import Dimos
from dimos.simulation.behavior.radio_policy import RadioPolicy

app = Dimos.connect(timeout=5)
try:
    policy = RadioPolicy.from_app(app)
    observation = policy.observe("left_wrist")
    if not observation["rgb"].save("policy-observation.png"):
        raise RuntimeError("RGB save failed")
    with open("observation-id.json", "w") as stream:
        json.dump({"id": observation["id"]}, stream)
    print({"camera": observation["camera"], "timestamp": observation["rgb"].ts})
finally:
    app.stop()
```

The next program loads only that ID, supplies the pixel selected from the image,
and calls `policy.ground`. No baseline target coordinate is supplied to it.
The 10-second observation lifetime may require a fresh observation if the model
is slow; treat expiry as a result, not permission to increase limits or use GT.

### Physical policy step after grounding review

Do not call a 3D surface point an executable end-effector pose. Still needed:
a reliable sensor-derived contact normal/approach, and a checked transform from
the robot's leading fingertip/TCP to the SDK `left_gripper_link` target. The
baseline's object-marker position/normal may not substitute for either. Robot
mesh/kinematic TCP calibration is legitimate robot information; its existing
5.3 mm development model-height offset must retain provenance. No new learned
perception model or asset download is proposed. The first grounding run tests
whether RGB/depth makes the physical policy feasible before guessing a press.

After those checks and stage approval, action intent becomes a precontact
base-frame EE pose and <=20 mm press displacement, realized through
`policy.move_pose`, `policy.press`, `policy.status` and `policy.cancel`. The
closed gripper remains fixed in this first task; no raw joint/base commands or
symbolic semantic primitive is exposed. A failed/uncertain action must stop and
return allowed robot/motion feedback. Do not forward oracle collision exception
text. Our independent owner records contact/ToggledOn/BDDL, official goal and
termination, radio displacement, action IDs/cancel confirmations, raw camera
frames, actual command payloads and timing. Policy video contains only sensor
views; evaluator side-camera/status overlays belong to a separate labeled
artifact. Success before retraction remains distinct from completed motion.

This is development validation with assisted initialization and a privileged
safety checker, not fair benchmark performance. Generic filesystem/process/
network isolation and cheating audit remain deferred. No additional model call
or simulator run has occurred during adapter preparation.

Before the approved live stage, copy the new child revision into a fresh disposable
runtime snapshot, reusing the already accepted simulator environment/assets.
Keep `/tmp/behavior-validation.oNeVq8` as the successful baseline recovery
snapshot. Point the owned gateway's Python imports and simulator worker at the
new snapshot, then verify `inspect.getfile(RadioPolicy)` and the exact source
revision before an agent call. Current remote gateway imports still point to the
older successful SDK snapshot; the new policy adapter has only local CPU tests.

## Placement variation preparation (CPU only, 2026-10-01)

The installed official source is BEHAVIOR-1K revision
`b1979916ec1549b10a4e65e630bc6504a9af1b00`, OmniGibson 3.9.2. The radio data
already provides 300 training instances (0–299) and 20 locally available public
test instances (301–320). The evaluator defines 20 additional hidden test
instances (321–340). These are different cached initial states. Repeating one
instance restores that state; changing the seed alone does not resample radio
placement. Optional online object sampling is a distinct initialization mode;
`randomize_presampled_pose` selects among robot poses, not radio positions. Our
current integration uses offline training-instance loading and the first robot
pose. Source references:

- [BehaviorTask initialization and reset](https://github.com/StanfordVL/BEHAVIOR-1K/blob/b1979916ec1549b10a4e65e630bc6504a9af1b00/OmniGibson/omnigibson/tasks/behavior_task.py)
- [Official evaluator cached-state loading](https://github.com/StanfordVL/BEHAVIOR-1K/blob/b1979916ec1549b10a4e65e630bc6504a9af1b00/OmniGibson/omnigibson/eval/evaluator.py)
- [Evaluation split constants](https://github.com/StanfordVL/BEHAVIOR-1K/blob/b1979916ec1549b10a4e65e630bc6504a9af1b00/OmniGibson/omnigibson/eval/utils/eval_utils.py)

Keep official cached instances and custom near-field variations in separate
results. Before inspecting geometry, select held-out training IDs by sorting
IDs 1–299 by SHA256 of UTF-8 `radio-variation-v1:<id>` and taking the first three:
**162, 171, 30**. Development instance 0 is excluded. Never replace a case after
observing its IK, perception or task result. Do not tune a policy on these
held-out geometries. Developer geometry access in this study is not an enforced
runtime isolation boundary.

### Actual CPU results

All four official cached samples initially have ToggledOn=false. A nominal
stationary left-arm-plus-torso screen retained each official R1Pro base, used
an explicitly oracle-derived candidate gripper pose, and kept base/opposite
arm/gripper joints fixed:

| Training instance | Horizontal base-to-radio distance | Precontact IK screen |
| --- | --- | --- |
| 0 (development) | 1.833 m | Joint-limit rejection |
| 162 | 2.049 m | Joint-limit rejection |
| 171 | 1.665 m | No convergence within iteration budget |
| 30 | 1.461 m | Differential-IK QP had no solution |

These are failed candidate screens, not proofs that every arm configuration is
unreachable. No official-instance navigation, physical contact or BDDL success
was tested. Official task starts must not be advertised as stationary-arm-ready
based on the custom-base success.

Three custom cases were declared before planning: radio root shifted -8 cm in
world X with -10 degrees world yaw; +8 cm X with +10 degrees yaw; and +8 cm Y
with unchanged yaw. Table and the same assisted near-field robot base stay
fixed. Only the initial radio pose varies; physical properties and the BDDL
goal do not. All three passed actual RadioManipulationModule SDK planning,
TOPPRA materialization, cached-command anchoring and discrete configuration/
edge collision validation for lift, cross, precontact and 12 mm press. Only
the intended finger/radio contact pairs are allowed during press, plus the
existing conservative static radio/table support pair; other checked collisions
remain active. Each case retained the same 11 selected arm/torso joints.

The entire conservative radio footprint stayed within table-box XY bounds;
the tightest margin across cases was 26 mm. The conservative radio box extends
about 9.3 mm below the table-box top, also true of the saved original. These box
checks **do not establish physical support, mesh clearance, stable settling,
sensor visibility or BDDL initial predicates**. CPU targets were transformed
from the known development target using object truth; they are only feasibility
diagnostics and must never be passed to the sensor policy. Full house obstacles
were not reconstructed. Keep all three cases, including any later live failure.

### Prepared common comparison interface

`radio_baselines.PressBaseline` executes either a locked development intent or
a perception-generated intent through the same `RadioPolicy` facade, checked
SDK planner and action-ID feedback. Its pollable runner halts on failure,
cancellation or uncertain stop; motion timeouts remain supervisor-owned.
Completed SDK stages do not imply task success. The independent owner checks
BDDL, contacts, displacement and termination.

`perception_intent` requires a detector callback supplied only an actual
left-wrist observation. It grounds that callback's pixel against the exact
observation ID, verifies base-frame provenance, and subtracts a known robot
finger-to-gripper offset using the sensor-estimated orientation. The callback
must also estimate the inward direction from sensors. **No detector or contact
normal estimator is implemented by this helper.** There is no fallback to a
marker, object transform, variant ID or evaluator pose. This prepares the
shared execution contract; it does not make the perception baseline complete.

For the eventual comparison, freeze the development-coordinate intent before
held-out runs; instantiate a deterministic sensor estimator for the perception
script; let Dimcode write sensor-grounded Python through the same facade. Use
identical initialization assistance, observations, action limits and timeouts.
Do not substitute a case-specific oracle target for either sensor condition.
Placement adaptation alone demonstrates value over fixed coordinates, not an
inherent agent advantage over a competent perception script.

### Next bounded stages and remaining blockers

1. The user approved one $1/60-second Luna vision-grounding stage while asleep,
   with at most six requests/tools and a coordinated <=180-second simulator slot.
   No new model call has been made yet. First verify opt-in marker hiding
   on a fresh live snapshot and usable wrist RGB/depth, with independent initial
   predicates and unchanged contact semantics. No policy press in that stage.
2. Resolve sensor-only button selection, approach-normal estimation and checked
   robot fingertip calibration using those actual observations. If imagery is
   inadequate, report that result rather than supplying hidden coordinates.
3. With separate GPU/run approval, initialize each custom pose only before the
   episode, settle with ordinary physics, reject/report invalid support or
   initially satisfied goal, then run the predeclared comparisons. Preserve
   every case and classify initialization failure separately from policy or
   motion failure. Do not freeze radio or change friction/mass. Keep the existing
   official goal; do not symbolically toggle it.
4. Original official-base evaluation remains a separate mobile-manipulation
   condition requiring a navigation/reachability plan. It must not be combined
   with near-field results as official benchmark performance.

Local development evidence is retained outside the repository in
`radio-transfer/radio-variation-input.json`, `radio_variation_cpu.py` and
`radio-variation-cpu.json`; the accepted host CPU run was isolated in
`/tmp/radio-variation-cpu.iH0rmz`. No simulator/GPU worker, physical controller,
paid request or new dataset download was started for this preparation.


### Full attachment dry run before the paid grounding stage

The development launcher must start the coordinator RPC service before any
borrowed `Dimos.connect()` client. `Dimos.run()` alone starts modules but does
not advertise that coordinator service. The disposable owner now uses the
existing public `ModuleCoordinator.build(blueprint)` and
`coordinator.start_rpc_service()` APIs, then attaches a borrowed Dimos client;
cleanup disconnects that client and stops the actual owner separately. This
matches the service startup that `ModuleCoordinator.loop()` performs. No MCP
wrapper is involved.

A complete CPU dry run passed with real forkserver workers, coordinator RPC,
module discovery, actual RadioPolicyModule initializer/supervisor and facade,
Arm SDK group discovery, borrowed-client disconnect semantics, and the existing
official Dimcode/Pi gateway tools. Only the external simulator-dependent motion
factory and telemetry sources were replaced by explicitly synthetic fixtures.
The gateway used a zero-network provider fixture, without loading the user's
credential file: write capture Python, execute it, read its PNG image payload,
write selected-pixel Python, execute it, then receive its actual grounding
feedback. All five tools succeeded; six fixture responses completed in about
1.52 seconds. Synthetic unit-depth and centered intrinsics produced expected
base-frame `[0, 0, 1]`. This is plumbing evidence, not model inference,
real-camera perception, physical task success or fair evaluation.

Earlier live preparation verified hidden-marker visibility with unchanged
position/radius/extent/scale/overlap result and toggle value, and completed three
assisted inspection moves without pressing. It stopped before model inference
because coordinator RPC was not advertised. Both that failure and the earlier
launcher failures are retained. The full CPU test fixes and exercises that
missing boundary before another GPU request. Paid request count and spend
remain zero as of this preparation. The same single bounded grounding stage
is still authorized and unspent; further paid stages require separate authority.

### Real planning-world handoff preflight

A subsequent live preparation reached the advertised RPC service and completed
all three assisted inspection moves, then stopped before inference: inspection
and policy initialization each constructed the checked-motion factory, so the
second factory tried to register the same named radio/table boxes again. The
real RoboPlan world correctly rejects duplicate names. This is a development
startup finding, not a GitHub review comment or CI automation failure.

The development SDK now retains explicit ownership of the first scene's boxes,
declaration, and measured reference. An identical registration is idempotent;
changed geometry, declaration, reference, externally mutated boxes, or preexisting
unowned names are rejected. Failed partial registration rolls back only its own
new boxes. The second factory checks live object drift against the original
reference (the existing 2 mm/0.01 rad limits) and reuses the original boxes in both
its private checking world and the SDK world. It does not silently move boxes.

The stronger CPU preflight passed two complete coordinator lifecycles with the
actual R1 bridge, coordinator, SDK planning world, first inspection factory,
and unmodified RadioPolicyModule initializer (second factory). Cached external
truth and cameras were explicitly synthetic; no motion was dispatched. It tested
0.1 mm cached settling without changing registered boxes, rejected changed box
extent and excessive cached object displacement, rejected second supervisor
initialization, and reattached/disconnected borrowed clients without stopping the
owner. The offline official Dimcode gateway completed all five tools and six
fixture responses in about 1.36 seconds, with an actual image payload and SDK
feedback. Both owner lifecycles shut down all workers. Eight real CPU world
regressions, 48 lightweight motion/policy/baseline tests, mypy and focused
pre-commit checks passed. The accepted assets and existing Python/Node runtimes
were reused; no simulator, GPU work or real model request was started.

The launch guard now requires this real-world handoff report in addition to the
native import and worker startup checks. All prior live failures are preserved.
The single bounded paid grounding stage remains unspent; another GPU slot must
be coordinated before retrying it. This preflight proves development plumbing,
not physical contact, autonomous perception or fair task performance.
