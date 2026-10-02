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

### Consumed grounding pilot and same-frame evidence correction

The single authorized Luna vision pilot completed on source
`8da8f56da28124e841785f466d00fe089ec788f5`: six requests, five successful
write/execute/read/write/execute tools, 11.77 seconds of model/tool time and
70.67 seconds including owner cleanup. Reported usage gives a conservative
$0.0039306 estimate, not a provider invoice. No retries or policy press occurred;
initial/final BDDL success remained false. The paid stage is consumed. Existing
source, original camera image, generated code, tool trace, evaluator diagnostics
and Library archive remain historical evidence; subsequent fixes do not change
that result.

Pixel `(195, 90)` grounded to base-frame
`[0.5045939933, 0.2097860704, 0.5258223400]` m. Post-selection evaluator analysis
placed the point 21.73 mm from a 22.36 mm-radius analytic toggle region, about
0.63 mm inside its boundary. That region is not a verified physical button
surface. Semantic control identification and press viability remain unproven.
The model's final text said it did not ground a pixel, contradicting successful
`ground.py` execution and returned XYZ. An explicit summary claim must be checked
against recorded command and SDK feedback; a successful tool trace is not proof
of arbitrary Python side effects or accurate natural-language narration.

The original pilot saved its selected RGB and reported depth/XYZ, but only the
older readiness full depth and rounded TF. The approximate post-selection
reprojection therefore cannot serve as an independent calibration measurement.
The new development path closes that recording gap for future runs:

- The native owner retains one messages() capture, tagged with an opaque capture
  ID, episode ID and owner simulation step. `get_sensor_snapshot()` returns only
  permitted camera RGB/depth/calibration and camera-to-base transforms; task
  objects, goals, semantic images and world transforms are excluded.
- Policy initialization now requires the owner to supply an absolute evidence
  directory, for example `initialize_development_scene(collision_scene,
  evidence_directory="/tmp/new-radio-run/sensor-evidence")`. It uses the atomic
  snapshot RPC rather than independently polling latest camera streams.
- An observation ID binds exact bytes, image formats, full camera calibration,
  timestamps, TF and step metadata via a content fingerprint. Mixed capture
  timestamps, expired frames, reused IDs with changed data, changed episodes and
  backwards capture clocks are rejected. Nonzero distortion is rejected by this
  pinhole-only development grounding path.
- Allowed sensor arrays and metadata persist in immutable pickle-free NPZ
  bundles. Grounding records bind the original observation ID/fingerprint to the
  selected integer pixel and SDK result. Independent projection and optical-Z
  checks reject mismatched pixel, point, depth or capture metadata. Existing
  conflicting/corrupted bundles are not silently overwritten.

This is narrow development evidence retention, not the generic isolation/audit
backlog or a claim of cheating resistance. The step is the native owner's capture
step; renderer latency has not been separately characterized. Evaluator truth
must remain in a separate owner log keyed by that episode/step, never in policy
observations or prompts.

CPU validation passed 85 focused tests, including real native owner methods with
an external engine fixture, retained-frame mutation/replacement/expiry checks,
projection/backprojection, persistence and explicit tool-summary reconciliation.
Two complete real planner/coordinator/policy lifecycles in a new disposable CPU
snapshot passed RPC serialization, exact bundle retention and cleanup with
synthetic external sensors. Native Python 3.11 source/contract imports passed
without initializing OmniGibson. No Node gateway, model, simulator, GPU or
actuation was started for those checks. The original paid snapshot was untouched.

The next physical attempt remains gated by new run/GPU authority. First acquire
and retain a fresh exact sensor bundle, identify the intended control or stop if
uncertain, then estimate a same-frame surface normal across several depth patches
and record their consistency. The historical 3/5/7-pixel plane fits are only
candidates from older depth and conditionally inferred intrinsics; they are not
execution-ready. Use known closed-gripper geometry to transform the chosen
contact point into an end-effector intent:
`p_base_ee = p_base_contact - R_base_ee * p_ee_fingertip`. Verify calibration,
contact uncertainty and collision shapes before selecting orientation and a
short precontact/press path. Use left arm plus explicit torso, freeze the base,
reject a stale/moved scene, and preserve confirmed cancellation/timeouts. The
independent evaluator must report actual contact and BDDL; sphere proximity alone
must never be reported as physical button correctness or task success.

### Live exact-frame and view-selection follow-up (2026-10-01)

The user subsequently authorized ordinary bounded simulator/model iterations
without per-phase budget questions. These development runs used ccdesktop's
RTX 3090 (24,576 MiB), independently of K1's 16 GiB host. Integration PR #4342
was unchanged; experiments stayed in the child workstream. Initialization still
used the explicitly disclosed custom base and oracle-designed inspection pose.

A new live capture on revision `09b9bf2d6` retained matching RGB, optical-Z depth,
full calibration and camera-to-base TF. The exact pixel grounding/projection
contract passed. A camera-axis retreat of 12 cm then completed through the
checked SDK. Real Dimcode gpt-6-luna wrote/executed observation and grounding
Python, but disclaimed control identification afterward. That numerical point
was not admitted for pressing; its local normal also failed consistency checks.

Two further bounded view experiments compared optical-X translations of
plus/minus 6 cm and optical-X orientation changes of 0.25/0.40 rad. Every
executed path passed the existing IK, collision, frozen-joint and scene-anchor
checks. A deterministic RGB red-body visibility score chose a view, without
receiving evaluator geometry. This score is a camera-framing baseline, not a
button detector. The agent received head-camera context and the original wrist
image; its actual wrist pixels were matched byte-for-byte to retained sensor
bundles. The head view did not show the radio. Both valid presentations ended
in explicit abstention and no grounding/press call. Radio pose and toggle
contact stayed unchanged in the recorded view stages; BDDL remained false.

An initial presentation attempt omitted montage creation in the prepared script.
Its evidence and estimated cost were preserved separately. The actual script
was corrected and checked with isolated SDK boundaries before the next live run.
A tool gate now requires a structured identification decision and rejects
grounding after abstention; its isolated CPU admission check passed without
provider calls. This experimental gate does not provide a sandbox or prove the
model's semantic judgment correct.

The three view-sequence runs, including the presentation failure, used 18 model
requests and a conservative configured-rate estimate of USD 0.0118847, not an
invoice. The earlier retreat stage used six requests and USD 0.0040196 separately.
All owned simulators, workers and gateways were stopped; the final GPU check
reported 22,450 MiB free. No radio freezing, changed dynamics, symbolic toggle,
hardware operation or publication occurred.

The remaining action precondition is a defensible sensor-derived control target.
These views do not prove the task globally impossible. Body visibility, a smooth
depth patch, and the benchmark's private spherical toggle annotation do not
establish a recognizable mechanical power button. Do not convert abstention
into arbitrary grounding or label this partial flow autonomous task success.
Full agent-selected view planning and sensor-derived contact execution remain
unverified. Local consolidated evidence is
`dimcode-grounding/same-frame-followup/view-selection-outcome.json` in the
delegated development workspace; it is not bundled into the repository.

### Evaluator-only observability audit

The official wxnicr metadata defines its toggle location as a spherical meta
annotation on base_link; no separate power-button part is identified there.
The official converter generates that sphere as guide geometry independently
of the base visual mesh. ToggledOn makes the marker visible and colors it
red/green, while its default body texture adjustment is neutral. Contact plus
analytic overlap for five steps toggles the state; mechanical switch travel
is not evaluated. The recorded visibility invariant checks preserve this
trigger, but hiding the marker removes an explicit location affordance as well
as its state-color feedback. It therefore changes the sensor task, even though
physics and BDDL are unchanged.

A privileged projection of the annotation onto actual retained agent pixels
lands on the dark center of the speaker-like circular feature. This is an
approximate nearby-step evaluator diagnostic, not an agent observation or a
confirmed physical switch. Metadata, images and source do not establish that
a camera-only agent can uniquely infer the annotated location. At that stage we had not opened
the loaded USD topology or proved that no other control exists anywhere;
the base texture atlas includes frequency/tone graphics on other regions.

The installed official radio model vgioak is a candidate for a new evaluator-side
whole-asset/front/top observability check with markers hidden. No matching cached
turning_on_radio instance using vgioak was found; the available definition-0
template uses wxnicr. Do not silently substitute models or relocate annotations.
An alternate custom development instance must be labeled accordingly. A static
neutral target marker would still encode privileged interaction-location data,
so it cannot silently replace realistic sensing. Until a recognizable control
and its mapping are established, label the current setup an observation/interface
limitation rather than spend repeated grounding calls or claim semantic task
performance. The initial audit added no simulator/model run and changed no asset/state.


A subsequent evaluator-only CAD audit loaded natural-scale render-only replicas
of both installed assets in a separate empty scene. With markers hidden, wxnicr
shows upper roller/knob features, but the registered toggle projects onto its
speaker face rather than those controls. vgioak has a visibly distinct top control
panel; its registered toggle projects near the middle large knob. This is
alignment evidence, not proof that the knob is a power control, reachable, or
physically actuated by a press. Both loaded assets have base_link plus the toggle
meta link and zero articulated joints. The renderer did not run a robot, scored
task, manipulation policy or model call. It did not substitute the original task
asset, move any trigger, or modify mass/friction. The application shut down after
approximately 14 seconds; its process was absent on a subsequent check and the
GPU had 22,447 MiB free. The script's post-shutdown report footer was not emitted
because shutdown exited the process; independent process/log evidence confirms
cleanup.

The pinned official ToggledOn implementation explicitly makes its marker visible
and colors it red/green. Our earlier default-marker recording contains that
marker in genuine robot wrist RGB. No radio-specific removal was found in the
inspected default evaluation observation path. This establishes the inspected
version/configuration, not every challenge configuration. Hiding the marker
changes the observation contract; a constant-color marker would remove color
state leakage but still provide privileged interaction-location information.

Two honest next experiments are distinct: reproduce the official visible-marker
observation contract and report its affordance/state hints, or use a declared
custom instance with a visibly interpretable control and verify its mapping
before sensor-only manipulation. The latter cannot silently substitute vgioak
into the wxnicr cached instance or claim original-instance benchmark performance.
The CAD result justifies a control-mapping review, not another blind model call.

### Future exploration: code-generated demonstrations

Record for later: use agent-written or static code-as-policy through the Python
manipulation SDK to generate simulator demonstrations, then train a learned
policy such as ACT on those trajectories. This is a future exploration, not
authorization to start dataset generation or training. The current agent-and-SDK
task remains the priority. Demonstration provenance and the runtime observation
contract would need to be declared before comparing a learned policy with its
scripted demonstrator. The pending visible-marker decision remains unresolved;
this idea does not authorize restoring marker location or state-color hints.


### Demonstration-derived bimanual preparation

The official task-0 annotations inspected for episodes 00000010/20/30 use
move-to → pick-up-from-coffee-table → coordinated press → place-back-on-table.
The development helper `radio_bimanual.py` now expresses that manipulation
sequence with the right arm holding and the left arm pressing. It uses actual
`Arm.set_gripper_position()` calls with separate configured gripper tasks and
per-arm measured feedback. The blueprint's `bimanual=True` enables both arm
groups and torso, retains the existing base-free trajectory task, and hides
toggle markers. It rejects the existing single-arm policy supervisor rather
than imply that supervisor supports both arms. This is development code on the
child branch; neither the integration head nor a remote PR was changed.

`BimanualRadioFlow.tick()` admits at most one synchronous SDK stage; a caller
can poll unfinished gripper/contact waits without spawning a worker. A total
deadline is bounded to 120 seconds, with at most 20 seconds supplied to each
checked move. One shared manipulation runtime owns both arms sequentially.
Cancellation halts later admission, preserves a holding gripper, and records
UNCERTAIN/FAULT or an RPC failure as unconfirmed stopping. Failed physical
retention prevents lift/press; failed placement support prevents release. The
left press target is requested after lifting, rather than reuse the old tabletop
target. Completion denotes SDK stage completion, never BDDL success.

The helper deliberately requires a caller-supplied checked-motion implementation
and physical-retention/support feedback. The current static radio collision
registration is NOT such an implementation: it rejects movement of the radio
and cannot represent a lifted object along the path. Before a live bimanual
run, the right grasp must use verified handle/contact geometry, and checked
planning/admission must include the carried radio, both arms, intended finger
contacts and the supporting table. Removing the radio from collision checks,
freezing it in the simulator, changing dynamics, forcing an assisted grasp,
or symbolically toggling success are not part of this helper. Targets and
feedback must declare sensor-derived versus evaluator-assisted development
provenance; no evaluator ground truth is silently supplied to a runtime agent.

CPU screening used the prior explicitly declared near-field base and the same
radio/table padded boxes and FK calibration. With torso frozen, the tested
right-above-radio and left-precontact targets did not converge. With explicit
torso assistance, independent right-above-radio and left-precontact endpoints
and paths were found. This does not establish simultaneous feasibility. One
common-posture attempt reached an IK endpoint but its RRT path timed out at
five seconds; a retained seeded attempt did not find a common IK solution.
These are bounded solver outcomes, not proof of global unreachability. The
above-radio pose is a geometry screen, not a known grasp. Installed finger
joint limits are 0–0.05 m on both hands; joint travel alone does not establish
handle pinch clearance or physical retention. All outcomes, including failures,
are retained in the delegated workspace's `dimcode-grounding/bimanual-development/`.

The next discriminating preparation is exact handle/finger collision geometry
and a common grasp/hold/press posture with carried-object collision admission.
No bimanual simulator, model or hardware execution has been performed. Marker
contract approval remains pending, but does not block this CPU preparation.

### Exact handle geometry and carried collision preparation

CPU inspection used the accepted asset through OmniGibson's official temporary
asset loader and the installed USD parser, without starting Isaac or a GPU run.
The wxnicr top rail is collision part 6: approximately 35.0 mm across the pinch,
227.3 mm along the handle and 17.2 mm vertically. Installed distal finger mesh
bands have an approximately 99.9 mm opposing inner gap at 0.05 m opening.
This supports screening a top handle pinch; it does not prove frictional
retention. All 14 original convex collision parts are represented. Internal
hull overlaps within the single rigid radio are excluded; every external
robot/radio pair stays enabled. The initial table-support contact is explicitly
separate from carried-object/table collision checks.

The bounded CPU pregrasp and grasp endpoint/path checks and their connecting
path succeed with explicit torso assistance and fixed base/opposite arm. These
are oracle-assisted scene checks, not physical grasp execution. The bimanual
flow now permits measured partial finger closure when independently verified
physical retention is present: a real handle can block the empty-hand zero
position. Partial closure, commanded closure or contact alone is insufficient.

Installed RoboPlan 0.6.0 has no attachment API, but can update a geometry's parent.
A synthetic regression exposed a contract discrepancy: updating geometry
relative to a fixed tool frame omits that frame's fixed offset. On R1 Pro,
`right_gripper_link` is offset from `right_arm_link7` by (-0.0295, 0, -0.16065) m.
The task-local `temporary_carried_radio_geometry()` composes this offset from
one FK snapshot and reparents to the moving wrist frame. It holds the existing
scene lock for planning/checks, preserves all collision pairs and restores
world-frame placements even after partial setup failure. Failed restoration
invalidates the world. This is a CPU collision model only, not a physical or
assisted grasp, execution authorization or general SDK attachment API.

Corrected-frame screening finds a collision-free lifted endpoint with base,
torso and opposite arm fixed and every external radio collision enabled. A
nominal 0.0205 m finger opening supplies geometric clearance for that check;
physical retention is explicitly unproven. The original calibrated left press
orientation and a shared torso endpoint attempt failed bounded IK joint-limit
checks. These outcomes are not proof of global unreachability. Alternate rolls
about the press normal are screened with recomputed TCP contact offsets.

A live trial still requires a verified grasp/retention signal, a continuously
checked lift with explicit initial support-contact handling, physically valid
press fingertip geometry and measured execution admission. No radio freezing,
dynamics changes, symbolic success, marker restoration, GPU run or autonomous
success claim occurred in this preparation. Raw failed and corrected CPU
reports/scripts remain in `dimcode-grounding/bimanual-development/` in the
delegated workspace. No integration head or remote PR was changed.

The three retained alternative press-normal roll checks (90°, 180°, 270°)
failed bounded IK: 90°/270° returned joint-limit outcomes and 180° returned
no differential-QP solution. All used recalculated contact offsets, a fixed
base/torso/holding arm and external carried-radio collision checks. A future
shared torso design endpoint also failed. Consequently no complete two-arm
execution or simulator trial is admitted from these candidates. The next CPU
work is a hold/reorientation pose compatible with both arms, rather than
spending GPU time on the current failed press configurations. Physical
retention and initial table-contact lift admission remain separate gates.

Focused regression validation after these changes: 61 tests passed across
bimanual flow, carried-collision restoration, radio baselines and blueprint
configuration. Ruff, scoped mypy and diff checks passed. Native CPU fixtures
separately reproduce the fixed-frame issue and the corrected moving-frame
collision behavior; these do not constitute simulator task success.

### Bounded hold search and lift prerequisites

A finite follow-up search tested two physically admissible top-handle pinch
rolls (0°/180°), fixed versus auxiliary torso before grasp, five small held
translations/yaws and two calibrated left press rolls. Of 28 recorded endpoint
checks, two grasp endpoints and seven held endpoints passed; none of the 14
left press checks passed. Base was fixed throughout; torso and holding arm
were fixed for every left press check. External carried-radio collisions stayed
enabled. This rules out executing these candidates, not every possible radio
posture or the official two-hand strategy.

The simpler grasp/lift checkpoint advanced independently: the first 10 mm
Cartesian departure succeeds with only the known body/table support pair
allowed. Across 79 sampled configurations, the lowest radio vertex rises
monotonically (0.2 mm comparison tolerance); its starting clearance against the
padded table is -1.526 mm and final clearance is +8.337 mm. The next 50 mm
Cartesian lift succeeds with ALL external collision pairs enabled. Its start
must be refreshed from actual FK rather than reuse the nominal endpoint of the
first segment. These are oracle-assisted CPU path checks, never grasp-retention
or simulator-execution evidence. Initial support contact must still be bounded
and validated during effective-command admission.

The execution gap is concrete: `DevelopmentRadioMotion` pins static radio/table
scene poses, and `configure_development_collision_scene()` accepts only those
two stationary boxes. The new temporary carried helper is currently local
planning infrastructure; it is not an SDK RPC lifecycle/ownership contract.
A grasp/lift trial requires a task-local binding for the exact convex handle
scene and the carried model during BOTH stored SDK planning and effective JTT
trajectory validation, with freshly measured retention/slip evidence and
bounded initial support contact. Broadening the SDK's general attachment API
is not required or claimed here. No unchecked carried trajectory was dispatched,
no GPU slot used, no marker restored and no physical retention/BDDL result
invented. The prepared next checkpoint is grasp-and-lift with per-stage pose,
finger-contact, gripper feedback and genuine camera video, not full press.

If the additional task-local binding is deferred, a simpler official BDDL
candidate outside the currently cached 100-task inventory is
`turning_on_the_hot_tub`: installed definition 0 has one goal, ToggledOn(hot_tub).
Its fixture dynamics, usable control geometry, scene availability and arm
reachability remain unverified; it is not presented as runnable. The cached
`turning_out_all_lights_before_sleep` requires all three switches and two lamps
across rooms, so a single stationary switch push cannot satisfy that full task.
No alternate task was loaded or substituted.

### Task-local SDK grasp/lift binding

The execution-binding gap now has a contained implementation on the radio child.
`radio_checkpoint.py` owns a checksummed 14-part wxnicr convex scene plus its
padded table and supplies the same phase context to both sides of SDK RPC.
`BimanualRadioManipulationModule.configure_radio_checkpoint()` establishes that
owner; `plan_radio_checkpoint()` stores an ordinary SDK GeneratedPlan using the
scene context and validates the nominal trajectory before returning its ID.
The checked runner prepares the coordinator's effective trajectory, validates
that exact command with the identical geometry signature and contact phase,
then authorizes and executes the same stored ID. Pregrasp may explicitly use
torso assistance; the holding stages freeze base, torso, opposite arm and
measured grippers. Linear checkpoint motion uses an explicit 0.01 m/s bound
without an additional speed multiplier that would exceed stage deadlines.

Only the known radio-body/table support pair is allowed before clearance;
departure additionally checks interpolated radio hull heights, footprint,
monotonic elevation and a bounded 2 mm initial overlap against the padded table.
Only the two right fingers against handle part 6 are allowed for holding.
After clearance every radio/table pair is enabled. All temporary external
allowances are restored; failed restoration invalidates the scene. The existing
moving-wrist frame composition handles installed RoboPlan's fixed-frame offset
behavior. This remains specific to this asset and right gripper, not a generally
validated attachment feature or framework redesign.

The runner checks fresh episode/scene/joints before authorization, rejects stale
or changed starts, polls candidate grasp contact/blocked closure and relative
radio/FK drift during execution, and requires measured FK arrival after SDK
clock completion. Failure calls SDK cancellation, preserves the holding
command, and records whether stopping is confirmed. SDK FK and separate
privileged physical-gripper pose evidence are recorded so model discrepancy
can be distinguished from physical slip. Runtime `ground_truth()` now includes
radio finger-contact diagnostics from actual rigid contact pairs; this remains
evaluator-only debug data and is not supplied as a policy observation.

Native CPU parity verification passes initial departure (79 waypoints; 158
support interpolation samples) and subsequent lift (395 waypoints) for both
nominal and coordinator-anchored paths with the same geometry signature. An
invalid cached start is rejected. Focused regression validation passed 101
tests, followed by passing targeted checks after explicit speed/evidence
adjustments; Ruff, scoped mypy and Python 3.11/3.12 compilation checks pass.
No hardware, simulator/GPU execution, marker restoration or task-state setting
occurred during this binding work.

A bounded genuine-camera smoke harness is prepared in the delegated workspace
and isolated remote `/tmp/behavior-radio-checkpoint-20261001`. It uses the
already accepted runtime/assets, the declared near-field start, measured
opposing-finger contact and settled gripper readback, then 10 mm departure and
50 mm lift. It retains stage poses/contacts/FK, SDK command digests, video and
independent BDDL status. It preserves dynamics and the official radio goal and
stops on admission/contact/retention failures. Actual grasp/lift validation and
any video are still pending a coordinated GPU slot. Full two-hand press search
is paused until that checkpoint; no alternative task was substituted.

### First bound grasp/lift runtime checkpoint — 2026-10-01

A coordinated 170-second development run used the existing accepted runtime and
assets, declared near-field spawn and hidden markers. Pregrasp and Cartesian
insertion completed through the checked SDK stored-plan route. Settled closure
was 0.749784 normalized; actual contact diagnostics reported both right fingers.
The radio was stationary during approach, then translated 11.725 mm in world Y
and rotated about 3.54 degrees during closure. Opposing contact alone is not
verified physical retention.

The first 10 mm departure was rejected in planning at path fraction 0.545455:
position residual 2.744 mm exceeded the 2 mm tolerance. No departure command was
sent. The latest independent BDDL diagnostic still had goal 0 unsatisfied. No
press, lift, autonomous success or fair benchmark result is claimed. The SDK
cancel response reported the previous trajectory completed; this was not an
execution timeout or process/compiler crash. App shutdown returned, the owned
process group and simulator PID were absent, and GPU memory returned to its
ordinary desktop level.

One CPU diagnostic replay used the recorded commanded grasp endpoint plus
measured closure/radio placement. Exact departure joint feedback had not been
persisted, so this is explicitly approximate. Fixed-torso right-arm departure
failed with both collision checking enabled and disabled, reproducing the
fraction and residual closely (2.730 mm). Declaring torso as a diagnostic
auxiliary produced a 201-waypoint native path in both cases. This points to the
chosen fixed-torso posture/kinematics, not a collision-filter failure; it does
not prove a solver defect or admit torso motion during a held-object checkpoint.
Future records now save exact measured starting joints, target transform and
structured planning result before dispatch; all 15 checkpoint regressions,
Ruff, scoped mypy and diff checks pass after that instrumentation change.

The next prerequisite is to select and validate a grasp posture with a
collision-checked arm-only lift lookahead before closure, keeping base/torso
fixed during retention. Full bimanual press search remains paused. The saved
head-camera video is genuine but faces the adjacent room and does not visually
establish finger/radio contact; contact and displacement claims come from
explicitly privileged development diagnostics. Reports, frame timestamps and
video are retained under `dimcode-grounding/bimanual-development` in the
operator workspace. No additional GPU run occurred during CPU diagnosis.

### Explicit torso-assisted checkpoint and evaluator camera

The earlier fixed-torso requirement was imposed by the task-local checkpoint,
not the SDK, R1Pro mobility model or a general manipulation contract. It is now
an explicit choice: holding stages may select right arm plus torso as auxiliary.
Base, left-arm joints and measured grippers stay frozen; torso motion naturally
moves the idle left TCP, and its full robot geometry is included in every path
check. This does not promise the left TCP will hold a world pose during a future
press. A simultaneous two-hand press must explicitly constrain both TCPs in the
existing multigroup planner, rather than mistake frozen left joints for a fixed
left-world pose. Carrying geometry follows the right moving wrist with the same
frame composition during both planning and effective command validation.

CPU checks validated the complete recorded effective approach paths, 21
samples of the observed closure displacement, 10 mm departure and 50 mm lift
with torso assistance. The path tolerance remains 2 mm. These are approximate
saved-command replays, not missing measured state reconstructed as truth. A
predictive preclosure check in the next harness uses its actual measured start,
expected blocked closure, prior observed closure transform and additional
world-Y displacement of -3, 0 and +3 mm. All three CPU fixture scenarios passed
both paths and support departure checks. Prediction remains a finite heuristic,
not a dynamics model or proof of grasp retention; actual closure/contact and
fresh execution guards remain mandatory.

A development-only `DIMOS_RADIO_DIAGNOSTIC_DIR` opt-in records the evaluator
viewer camera with a side view centered on the actual radio, `EVALUATOR ONLY`
video labeling and frame timestamps/stage provenance. It does not change robot
head/wrist cameras, policy sensor messages, markers or physical dynamics. The
runtime head-camera recording continues separately. Viewer/recording failures
are recorded explicitly. The new checkpoint is isolated in
`/tmp/behavior-radio-checkpoint-20261001b`; the first failed run is preserved.
Targeted tests pass 45 checks, including explicit SDK torso auxiliary selection
and camera recording/error cleanup; Ruff, scoped mypy and cross-interpreter
compilation checks pass. The single bounded live follow-up is pending completion.

The follow-up completed with exit 0: all three fresh-start predictive scenarios
passed, both checked holding motions completed, and the radio rose 59.084 mm
from its post-closure pose. Opposing right-finger contact was present in every
sample of departure (20 samples) and lift (91 samples). Evaluator physical
relative-position variation was at most 0.0164 mm in departure and 0.0080 mm
in lift. These measurements must be qualified: the pinned official r1pro.yaml
sets `grasping_mode: assisted`, and installed Robot._handle_assisted_grasping
creates/removes constraints. The run loaded that file unchanged; its runtime
mode was not explicitly persisted in this run. This proves SDK/contact/geometry
plumbing under the configured assisted simulator mode, not unassisted
friction-only retention. Future harness output explicitly records describe()'s
actual grasping_mode as well as measured ending joints.

BDDL goal 0 remained unsatisfied: the radio was not pressed/toggled and this was
not full task completion. Camera evidence now shows the actual handle grasp and
lift. The side camera recorded 169 frames. Its original stage file was not
updated before lift, so original lift frames retained a departure caption.
The operator MP4's caption is corrected using the saved SDK monotonic stage
start; the original AVI/JSONL remain preserved and the derived display timeline
records the correction. No scene pixels below the caption were altered.
Playback uses 4 fps and is not guaranteed to match wall-clock timing; exact
capture timestamps are retained. Both owned process group and simulator PID
were gone after shutdown, with GPU memory back to the desktop-only level.

The next press prerequisite is a fresh measured held-object state and a
collision-checked coordinated path constraining both right holding TCP and
left approaching/pressing TCP, with torso explicitly auxiliary where needed.
Frozen left-arm joints alone do not preserve its world TCP while torso moves.
The current task-local checkpoint binds only right-hand targets; a contained
extension of that binding is needed before using existing SDK multigroup
planning for a press. Recompute the hidden evaluator-only contact target and
carried geometry from the new radio pose, preserve realistic policy observation
boundaries, and check the independent original BDDL goal afterward. No press,
marker restoration, new GPU experiment, push or integration-head change was
performed after this checkpoint.

### Bounded dual-TCP press comparison

The task-local binding now passes both right holding and left acting TCPs to the
existing synchronized Cartesian planner, with explicit optional torso. Nominal
and effective paths share the carried mesh scene. Interpolated right holding
pose is limited to 2 mm and 0.005 rad; the same guard is applied to measured
execution feedback. Press contact admits only the declared left finger 1 against
radio body part 0, with a calibrated local contact corridor and at most 2 mm
normal depth; other left fingers/radio parts and every radio/table pair remain
collision checked. Failed or stale requests never authorize execution. This is
a specific development contact proxy, not a continuous collision guarantee or
untrusted-agent isolation interface. Focused regression validation passes 51
tests, Ruff and scoped mypy.

The CPU backend accepted a 50 mm left-clearance trajectory while constraining
the right holding TCP: 965 waypoints, 1,930 interpolation samples, maximum right
position error 0.719 mm and rotation error 0.000854 rad. The subsequent calibrated
precontact failed at path fraction 0.148383 with position error 2.026 mm and
orientation error 0.000264 rad. The initial replay had incorrectly reused a
stale first right waypoint between stages; correcting that native start-pose
contract removed the INVALID_GOAL error but did not solve the approach.

A bounded follow-up sampled tool rolls -90/+90 degrees about the same press
normal, recomputing TCP position to keep the calibrated fingertip contact point.
It additionally tested translating the held radio 50 mm toward the idle left
TCP ([+11.045, -48.746, -1.347] mm); this preparation path passed with 986
waypoints and carried geometry. After fixing the new right anchor, none of the
four dual-TCP approach candidates passed. Reported position residuals were
2.083, 2.544, 2.198 and 2.356 mm at path fractions 0.074–0.076; all orientation
residuals were below the 0.005 rad bound. Endpoint IK either did not converge or
reported joint-limit violations. These are backend Cartesian residual failures,
not the separate collision-post-validation rejection. The native error does not
identify which TCP contributed the reported aggregate residual; no unsupported
attribution to the left alone or proof of global unreachability is made.

The public SDK currently uses complete pose/quaternion targets and a scalar
orientation tolerance. These tests changed candidate roll without loosening that
tolerance; they do not constitute continuous free-roll optimization. No SDK
axis-mask feature, broad planner rewrite, new simulation, paid model request or
marker restoration was introduced. Exact measured final lift joints were absent
from the saved successful run, so these CPU trials used its actual measured
starting state plus final effective command, explicitly approximate. Future
harnesses retain measured ending joints.

Grasping remains optional for the original task. The already successful
near-field oracle tabletop baseline is the shortest path to an agent+SDK
end-to-end attempt: resolve the observable button/marker contract, capture a
fresh discriminating real sensor view, obtain a sensor-derived press intent,
execute the existing checked single-arm SDK contact path, and read independent
BDDL success. It avoids a grasp, assisted retention, lifted-object geometry and
an additional constrained arm. The tabletop baseline's oracle success does not
prove agent perception; earlier sensor-policy abstention remains evidence, so
another paid call without a discriminating input is not justified. The grasp/
lift strategy and its successful video are preserved as a separate development
capability, with a jointly feasible holding/press posture still to be found.

### Contact provenance and exact CPU failure diagnosis — 2026-10-01

The old tabletop approach placed the gripper reference origin inside the
radio collision box. The accepted asset's nearest face implies an outward
direction approximately [-0.05003, +0.99742, +0.05150] in that recorded world
pose; physical pressing should approach along its negative. This is inferred
collision-surface geometry, not a normal supplied by the spherical toggle
marker. Recompute it and the calibrated fingertip-to-TCP offset from the fresh
radio pose after every placement, grasp or lift. A TCP point alone does not
bound the palm and both finger collision shapes. No friction, mass, fixed-object
constraint or symbolic toggle was changed to obtain a task pass.

Runtime evaluator diagnostics now preserve full robot/radio rigid contact-link
pairs, both physical gripper poses, per-stage radio translation in world and
starting-radio frames, and rotation angle. A sample rejected by the contact
guard is retained before cancellation. These records help localize first
displacement to insertion, closure, lift, approach or press, but contact-pair
membership does not establish contact force or normal. Sampling can miss brief
contacts; no continuous physical-contact claim is made. They remain
evaluator-only development observations, not agent inputs.

The native failure state of the baseline held-radio approach reproduces the
reported position residual within 2 micrometers: left TCP 2.026411 mm, right
holding TCP 0.549597 mm, at path fraction 0.148383. Both initial TCP frame errors
are zero. Left arm joint 2 is at its actual -0.1745 rad lower limit, within
2.4e-9 rad; the stacked Jacobian has rank 12 and smallest singular value
0.018693. Lowering damping from 0.01 to 0.001 or halving dt from 0.01 to 0.005
reproduces the same failure. This identifies an active joint bound on this
continuous path, not global unreachability or a demonstrated reference-frame
bug. These replays still use measured lift-start joints plus the final effective
command because exact measured ending joints were absent from the saved run.

Two model-defined left-arm clearance routes passed carried-geometry and
holding checks, but subsequent calibrated precontact still failed at the same
shoulder bound. With the actual negative elbow-flex direction, the approach
fails at fraction 0.189487: left residual 2.291861 mm, right 0.909362 mm. The
snapshot again matches the native reported residual. An earlier candidate's
snapshot does not match its reported residual and is not treated as an exact
failure configuration.

A single CPU-only prototype used RoboPlan 0.6.0's public custom components to
prefer left shoulder joint 2 at 0.45 rad and elbow joint 4 at -1 rad as a
secondary nullspace objective. Both TCP objectives retained priority 1; native
position/velocity limits, 2 mm / 0.005 rad tolerances and the carried collision
scene were preserved. It failed at fraction 0.158931 with reported position
error 2.231 mm. No production planner or physical joint command was changed.
The pinned source's custom-component constructor does not automatically add
default constraints; this prototype explicitly supplies both constraints.
See [the pinned RoboPlan source](https://github.com/open-planning/roboplan/blob/0.6.0/roboplan_cartesian_planning/src/cartesian_path_planner.cpp).

All 53 focused checkpoint/contact/bimanual/camera tests pass after instrumentation;
Ruff and scoped typing/diff checks are run locally. No simulator/GPU run was
requested: there is still no accepted holding-and-press path. The next useful
physical trial requires a jointly feasible precontact posture with the held
radio, fresh measured joints and carried meshes, followed by the bounded
finger-pad contact path. Holding is one stabilization option already demonstrated
under assisted grasp; it is optional for the official task and is not evidence
of an unassisted friction-only grasp or completed BDDL goal.

### Endpoint-first held-object search and accepted contact continuation

A bounded 16-pose screen changed the retained radio pose instead of tuning the
same failed path: 150 mm toward the idle left TCP, two heights, 90/180-degree
held-wrist yaw and four normal-axis left-tool rolls, at most three IK attempts
per endpoint. None passed collision admission. A separate six-case diagnostic
retained converged unfiltered IK only to inspect native collision rejection.
The 180-degree turn removes the original shoulder-bound problem, but the
left gripper/wrist or retained radio then collides with torso geometry. Wider
button clearance alone did not fix those body collisions. No collision pair
was waived to admit those candidates.

Four evidence-directed placements then retained the exact right-gripper/radio
relation, turned the held wrist 180 degrees, and moved it world +Y 100/200 mm
and +Z 150 mm (the declared robot base yaw is pi/2, so +Y is forward). Both
placements with left roll -90 degrees produced valid dual-arm endpoints and
connected sampled collision-free joint paths. The other two did not converge.
The base and gripper joints remain fixed; torso is explicitly auxiliary.

| Forward / upward shift | Right position / rotation error | Left position / rotation error | Left shoulder margin | Minimum selected-joint margin |
| --- | --- | --- | --- | --- |
| 100 / 150 mm | 1.653 mm / 0.002722 rad | 1.694 mm / 0.004693 rad | 0.235087 rad | 0.004324 rad |
| 200 / 150 mm | 1.057 mm / 0.001635 rad | 1.492 mm / 0.002947 rad | 0.158655 rad | 0.000253 rad |

The first candidate offers more joint margin and is selected for the next
development trial. The joint path is a pose-intent reposition, not a Cartesian
straight-line press; the right TCP may move while its retained radio relation
and carried meshes remain fixed. Native joint-path checks and independent
0.01-rad edge sampling include all external table/self/radio collisions.
Neither candidate needs base motion, hand swapping or joint-limit relaxation.

After updating both TCPs and the carried radio to each actual modeled endpoint,
the short left-finger press to 1 mm inward of the declared contact proxy passes
the existing dual-TCP Cartesian planner and nominal/effective admission. The
100-mm-forward candidate has 302 waypoints / 604 holding samples, maximum right
position error 0.015906 mm and rotation error 0.000036515 rad. The other has
295 waypoints / 590 samples, 0.020122 mm and 0.000039496 rad. Only the declared
left finger 1 / radio part 0 contact is admitted, within the existing corridor;
all other radio/finger/table collision pairs and true joint limits remain.
CPU admission is not physical contact or BDDL success.

The contained SDK checkpoint now includes a `reposition` phase using existing
`plan_to_poses` for both TCPs, followed by the existing checked stored-plan-ID
dispatch. It validates both endpoint errors at 2 mm / 0.005 rad, preserves
right grasp/contact feedback during motion, and keeps strict measured arrival
checks. The later press continues to constrain the right world TCP throughout.
56 focused tests, Ruff, scoped mypy and diff checks pass. No general planner
API or integration head changed.

A fresh-feedback operator smoke harness is staged separately in
`/tmp/behavior-radio-reposition-20261001`, preserving successful lift/video and
CPU records. It replays the proven grasp/lift, opens/closes the idle left gripper
to its explicit modeled 0.08 normalized setting, computes reposition targets
from the fresh right TCP and radio pose, recomputes press contact afterward,
and reads the original independent BDDL goal. Every intent remains explicitly
oracle-assisted development; the exact live posture must pass SDK planning,
effective command and measured-state checks again. No simulator/GPU run has
started for this candidate. The next step is a coordinated bounded smoke slot,
not another tuning search or an asserted task pass.

### First fresh-feedback reposition trial — stopped before pregrasp

The coordinated 3090 slot had 22,458 MiB free. A startup-only launch failed
because the disposable copy lacked the project-root marker used by the prior
successful harness; it fell back to a different model cache. Restoring the same
harness-only root-detection convention resolved the model to the already
accepted asset tree. Its startup log/exit record are retained separately.

The actual simulator launch loaded the official radio task and reported
`grasping_mode: assisted`. Markers stayed hidden and original task goals/dynamics
were preserved. The trial stopped during pregrasp planning: right finger 2's
measured float32 readback was 0.05000000074505806 m, just above the model's
0.05 m bound. No pregrasp, grasp/lift, reposition or press trajectory was sent.
This is a strict model-state admission failure, not a Cartesian residual,
execution timeout, PyTorch crash or CI failure. Independent goal 0 remained
unsatisfied. The SDK cancellation and normal app shutdown ran.

The harness now requests 0.99 right-gripper opening instead of the exact upper
boundary. No bounds, numerical tolerance or measured feedback are changed.
A CPU replay of actual failed-run joint/table/radio observations substitutes
only this hypothetical interior opening (float32 readback 0.0494999997317791 m).
The start is collision-free, pregrasp IK succeeds, and its connected two-waypoint
path passes independent sampled collision validation. This is preparation for
another live trial, not proof of live grasp/reposition/contact.

Because the accepted posture has only 0.00432 rad minimum selected-joint margin,
the development checkpoint now checks every selected measured joint against
its true model bounds before dispatch and during execution/arrival. It records
the nearest-bound margin and full measured joint sample. A violating sample
stops execution through the existing cancellation path; no gripper release is
issued on failure. Its cancellation regression passes; focused validation now
passes 57 tests, Ruff, scoped typing and diff checks.

Raw head/side-camera evidence and stage report are preserved under the delegated
workspace's `dimcode-grounding/bimanual-development/reposition-checkpoint`.
The short video contains initialization/opening, not a new action milestone,
so no new progress video was saved to Library. Owned launch/timeout/Python/
simulator PIDs were all absent after cleanup. GPU memory returned to 1,669 MiB
used / 22,458 MiB free. Compute is finished. The corrected harness is staged,
but another simulator trial requires a newly coordinated slot.

### CPU contact audit after the paired-feedback reposition trial

The most recent preserved trial is `behavior-radio-reposition-20261001e`.
Pregrasp, insertion, departure and lift completed. During reposition, at step
1094, the right second-finger contact disappeared and the existing guard
cancelled the trajectory. The press stage was never entered. The rejected
snapshot reports the original goal as unsatisfied. This is a development
execution/contact failure, not a CI failure or a review comment. No simulator
was started for this subsequent CPU audit; the owned simulator is absent.

The audit compares object motion with motion relative to the actual physical
right gripper, using paired snapshots rather than SDK FK from a later RPC.
From reposition start step 1021 to rejected step 1094, radio world displacement
is 190.400 mm and rotation is 21.021 degrees. Its change relative to the right
gripper is only 0.137 mm / 0.018 degrees. Thus world translation/rotation alone
does not establish a free flip or gross grasp slip. The first rejected sample
has one right-finger contact; the preceding accepted step 1091 has both. There
are no measured contact forces/normals, so these records cannot determine
whether a contact intermittently separated, an assisted constraint changed,
or another physical effect caused the loss. The evidence is preserved in
`dimcode-grounding/bimanual-development/reposition-paired-checkpoint/`
as the unchanged raw report and `cpu-stage-contact-analysis.json`.

Instrumentation now enriches every matching checkpoint observation, including
the first rejected sample, with stage displacement and physical gripper-relative
displacement. Press observations additionally report the calibrated left pad
in the **current** radio frame, signed outward gap, tangential offset, and the
candidate radio +X normal in world coordinates. This is an operator-calibrated
plane, explicitly not a measured collision normal. Same-owner-step full robot
base pose and official toggle-region diagnostics are retained to distinguish
nonplanar base tilt from planar-model FK or observation timing errors. These
fields remain evaluator-only diagnostics; they are not agent input or state
setting. Existing bounds, frozen-joint, collision, retention, deadline and
cancel-confirmed guards remain intact.

The passive owner snapshot additionally retains each physical finger link pose
and radio contacts with any body, including the supporting table. The installed
official `RigidContactAPI.get_contact_pairs` explicitly supports `with_set=None`
for this query. These fields allow reconstruction of collision-mesh placement
and identification of furniture contact during displacement. No impulse or
contact normal is synthesized. Simulator validation of the new passive fields
is still pending; CPU helper/checkpoint regression tests pass.

The geometry audit is still decisive: the official wxnicr asset contains 14
convex body parts, no articulated switch joint, and a toggle meta sphere. The
official implementation uses object/finger contact plus finger overlap with
that sphere for five consecutive steps, rather than measuring mechanical
switch travel. The observed trigger radius in the retained near-field trial
is 22.358 mm. The finger collision mesh has a distal 12 mm band roughly
13.56 mm wide; a TCP is not itself the contact pad. Existing calibrated pad
coordinates and the right-wrist-to-gripper fixed transform must therefore be
kept explicit. The candidate radio +X plane is not proof of a physical power
button or a camera-grounded affordance.

A viable next candidate must first demonstrate stable support, not push harder:
either maintain the verified opposing handle contacts through a smaller
collision-checked reorientation, or independently plan a table-supported
counter-support action by the other hand. Neither alternative is physically
validated yet. Once support is retained, recompute the surface/pad approach
from the latest radio transform, stop at precontact, then use the existing
short linear press corridor (maximum 2 mm admitted normal depth) while logging
both hands and all contact transitions. Do not freeze the radio, alter mass or
friction, relax collision/retention guards, or set `ToggledOn`. The current
large-turn candidate loses a contact before pressing and is not ready for
another GPU trial. CPU geometry/path admission and explicit compute
coordination are required first.

### One discriminating assisted-grasp replay

The authorized bounded replay `behavior-radio-assistance-20261002a` kept the
existing guard unchanged and excluded the press stage entirely. Passive owner
diagnostics now record official `is_grasping(candidate_obj)`, held-object
identity, release counter and attachment constraint path/validity for each arm.
They inspect the installed version's private bookkeeping without establishing,
releasing or modifying any constraint. After cancellation the operator sampled
15 stationary observations over approximately 1.5 seconds, then shut down.

At rejected step 1095, right finger 2 had no current contact. Nevertheless,
official `is_grasping` was TRUE (serialized `1`), the held object was the radio,
the release counter was null, and the assisted constraint was valid. Measured
radio/physical-gripper drift was 0.108 mm / 0.023 degrees. At the next recorded
step 1097 both contacts returned. All 15 stationary observations retained
official grasp/constraint status; maximum post-cancel relative drift was
0.061 mm / 0.0135 degrees. The original goal remained unsatisfied and no press
was entered. Known owned launch/timeout/Python/native PIDs were all absent
after exit; compute is finished. The evidence directory is
`dimcode-grounding/bimanual-development/assistance-checkpoint`, including the
raw report, `assistance-contact-verdict.json` and official lifecycle excerpts.

The installed official `robot.py` uses contact/raycast grasp windows to establish
assistance. Once established, `_handle_grasping` maintains the object while
grasp control continues; it does not require both fingers to contact at every
physics sample. `_release_grasp` deletes the constraint and starts the release
window. `is_grasping(candidate_obj)` checks object identity and absence of that
release window. The observed event is thus a transient contact-sample loss
under maintained official assistance, not demonstrated grasp release/drop.
Constraint validity alone would not prove stable dynamics; the measured
relative pose and recovered contact corroborate this specific episode.

A justified proposed mode-aware contract is: require opposing finger contact
and blocked closure when establishing the grasp; for ongoing **assisted-mode**
holding, require the same held object, official TRUE grasp state, no release
window, a valid attachment constraint, blocked closure and unchanged measured
attachment within the existing retention bounds. Preserve per-finger contact
transitions as evidence instead of using a single absent contact as synonymous
with release. Stop on lost assistance, drift, wrong object, stale evidence,
joint/collision failure or cancellation/timeout. Physical-mode holding must
not inherit an assistance-based exception. This contract is a proposal;
the live replay and production guard have not been relaxed.

A bounded CPU comparison used exact paired encoders/radio pose after the prior
successful lift, three IK attempts per candidate, unchanged geometry and
true limits. At the same +100 mm world-Y / +150 mm height placement and
left roll -90 degrees, 0 and 90 degree object turns did not converge. The
180 degree candidate had a collision-free connected sampled path, but its
minimum joint margin was only 0.0000193 rad. These outcomes do not establish
global infeasibility of a smaller turn, or readiness of the large one. No
further motion was issued from this CPU screen.

For the task goal, the simpler table-supported route deserves priority over
making a large carried-object rotation work. Retained earlier original-task
oracle-assisted table-supported trials independently reached BDDL success,
with radio displacement 6.70--6.88 mm and rotation 2.78--2.97 degrees rather
than a large flip. Their SDK `motion_completed` field was false when the
official successful episode terminated; this is not a proof of full planned
trajectory completion. Those records cannot substitute for a newly checked
path or stabilization action. A future small experiment should retain table
support, use a fresh-frame precontact pose and short approach, and inspect
object drift/contact before permitting press. Add a collision-checked
counter-support hand only if measured displacement requires it. No dynamics,
marker, limit, trigger or symbolic-goal changes are justified.

### Mode-correct continuation and original BDDL goal observed

The task-local `AssistedRadioRetention` contract is now implemented. Initial
admission requires opposing finger contacts, measured blocked closure and
official assistance for the radio. Continuation requires the same object,
official TRUE holding state, unchanged valid constraint identity and no release
window, plus measured rigid radio/gripper retention at the existing 4 mm /
0.03 rad bounds. SDK-FK retention, joint bounds, frozen joints and all collision
checks remain separate and unchanged. All contact transitions remain logged.
This validates the stock simulator's assisted mode, not a friction-only grasp.
Wrong object/state, detached/replaced constraint, release, physical-mode misuse,
translation/rotation drift and nonfinite pose/closure regressions pass. Focused
helper/checkpoint validation is 44 tests; Ruff and scoped mypy pass. A CPU
replay of 33 actual first-contact-loss snapshots passes the new contract,
including the one missing-finger sample, without changing any tolerance.

The first continuation launch (`20261002b`) stopped before departure: gripper
settling completed before the official assisted grasp window did. Passive
post-cancel observations show absence at step 700 followed by correctly
established assistance at step 702 and stable opposing contacts thereafter.
The operator harness now waits passively, at most two seconds, for strict
initial admission. It never moves while waiting and does not ignore failed
guards. The correction changes initialization timing, not the holding contract.

The resulting bounded continuation (`20261002c`) completed pregrasp, insertion,
departure and lift. Reposition then physically rotated the radio 179.91 degrees
and translated its center 154.49 mm. Across 169 recorded reposition samples,
47 had a missing right-finger contact sample; official assisted grasp and
measured retention remained valid. Maximum radio/physical-gripper relative
drift was 0.330 mm / 0.001407 rad (0.081 degrees). Minimum selected measured
joint margin was positive, 0.001184 rad. There was no left-finger/radio contact
in those samples and no changed dynamics, markers, limits or collision policy.

The independent original BDDL goal changed to satisfied at step 1482 while
still in reposition. The official toggle overlap counter progressed from 0
to 3 to 5; its observed trigger radius was 22.358 mm. Right grasp contacts made
the radio a finger-contact object. The official trigger uses contact with the
object plus any robot finger's sphere overlap, not necessarily contact by the
overlapping finger. These records are consistent with approaching fingers
entering the trigger region while the radio was held. They do not demonstrate
left physical button contact or mechanical switch travel. **The planned press
stage was not entered.** This is official task-goal success in an explicitly
oracle-assisted, custom-nearfield, stock-assisted-mode development trial;
it is not autonomous sensor-grounded benchmark performance.

The native runtime stops stepping/holds when `env.step` returns terminal or
truncated. After goal satisfaction, feedback ceased; the development supervisor
then rejected stale truth and confirmed cancellation. SDK measured arrival was
not complete: the last right TCP target residual was 3.393 mm / 0.006889 rad,
outside the existing 2 mm / 0.005 rad arrival thresholds. No threshold was
relaxed and the trajectory is not reported complete. Distinguishing terminal
task-goal events from motion-arrival feedback is the remaining orchestration
issue; another large-motion experiment is unnecessary to establish the observed
goal. Sensor-only selection of the hidden radio affordance remains unresolved
before agent-generated policy performance can be claimed.

Raw report/log and side-camera AVI/JSONL are retained in
`dimcode-grounding/bimanual-development/assisted-admission-checkpoint` with
`development-goal-verdict.json`. The derived 640x480 preview removes 24 initial
frames, preserves original stage captions, and encodes remaining frames at
4 fps (54.25 seconds). Source timestamps remain in JSONL; preview playback is
not a strict real-time clock. No scene/sensor pixels were relabeled as agent
observations. Library preview:
`libfile_2b63875276f88191b7be4ff00d59f160` /
`file_000000002d4c8230bdc024f36ba8d478`. Known owned launch/timeout/Python/native
PIDs were absent after shutdown. Compute is finished, and no further simulator
trial or branch publication was performed.

### Trigger attribution and explicit terminal outcomes (CPU only)

The Library ID suitable for attachment is
**`libfile_2b63875276f88191b7be4ff00d59f160`**; the `file_...` value is its backing
file ID. Its caption remains BDDL goal during reposition, no entered press or
left-finger/radio contact, and no autonomous/mechanical-button claim.

The original official overlap callback stores only a Boolean and stops after
the first valid robot-finger hit. It did not retain the hit body ID, so the
exact left finger encountered first cannot be recovered from the trace. A
CPU-only screen instead used the actual accepted R1 USD's enabled collision
meshes, their mesh-to-link transforms, retained physical finger poses and the
recorded trigger center/radius. The URDF refers to collision OBJ files absent
from this asset tree, so the screen did not silently substitute its visual
meshes. Existing installed USD bindings were loaded with process-local paths;
no simulator, installation, asset download or GPU initialization occurred.

At step 1477 the closest left collision shapes were 27.19 and 24.54 mm from
the trigger center, outside its 22.358 mm radius, with counter 0. At step 1480
they were 20.27 and 17.49 mm, inside, with counter 3. At successful step 1482
they were 16.61 and 13.44 mm, with counter 5. Right finger shapes stayed near
133.5 and 148.2 mm throughout these samples, far outside. Thus both left
fingers are geometrically eligible overlaps; the retained source/trace do not
identify which was reported first by PhysX. The held-object rotation did not
move the trigger onto the right holding fingers. CPU convex shape distance is
an attribution screen, not a rerun of the exact original PhysX query ordering.
`trigger-overlap-cpu.json` records solver residuals and all relevant samples.

This agrees with the official two-part predicate: right fingers make the
radio a currently finger-contacted object; either approaching left finger may
overlap its annotated sphere without colliding with the radio's body. The
five-step counter then toggles the official state and BDDL succeeds. No trigger
or goal was moved or optimized, and no new bimanual trial was run.

The task-local checkpoint now has an explicit `RadioEpisodeTerminalError`
outcome path, based on `BehaviorConnection.get_status()` and the same episode
ID. `TASK_GOAL_MET` requires a finished, terminated, nontruncated episode,
official episode and evaluator success flags, nonempty satisfied goal list
and no unsatisfied goals. It is **not** an SDK arrival result. A nonsuccessful
terminal episode is `EPISODE_ENDED`; ordinary SDK abort is `MOTION_CANCELLED`;
wrong episodes, disconnected/missing status, inconsistent terminal evidence,
runtime errors, physical failures and unconfirmed stops are `RUNTIME_FAULT`.
The supervisor cancels/stops outstanding motion before reporting its outcome.
It never synthesizes fresh robot feedback or marks motion complete after
termination. A stale feedback race can be classified only by separately
verified authoritative terminal status, never by missing feedback or an old
cached positive goal. A concurrent goal cannot override a physical protection
failure. Callers must handle this explicit event and stop subsequent stages;
they must not swallow it and continue pressing.

Focused tests cover success without reading stale feedback, terminal/feedback
races, missing terminal RPC, stale feedback without terminal evidence, wrong
episode, truncated/inconsistent success, ordinary cancel, physical failure
concurrent with goal, and unconfirmed cancellation. No simulator validation of
the new terminal path has been run. Sensor-only agent affordance selection is
a separate unresolved issue and no hint/marker restoration was performed.

### Final bounded terminal validation and local milestone handoff

One authorized replay (`behavior-radio-terminal-validation-20261002d`) used
the same custom-nearfield original task, unchanged grasp/reposition motion
intent and controller settings, stock assisted mode and hidden markers. No
goal, trigger, model, dynamics or joint/collision tolerance was changed.
The passive owner query logs all eligible PhysX sphere-hit body IDs without
modifying official toggle state or its counter. Its original callback first-hit
ordering remains distinct from this diagnostic query.

At step 1431, the official counter was 1 and the passive query hit only
`/World/scene_0/controllable__r1pro__robot_r1/left_gripper_finger_link2`.
At step 1433, counter 3, it again hit only that left finger. No right finger
hit was recorded. At step 1435 the authoritative episode status reported
success=true, terminated=true, truncated=false, reward=1 and original goal
0 satisfied with no unsatisfied goals. The new explicit `TASK_GOAL_MET` path
propagated this result, confirmed cancellation, and shut down normally with
exit code 0. It did not wait for stale motion observations. The report has
no generic runtime error, `motion_arrived=false` and `press_entered=false`.
Original-task success remains a sphere-overlap development result during
reposition, not an intentional/mechanical press or autonomous agent result.

All known owned launch/timeout/Python/native PIDs (1958854, 1958855, 1958856,
1959413) were absent after shutdown. Compute is finished. Raw report/log and
the concise `terminal-validation-verdict.json` are preserved under
`dimcode-grounding/bimanual-development/terminal-validation-checkpoint` in
the delegated workspace. No broader agent experiment or additional simulator
iteration follows this milestone.

Validation: 58 focused contact/checkpoint tests pass; all related radio/demo
tests pass (185 passed, 1 optional RoboPlan backend skip). An initial broader
run in the default sandbox failed because Zenoh could not open an ephemeral
loopback listener. Rerunning in the authorized test environment passed; no
host network/security settings changed. Ruff checks/format, scoped mypy and
diff checks pass. Production/test headers now use the repository license.
The milestone is preserved locally on `cc/feat/behavior-radio-sdk`, descended
from #4342's integration head; it is not pushed and integration PR refs are
unchanged. Physical intentional-press semantics and the sensor-only runtime
agent observation/affordance contract remain explicit decisions before the
next agent experiment.
