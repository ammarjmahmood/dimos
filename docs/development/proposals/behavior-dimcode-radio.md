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
