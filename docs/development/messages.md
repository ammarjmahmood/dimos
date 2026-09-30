# Add and use a message

This CDR stack lets an application own its message definitions. A ROS2 `.msg`
file is the source of truth; the same generator produces Python bindings,
C++ structs and native Rust structs. ROS is not required. Generated types carry
fields, `encode`/`decode`, a qualified `msg_name` and complete `schema` metadata.
Geometry, time, NumPy and viewer conversions stay in separate helpers.

## I want to add a new message type

Start with [`DeviceReading.msg`](/examples/message-codegen/user-story/story_msgs/msg/DeviceReading.msg).
Its directory gives it the identity `story_msgs/msg/DeviceReading`. It contains a
standard Header plus an application sequence, value and default label. Use your
own package namespace; conflicting definitions of one identity are rejected.
Do not edit the generated sources or add the type to a handwritten registry.

Install the development dependencies and pinned Fast CDR as described in the
[message example](/examples/message-codegen/README.md). With Python, C++17,
CMake and Rust available, run from the checkout root:

```bash
bash scripts/test_message_user_story.sh
```

This generates only the new type and its dependency closure, builds all three
languages, and runs Python module → C++ → Rust → Python through CDR files.
Cargo runs offline: its dependencies must already be cached during setup.
Expect sequence 42, value 23.5, label `new-local-type/python/cpp/rust`, and the
original nanosecond timestamp. The process exits after the exchange; it does not
start a robot. Outputs and terminal evidence remain under
`build/message-codegen/user-story/`. These ignored outputs can be removed when
no longer needed.

The generator command is explicit and can also emit a reusable Python package:

```bash
.venv/bin/python -m dimos.message_codegen.generate \
  --package-root examples/message-codegen/user-story \
  --type story_msgs/msg/DeviceReading --python-module story_messages \
  --package --output build/message-codegen/story-package
```

The generated `python/` project builds a wheel/sdist with schema resources and a
`dimos.messages` provider. Its CMake target is `story_messages::messages`; its
Cargo package is `story-messages-messages`. Use ordinary pip/build, CMake and
Cargo tooling, as in the installed external-application story in the example
README. Install packages at setup time. Runtime does not download schemas or
invoke the generator. Adding a field requires rebuilding and distributing the
matching package to all producers and typed consumers; publishing is optional.

## I want to use the type in a Python module

[`ReadingProcessor`](/examples/message-codegen/user-story/demo_module.py) is the
runnable module in the command above. Import from the generated application
package, annotate `In[DeviceReading]` and `Out[DeviceReading]`, and implement
`handle_reading`. The module handler receives ROS-shaped data directly. The demo
calls the handler and subscribes to its output in-process, then crosses actual
CDR boundaries through the native file consumers; it does not exercise worker
startup or transport discovery. In a running blueprint, connect modules through
[typed streams](/docs/usage/modules.md) and install the message package in every
worker environment.

The handler decodes an encoded copy before editing it, leaving its input intact.
Nested message fields are live; primitive sequences are live. Elements of nested
message sequences are values: assign an edited element back into the sequence.
Image/cloud borrowed views are read-only and retain their storage owner. Request
a copy for mutable processing; resizing borrowed storage raises `BufferError`.

Keep generated fields explicit: `message.header.stamp.sec/nanosec`,
`message.header.frame_id`, and `pose_stamped.pose.position`. Use the
[time helpers](/dimos/msgs/time.py), [geometry helpers](/dimos/msgs/geometry.py),
[image helpers](/dimos/msgs/image.py) and [point-cloud helpers](/dimos/msgs/pointcloud.py)
for operations; generated types do not have old rich-message convenience methods.

## I want to use the type in C++

[`consumer.cpp`](/examples/message-codegen/user-story/consumer.cpp) builds and
runs in the same script. Include the generated `messages.hpp`, use
`story_msgs::msg::DeviceReading`, and call `dimos::cdr::decode<Type>` and
`dimos::cdr::encode`. The example prints and changes real fields before writing
the next CDR file. Installed consumers include `story_messages/messages.hpp`
and link the exported CMake target.

For a native dimOS module, follow the actual
[C++ CDR relay](/examples/native-modules/cpp/src/cdr_relay.cpp): derive `Module`,
register the typed input with `Builder::input`, retain an `Output<Type>`, and
publish the generated struct in its handler. The SDK selects the CDR codec for
generated types. Link `dimos_native` and your generated CMake message target.
The native SDK additionally needs its pinned Zenoh C/C++, raw LCM, JSON and PFR
build dependencies; compiling the file consumer alone does not verify that SDK.

## I want to use the type in Rust

[`consumer.rs`](/examples/message-codegen/user-story/consumer.rs) also builds
and runs in the script. Import `story_msgs::msg::DeviceReading` and the generated
`codec::Message` trait. Call `DeviceReading::decode` and `message.encode`;
owned fields use ordinary Rust mutation. The default encoder emits little
endian encapsulated XCDR1 and the decoder accepts the supported big endian form.

For a native dimOS module, follow the actual
[Rust CDR relay](/examples/native-modules/rust/src/cdr_relay.rs): derive `Module`,
annotate `Input<Type>` with `decode = cdr::decode` and `Output<Type>` with
`encode = cdr::encode`, then implement `handle_<input>` and publish asynchronously.
Add the generated message crate alongside `dimos-module` in Cargo dependencies.
The recorder does not need a newly compiled decoder for each custom type.

The [native relay demo](/examples/message-codegen/demo_native.py) runs the
existing Python → C++ → Rust typed streams on LCM and Zenoh after the SDK builds.
See the example README for exact build/run commands and dependencies. LCM large
images exercise fragmentation. The file user story above proves message and
codec use; the native relay proves SDK launch and transport use. Full coordinator
and viewer acceptance is tracked separately in
[the migration checklist](/openspec/changes/replace-lcm-message-encoding/tasks.md).

## I want to record, inspect and replay it

Installed providers expose each qualified type and its complete concatenated
schema. The runtime passes this metadata through recorder stream configuration.
MCAP stores the ROS2 profile, `cdr` channel bytes and `ros2msg` schemas, with chunk
compression. Foxglove and Rerun can inspect fields using embedded definitions,
without installing your application package. Typed dimOS replay needs the matching
installed message package; unknown types can still be accessed as bytes.
Source stamps are used for recognized stamped layouts, while unknown or unstamped
layouts use reception time. MCAP log time always reflects reception.

This is an intentional API and wire break. LCM remains a raw transport; its old
message encoding and recordings are not a supported interchange format for the
new stack. Mixed old/new deployments, automatic legacy recording conversion,
ROS graph integration and runtime schema-hash enforcement are outside this change.

## I have a recording from before the CDR cutover

Historical SQLite `lcm`, `lz4+lcm`, and private `jpeg` codec streams are an
intentional compatibility break. The current reader rejects them with an
actionable error; it does not reinterpret those bytes as CDR or silently
substitute a legacy decoder. MCAP channels using `cdr` with complete `ros2msg`
schemas and new SQLite `cdr`/`lz4+cdr` streams are the supported message paths.

Preserve the old recording. Export its contents using the original compatible
checkout, or record the source again with the installed generated message
package. This branch does not provide an in-place legacy recording converter.
For an offline example, use
[`write_demo_recording`](/dimos/memory/demo_data.py) to create a new deterministic
CDR image/pose/cloud recording; it refuses to overwrite an existing file and
does not download data or models. It contains no learned embedding stream.
