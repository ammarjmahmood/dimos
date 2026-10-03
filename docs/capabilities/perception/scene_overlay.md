# Scene overlay

Rebuild a tabletop scene the same way every time, without markers. The overlay
module draws a saved reference image half-transparently over the live camera
view, and the operator moves objects until they line up with the reference.
This is the scene replication method of SceneReplica (arXiv 2306.15620).

It only works while the camera is fixed relative to the robot. Mount the camera
rigidly, keep its resolution and exposure settings constant, and verify it has
not moved before a session (fiducial markers on the table are a good check).

## Run

One camera, `SceneOverlayModule` and a Rerun bridge. The module subscribes to
`color_image` and publishes `overlay_image`; Rerun opens with the camera's 3D
view on the left and the overlay full size on the right. Hide either side
with its eye icon. On a robot computer without a display, pass
`--rerunbridgemodule.rerun-open none` and connect `dimos-viewer` from your
laptop using the addresses the bridge prints.

```bash
# fixed RealSense over the table
dimos run scene-overlay-realsense --realsensecamera.serial-number <SERIAL>

# any webcam
dimos run scene-overlay-webcam --cameramodule.hardware.camera-index 0
```

Add `SceneOverlayModule.blueprint()` to a robot blueprint that already has a
camera to get the same stream inside a full stack.

## Capture and switch scenes

From a second terminal:

```bash
dimos shell
```

```python skip
overlay = app.SceneOverlayModule

overlay.capture_reference("scene-01")   # saves the current frame as scene-01
overlay.set_scene("scene-02")           # False when scene-02 has no reference yet
overlay.list_scenes()
overlay.set_opacity(0.3)                # 0.0 shows only the live view, 1.0 only the reference
overlay.set_mode("edges")               # reference edges in green instead of a blend
overlay.get_status()
```

Until a scene has a reference the live image passes through with a label
saying so. References live under `~/.local/state/dimos/scene_overlay/` as
`<scene>.png` with a `<scene>.json` sidecar recording the frame size, frame id
and capture time. Set `--sceneoverlaymodule.scenes-dir` to keep them with a
benchmark instead.

## Options

| Option | Default | Meaning |
|---|---|---|
| `scene` | `scene-01` | Scene loaded at start |
| `opacity` | `0.5` | Reference weight in blend mode |
| `mode` | `blend` | `blend` or `edges` |
| `max_fps` | `10` | Overlay publish rate cap; `0` for every frame |
| `label` | `true` | Draw the scene name and mode on the image |

A reference of another size than the live frame is resized to fit and a
warning is logged once; capture references at the resolution you will run.
