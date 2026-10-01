# xarm_table

A small world-only MuJoCo scene for the sim2 xArm7 evals: a table ahead of the
arm (top at z=0.13 m, x 0.30..0.60 m), a red ball `apple` (8 cm), an orange
ball `orange` (9 cm) and a cylinder `cup` (7 cm wide, 12 cm tall), all free
bodies, inside a plain room. It matches the layout described in
`dimos/evals/suites/mujoco_xarm.py` without needing the `sim2` LFS archive.

`scene.xml` holds no robot: sim2's `load_scene` attaches the arm itself.
`scene.json` names the movable entities, a `table/top` support region and a
`workbench` spawn at the world origin, which is where the xArm7's `link_base`
sits in the existing eval scene.

```bash
MUJOCO_GL=egl dimos --simulation mujoco --transport zenoh \
  --scene-package dimos/evals/scenes/xarm_table/scene.xml \
  run xarm7-planner-coordinator --simulationmodule.viewer=false
```
