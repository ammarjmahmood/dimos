# RoboPlan 0.7 migration

The migration uses RoboPlan 0.7.0, Pinocchio 4.1.0, Coal 3.0.3 and
cmeel-tinyxml2 11.0.0. Scene construction loads a URDF description, then imports
SRDF and the prepared model's velocity and acceleration limits through the
supported YAML interface. This avoids infinite acceleration limits when the
URDF parser ignores nonstandard acceleration attributes.

## Point clouds

`PointCloudSelfFilter` uses `RobotBodyFilter.computeMask` with `Narrowphase` and
one native worker. Its existing cloud and clear-mask outputs remain unchanged.
The added `coordinator_joint_state` input receives canonical model joint names.
States are copied into a bounded time window and matched to the capture timestamp
within `state_tolerance_s`; there is no fallback to the latest configuration.
Missing, malformed or stale state drops the whole capture. Fixed robots do not
need joint state. Out-of-order captures do not replace map-clear history.

Scalar joints map to full native configuration indices; continuous joints use
cosine/sine coordinates. Pinocchio derives mimic joints from their source.
Planar/floating joints use capture-time parent-to-child TF after removing the
joint origin. Capture-time TF also maps sensor points into the URDF root frame
and maps clear-mask samples into the configured world frame. The grasp blueprint
uses a 100 ms state tolerance alongside its existing 100 ms TF tolerances.

A lock covers native filter scratch, history and paired output publication.
Per-point ancillary fields retain their order and values. World voxels use floor
quantization and cell centers; clearing still includes the current and previous
robot volume. Volume samples remain separate from sensor classification.

Coal's triangle-mesh Narrowphase query removes points near mesh surfaces, but
does not classify a closed mesh's entire interior as solid. Mesh volume sampling
is retained for clear masks so this upstream behavior does not leave interior
map cells uncleared. PaddedObb is not enabled; it can remove nearby obstacles at
bounding-box corners.

## Planning contexts

Each DimOS scratch context owns a native `SceneContext` for FK, collision queries
and partial-to-full configuration conversion. Geometry changes recreate stale
contexts; placement-only updates keep them. These queries no longer change the
Scene's current configuration. Contexts cannot be reused across worlds.

The scene lock still excludes geometry updates while queries execute. Native
planners, TOPPRA and the Python Jacobian/path bindings retain their existing
locks. This layer improves scratch isolation; it does not claim parallel query
throughput while those consistency locks remain.

## Validation

CPU validation on Linux x86_64 / Python 3.12.14 / AMD Ryzen 7 8700F:

- 103 focused tests: world/planner adapters, point-cloud alignment and fields,
  geometry refresh, native RRT, Cartesian and TOPPRA, and parametrizer validation.
- 5 blueprint generation tests; generated registry unchanged.
- Focused mypy for the three changed production adapters; changed-file pre-commit
  checks, including lock, LFS and branch checks.
- Real STL surface filtering and interior clearing; synthetic fixed, prismatic,
  continuous, mimic, planar and floating models; real xArm Cartesian fixtures.

For a fixed sphere fixture with 100,000 uniformly distributed points (seed 7),
radius 0.1 m, padding 0.01 m and voxel pitch 0.05 m, full filtering and mask
construction were measured over 50 iterations after five warmups. Both versions
removed the same 69 points and produced identical clear masks.

| Version | p50 (ms) | p95 (ms) |
| --- | ---: | ---: |
| main 943ce13c | 2.11 | 2.37 |
| migration, one native worker | 2.85 | 3.23 |

This simple primitive fixture is slower after migration. It does not establish
performance for dense real robot mesh scenes or hardware streams.

Lock generation and pre-commit used the repository-supported uv 0.9.25. uv
0.12.21 rejected a fresh universal resolution because the existing a750-control
constraint admits Python 3.10/3.11 while its available wheel is cp312-only.
No unrelated constraint was changed.

Hardware, GPU, Windows, macOS, ARM, MuJoCo and the full self-hosted suite were not
run. The selected native tests marked self_hosted were explicitly run on CPU.
