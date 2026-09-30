# sim2 Timing Comparison

This experiment asks two separate questions:

1. Does removing rendering and raycasting from the physics loop improve the
   timing delivered to a real DimOS controller?
2. Do those independent sensor loops need their own processes, or is a smaller
   worker arrangement competitive?

It does not change the production implementation or declare a winning design
in advance. Results are hardware-, scene- and workload-specific.

## Results: 2026-10-01

Apple M4 Max, 16 CPU cores, 128 GiB RAM, macOS 15.6, MuJoCo 3.10.0.
Production code: `6c48b8bba`, with the old engine retained from main base
`0a8979e10`. [measurements.csv](measurements.csv) contains the per-run values.
The table gives the range across two accepted repeats, not confidence intervals.

| Workload | Old physics Hz | Split physics Hz | Compact physics Hz |
|---|---:|---:|---:|
| Control only | 168.2-168.4 | 200.0-200.1 | 200.0 |
| Normal logistics | 146.5-146.7 | 200.0 | 200.0 |
| Heavy logistics | 109.5-110.7 | 200.0 | 200.0 |
| Heavy kitchen | 10.5-11.3 | 200.0 | 200.0 |

The requested physics rate is 200 Hz. Sim2's real-time factor was approximately
1.0 across the matrix; the old engine ranged from 0.84 without sensors to
0.052-0.056 in the heavy kitchen. This is integration throughput, not evidence
that sim2's `mj_step` computes physics faster.

| Normal logistics measurement | Old | Split | Compact |
|---|---:|---:|---:|
| Physics interval p99, ms | 16.77-16.87 | 6.07-6.13 | 6.09-6.10 |
| Command-write to physics-read p99, ms | 15.30-15.48 | 5.43-5.95 | 5.28-5.77 |
| Feedback age p99, ms | 14.62-14.83 | 5.13-5.54 | 5.49-5.55 |
| Known private memory, MiB | 1088-1120 | 1873-1929 | 1551-1553 |
| Worker processes | 2 | 6 | 2 |

The controller itself stayed near 50 Hz on every arrangement, with p99 tick
intervals around 24-25 ms. It is therefore misleading to claim that sim2 made
the policy loop dramatically faster. What improved was the physics cadence,
command consumption and feedback freshness. In the heavy kitchen, old physics
did not observe 781-798 of roughly 1,000 command updates per window; its latest-
value SHM overwrote intermediate commands while sensing blocked the next step.
Sim2 missed zero or one. Command latency alone would conceal these losses.

The sensor targets were **not fully met**. Normal runs delivered roughly
9.5-9.6 fresh RGB-D/lidar frames/s against a 10 Hz target. Heavy sim2 camera
delivery was about 18.4-18.6 Hz against 20 Hz. Kitchen lidar delivered only
12.9-13.3 Hz on either sim2 arrangement, versus 10.4-11.1 Hz on old. Thus this
is proof of useful timing isolation, not completion of the notebook's 30 Hz
camera/ten-minute acceptance target or a high-fidelity lidar claim.

**Architecture conclusion:** retain independent physics and sensor loops, but
do not defend one process per sensor as already proven optimal. Compact sim2
matched split closely here, saving about 320-380 MiB of known private memory.
It still has separate modules, threads and query models. Both sim2 arrangements
use substantially more memory than the old single-model integration. Sharing
a worker is not the same as sharing mutable `MjData` with physics.

### CPU Diagnostic

With default environment settings, sensor cases consumed about 13-14 CPU cores;
the no-sensor cases used about 0.3-0.4. Process samples localized most of the
native sensor-case CPU use to the lidar worker. Local library inspection found
Open3D's OpenMP pool configured for 16 threads.

Three separate ten-second normal-workload probes set only
`OMP_WAIT_POLICY=PASSIVE KMP_BLOCKTIME=0`. Old/split/compact then used
0.55/0.53/0.50 cores respectively, at similar sensor rates. Sim2 camera p99 age
rose from about 31-32 ms to 52-55 ms; old physics slowed further to about 0.62x.
This strongly implicates OpenMP idle waiting in the default CPU cost, but does
not establish a universally better thread policy or identify every call site.
These single diagnostic runs are separate from the repeated primary matrix.
No production environment defaults, thread pools or sensor algorithms changed.

### Evidence And Corrections

- There are 24 primary twenty-second trials: 16 sim2 trials and eight corrected
  old-backend trials. Three shorter OpenMP diagnostics are separate.
- The initial eight old trials placed the feedback probe after odom/IMU stream
  publications. This could produce negative ages. The probe was moved to the
  actual SHM writer and **only those eight trials were rerun**. Initial traces
  are retained as `superseded-feedback-probe`; their invalid feedback-age fields
  are excluded from the CSV. The old CPU diagnostic also excludes that field.
- The remaining one negative old age was -0.9 microseconds, a probe-ordering
  race at publication, retained rather than clamped. Sub-millisecond precision
  and coherent old split-array reads are not claimed.
- All accepted trials verified common model hashes, nonblank RGB-D, live
  requested streams, live final physics/control loops and pelvis height above
  0.5 m. This was stationary balance, not navigation or task success.
- All primary and diagnostic runs exited without forced worker termination.
  Old SHM cleanup printed `resource_tracker` unregister `KeyError`s after the
  measurement window. Those logs are preserved, not reported as clean shutdown.
  Both paths also emitted existing macOS GLFW/depth warnings.
- Effective rates and RTF count the **whole measurement window**, so a stall
  at its end cannot disappear from the denominator. CSV summaries are derived
  from the raw events with the final `summarize()` implementation.

Full local traces and logs are retained at
`results/2026-10-01-raw.tar.gz` (8,117,508 bytes), SHA-256
`342938fbe64bf9b9b9de94814671d974c2d5d325eaf959409573bb1893f71ad2`.
That generated archive is covered by the repository's existing `results/`
ignore rule and is **not committed**. The compact measurements and reproduction
harness are versioned. CSV rows retain each raw JSON's archive path and hash.
The first suite's arrangement order was reversed on repeat two; old-probe
correction runs occurred afterward, so the final matrix is not a fully
randomized/interleaved experiment. No significance or universal-win claim.

## Arrangements

All three use ordinary DimOS forkserver workers, the existing 50 Hz G1 GR00T
balance policy and arm hold task, a 5 ms MuJoCo physics timestep, and normal
typed sensor publications through a private local Zenoh router. There is no
mapping, planning, Rerun or native viewer in the controlled comparison.

```text
old
  worker 0: MujocoSimModule
              physics -> camera -> lidar -> relative sleep
              separate RGB-D / pointcloud publisher threads
  worker 1: ControlCoordinator / GR00T

split
  worker 0: sim2 physics, absolute-deadline pacing
  worker 1: RGB-D camera loop
  worker 2: lidar loop
  worker 3: robot connection publications
  worker 4: ControlCoordinator / GR00T
  worker 5: unused pool capacity

compact (experiment only)
  worker 0: identical sim2 physics
  worker 1: identical camera, lidar, connection and controller loops
            running on their existing independent threads
```

The normal DimOS allocator reserves ordinary worker capacity alongside
dedicated workers. Three dedicated modules therefore expand the requested
two-worker pool to six. The control-only cases need two workers on every
arrangement. Parent, forkserver and resource tracker count in the reported
process totals, but are not additional simulator modules.

`compact` changes only the camera/lidar subclasses' `dedicated_worker` flag.
It does **not** combine their models or data, change rates, alter the policy,
replace a loop, or introduce a new runtime. It also co-locates the controller
and connection with sensors, so it tests that complete two-worker arrangement,
not a pure camera-process-only ablation.

## Matching Work

Both engines load the same composed G1 and scene, including the same actuator
and IMU bindings. Each run verifies the compiled MJB hash against the common
input model before accepting results. Home positions, support placement,
integration method and timestep match. Startup timings exclude the common
preparation and are **not** an apples-to-apples scene-loading comparison: the
old engine opens the prepared MJB; sim2 composes the model during build.

The old and new G1 blueprint defaults are not equivalent. The benchmark uses
the old engine's actual pinhole ray fan on both sides, with equal pose, FOV,
ray count, range and self-exclusion. Camera and lidar mounts are numerically
checked before starting workers. Camera geometry visibility is made equal.
This is an ideal ray sensor comparison, not MID360 fidelity validation.

| Workload | Scene | RGB-D | Lidar |
|---|---|---|---|
| Control | Logistics | Off | Off |
| Normal | Logistics | 640x480 at 10 Hz | 8,192 rays at 10 Hz |
| Heavy | Logistics | 1280x960 at 20 Hz | 32,768 rays at 20 Hz |
| Kitchen | RoboCasa kitchen 1 | 1280x960 at 20 Hz | 32,768 rays at 20 Hz |

The suite runs each arrangement twice per workload, reversing arrangement
order on the second pass. Each trial has a five-second warmup and a separate
twenty-second measurement window. GR00T balances throughout; no task success
or walking-under-load claim follows from this experiment.

Production differences deliberately remain: old physics uses relative sleep;
sim2 uses an absolute deadline. The old RGB-D path has two renderers and two
scene updates, whereas sim2 reuses one. The old lidar publisher downsamples
at 1 mm here and has its existing independently timed publication loop;
sim2 publishes immediately after capture. Thus old-vs-split measures the whole
integration, **not** the isolated causal effect of multiprocessing. Split-vs-
compact holds the sim2 algorithms and pacing constant.

## Measurements

- Physics: actual step completion intervals, rate, p50/p95/p99/max, gaps over
  7.5 ms, and simulation-time/wall-time ratio. A 1x average can conceal stalls.
- Control: actual tick entry intervals and work time, not configured tick rate;
  gaps over 30 ms are reported separately.
- Command latency: beginning of an actual adapter SHM write to the physics
  owner's first read of that command sequence. This includes write cost and
  is **not** the time until a visible mechanical response. Unobserved commands
  are counted rather than silently discarded. sim2's currently unused
  `applied_action_sequence` field is not treated as evidence.
- Feedback age: physics publication to controller read, joined by observation
  sequence. The probe runs at the SHM publication, not after unrelated odom/IMU
  stream publications. The old split-array SHM protocol is not made coherent
  by this probe; concurrent publication can make this measurement approximate.
- Sensors: fresh source timestamps at an actual typed subscriber, not merely
  number of publisher callbacks. Duplicate old frames do not inflate delivery
  rate. Age includes render/raycast, publication and transport. Old source
  timestamps are joined to the corresponding post-physics snapshot; a missing
  old timestamp join is an error, not a substitute age measurement.
- Memory: RSS for the entire benchmark process tree, plus known unique-set
  size (USS). macOS blocks external USS inspection of children; active module
  processes report their own USS after measurement. Unreadable helper/idle
  processes remain explicitly listed. RSS sums double-count shared pages;
  known USS is a lower bound, **not** a complete physical-memory total.
- CPU: process-tree user+system seconds divided by wall seconds, in occupied
  CPU cores (one core = 100%). GPU time/driver allocations are not measured.
- Basic validity: matching model, live RGB-D/lidar streams, image dimensions
  and value statistics, and minimum G1 pelvis height. Raw traces remain
  available for checking stalls, timestamp joins and distribution summaries.

Instrumentation is confined to `instrumentation.py`: in-memory timing lists,
wrappers around existing methods, and one final trace RPC per active module.
It does not log or send RPCs per tick. Both variants import the same benchmark
dependencies, which adds common overhead; memory numbers are for this harness,
not a claim about the minimum possible production footprint. Runs use the
same interpreter and MuJoCo version. The old engine is the one retained at the
tested sim2 branch's main base, not an unverified claim about future main.

## Reproduction

From the core worktree with its installed environment and downloaded scenes:

```bash
.venv/bin/python -m experiments.sim2_timing.demo_benchmark suite \
  --seconds 20 --warmup 5 --output /tmp/sim2-timing-new-run
```

Or run one arrangement:

```bash
.venv/bin/python -m experiments.sim2_timing.demo_benchmark compact \
  --case heavy --scene robocasa-kitchen-1 \
  --seconds 20 --warmup 5 --output /tmp/sim2-timing-one-run
```

Use a new output directory. Each trial writes `result.json`; the suite also
keeps logs and updates `summary.json` after each completed trial. No existing
DimOS run is contacted. The private router isolates traffic without modifying
Zenoh or testing macOS multicast discovery. Unexpected trial failures stop
the suite, preserving earlier results.

## Decision Boundary

Faster physics with stale or missing sensors is not a complete win. A
dedicated-process policy deserves its cost only if it improves the joint
control/sensor outcome enough to justify extra resources. Similar compact
results would support independent loops, but would not establish that every
sensor needs a dedicated process.

This first matrix does not cover Linux/NVIDIA, smaller machines, multiple
robots, multiple camera counts, viewers, mapping load, long-duration thermal
behavior or validated high-fidelity lidar. Those are limits on generalization,
not reasons to infer results without measuring them.
