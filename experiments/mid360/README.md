# Sim2 Mid360 Versus A Real Mid360

Measured on 2026-10-06. **The previous firing table was wrong for this device:
it swept in the opposite horizontal direction and repeated every four seconds.**
The corrected implementation uses Andrew's continuous Fourier geometry,
with two rotor rates adjusted using a short hardware recording. Timing and
Point-LIO integration work; material response and IMU realism remain limited.

This is a comparison against **one physical G1-mounted Mid360 in one room**,
not a claim of universal sensor calibration or real-world localization accuracy.
The robot was not commanded to move. No sensor settings, drivers or services
were changed. The two recordings were separate passive multicast captures.

## What Changed

Only the firing model and its data changed in production:

- [Mid360](../../dimos/sim2/sensors/lidar/models/mid360.py) now evaluates a
  continuous four-channel Fourier model at absolute acquisition time.
- It retains Andrew's PR #4441 coefficients and the same return-noise model.
  There is no competing reference/fitted runtime mode or fallback.
- Two rotor rates changed from **181.083626 / 9.9 Hz** to
  **181.08313465387945 / 9.899443879866606 Hz**. These are measured approximations,
  not factory constants. Starting simulation phase remains zero.
- The replacement coefficient NPZ is **8,640 bytes**, in a **9,336-byte archive**,
  replacing the approximately 3 MB reference-table archive.
- Public device configuration, rolling scene acquisition, timestamp fields,
  workers, IMU, control, transports, mapper and planner are unchanged.

Andrew's model already described this hardware substantially better than the
retained Livox table. This work reuses that model, rather than claiming a new
scan-pattern invention. Sim2 supplies rolling world/robot raycasting and the
existing actual Point-LIO composition around it.

## Hardware Measurements

Each packet contained 96 Cartesian-high point slots. Slot timestamps were
5,000 ns apart; point packets arrived in sensor time every 480,000 ns.
Both recordings had continuous point-packet counters, with no missing or
reordered point packets detected. Counts use sensor timestamps, not SSH or
host arrival time. The device used its boot-relative clock, not UTC.

| Measurement | Capture A | Independent Capture B | Simulation |
|---|---:|---:|---|
| Duration | 12.00048 s | 12.00096 s | Configurable |
| Point slots | 2,400,096 | 2,400,192 | 20,000 per 100 ms |
| Firing slots/s | 200,000 | 200,000 | 200,000 |
| Returns with range >=0.16 m | 1,593,164 | 1,596,230 | Scene-dependent |
| Such returns/s | 132,758 | 133,009 | Not expected to match another room |
| IMU samples | 2,400 | 2,400 | 200/s |
| Mean IMU rate | 199.990 Hz | 199.988 Hz | 200 Hz |
| IMU interval minimum/maximum | 3.967 / 6.005 ms | 4.018 / 5.996 ms | Exactly 5 ms |
| Observed return elevation | -6.900 to 52.151 deg | -6.900 to 52.148 deg | About -7.42 to 52.00 deg in the first scan |

Return counts above include flagged returns, so they are not synonymous with
high-confidence usable points. Angular fitting uses only tag=0 returns farther
than 0.5 m. Missing returns are not converted into guessed directions. Observed
elevation limits are constrained by what this room returned, not a full optical
field-of-view calibration.

### The Two Firing Defects

Compare successive observations of the same laser channel, 20 microseconds apart:

| Median azimuth increment | Degrees |
|---|---:|
| Real hardware, capture B | +1.313 |
| Previous reference table | -1.320 |
| Corrected model, first 100 ms | +1.309 |

This is in raw sensor coordinates. A read-only SDK2 query confirmed zero
installation extrinsics, so it is not an unnoticed robot mounting transform.
The old table's conversion also follows its upstream convention; blindly
flipping its Y coordinate did not produce a stable time-aligned match.

The previous implementation replayed 800,000 directions, giving an **exact
four-second repeat**. The real device's valid directions four seconds apart
differed by a median **98.08 degrees** in capture B. The replacement evaluates
rotor phase continuously and does not wrap a four-second lookup table.

[Old reference pattern](mid360-100ms.png) |
[Corrected full 100 ms firing pattern](continuous-100ms.png) |
[Numeric comparison](pattern-comparison.json)

## Held-Out Angular Comparison

We compare unit directions, not distances, so the room geometry is not needed
to assess firing angles. Integer-millimetre point quantization still contributes
to measured angular error, particularly at shorter distances.

1. Freeze Andrew's four-channel shape coefficients. No shape coefficients or
   individual rays are fitted to these captures.
2. Choose 1,200 tag=0 returns beyond 0.5 m using RNG seed 0. Fit two starting
   phases and two rotor frequencies on **capture A, seconds 0-4 only**.
3. Keep those parameters fixed for A's remaining eight seconds.
4. Keep the geometry and frequencies fixed for the independently recorded B.
   Fit **only two unknown starting phases** on B's first second, then hold them
   fixed. There is no alignment per scan or per validation window.
5. Score 12,000 eligible returns in each interval. Errors are angular separations
   between the measured unit direction and the corresponding predicted laser.
   Fit-interval scores are not held-out evidence.

| Recording / scored interval | Median error | 95th percentile | Maximum in scored sample |
|---|---:|---:|---:|
| A, 4-8 s, held out | 0.487 deg | 1.207 deg | 2.017 deg |
| A, 8-12 s, held out | 0.703 deg | 1.932 deg | 2.703 deg |
| B, 1-4 s, held out | 0.446 deg | 1.042 deg | 1.445 deg |
| B, 4-8 s, held out | 0.549 deg | 1.154 deg | 1.795 deg |
| B, 8-12 s, held out | 0.783 deg | 1.364 deg | 1.934 deg |

For comparison, Andrew's **unadjusted frequencies**, with the same phase-only
alignment on B's first second, give **1.793 deg median / 3.093 deg p95** in
B's final four seconds. The small rate correction reduces accumulating phase
error on this device; it does not establish the same improvement on other units.
Residual error still grows over time, so exact long-run synchronization is not
claimed.

![Held-out hardware comparison](validation.png)

**Both top plots deliberately use the hardware's retained firing indices.**
Their identical holes are not a simulated dropout result. This isolates the
angular comparison from return physics. The separate full-pattern image above
shows all 20,000 simulated firing directions.

Machine-readable evidence: [A calibration](calibration.json),
[B validation](validation.json), [unadjusted model on B](andrew-unadjusted.json).

## What This Does Not Validate

- **Range/material response:** we do not have a geometrically matched room or
  known reflectivity targets. Return coverage, range noise, grazing dropout,
  glass, dark surfaces and intensity are not calibrated by this experiment.
  Simulated range/dropout coefficients remain source-derived approximations.
- **IMU realism:** simulation has ideal specific force/angular velocity and
  fixed timestamps. Capture B's mean gyro was approximately
  `[0.0135, -0.0118, -0.0363] rad/s`, with per-axis standard deviations
  `[0.00559, 0.00495, 0.00611] rad/s`. Those are observations, not isolated bias
  or white-noise parameters: we did not establish a vibration-free stationary
  reference. Do not paste them into the simulator as a calibration.
- **Mounting:** the real IMU's negative Z gravity reading is consistent with an
  inverted mounting. This does not measure lidar-to-IMU extrinsics. Sim2 still
  uses a colocated device IMU, and the G1 optical-window exclusion also excludes
  head-shell occlusion. Other robot links occlude normally.
- **Timing fidelity:** real IMU intervals have two clusters around 4.25 and
  5.75 ms. Adding independent random jitter would not reproduce that temporal
  structure. Delays, packet loss and long-run clock drift are not modeled here.
- **Transfer:** this is two short captures from one sensor. It is not a moving
  hardware SLAM evaluation or proof that simulated estimator accuracy transfers.

## Simulation Integration Check

After changing the pattern, ran the actual existing G1 GR00T blueprint with
rolling Mid360, RGB-D, native Point-LIO, ray-tracing mapping and ordinary Zenoh,
headless, for 15 seconds of simulated walking/turning. Truth was read separately
by the probe, aligned once, and never supplied to the estimator. Pose pairs
matched within 25 ms. These are integration measurements, not hardware accuracy.

| Measurement | Result |
|---|---:|
| Startup | 2.19 s |
| Real-time factor | 0.99972 |
| Lidar / IMU rate | 9.977 / 200.011 Hz |
| Recorded scan / IMU drops | 0 / 0 |
| Last complete 20,000-ray capture | 15.25 ms |
| Matched estimated/truth poses | 151 |
| Simulated displacement | 1.263 m |
| Position error RMS / maximum | 2.94 / 6.16 mm |
| Maximum orientation error | 0.147 deg |

Seventy sim2 tests pass, including real MuJoCo rolling acquisition. Tests cover
Fourier evaluation against an independent real sine/cosine calculation, channel
order, absolute-time continuity, no artificial four-second repeat, retained
downsample timing, history/IMU acquisition and raw-field publication. The native
processes exited normally. The existing macOS clip-control warning remains;
no renderer, mapper, control or Zenoh changes were made for this study.

## Reproduce

Run from this branch's DimOS checkout. `dpkt` is an offline analysis dependency,
not a new runtime dependency. Raw room captures stay outside the repository.

```bash
STUDY="$HOME/Desktop/sim2-mid360-study-20261006"
uv run --no-sync --with dpkt python experiments/mid360/analyze.py \
  "$STUDY/g1-mid360-20261006-multicast.pcap" \
  --output /tmp/mid360-calibration --train-seconds 4 --calibrate \
  --frequencies 181.083626 9.9

uv run --no-sync --with dpkt python experiments/mid360/analyze.py \
  "$STUDY/g1-mid360-20261006-validation.pcap" \
  --output /tmp/mid360-validation

uv run --no-sync --with dpkt python experiments/mid360/analyze.py \
  "$STUDY/g1-mid360-20261006-validation.pcap" \
  --output /tmp/mid360-unadjusted --frequencies 181.083626 9.9

uv run --no-sync python -m dimos.sim2.demo_mid360_pattern
uv run --no-sync python -m dimos.sim2.demo_pointlio --move --seconds 15
uv run --no-sync pytest dimos/sim2 -m 'not self_hosted' -q
```

[capture.py](capture.py) is the standalone Linux standard-library capture
helper, with explicit interface, existing multicast group, host IP and sensor
IP arguments. It joins existing point/IMU multicast destinations, records only
that sensor's traffic and closes its sockets; it sends no Livox commands.
It requires raw-socket permission and untagged Ethernet/IPv4, using the
observed default point/IMU ports. The analysis rejects missing/reordered point
packets rather than inventing a channel sequence. Do not launch a competing
SDK driver or change the sensor's destination to collect this data.

```bash
sudo python3 capture.py INTERFACE /tmp/mid360.pcap --seconds 12 \
  --group EXISTING_MULTICAST_GROUP --host-ip INTERFACE_IP --sensor-ip LIDAR_IP
```

## Provenance

- Original implementation: DimOS `7465a7b2f`, native `pim/feat/sim2-core`.
- Original table: PimSim `0f040e3`, derived from Livox's `mid360.csv`;
  retained NPZ SHA-256
  `24ab19091f424acc1e271a4ac33d40f8618a2698470703d90603fab71c9aaac2`.
- Andrew's coefficients and method: DimOS PR #4441 at
  `63a9684d4505f3f54d0515c83e1b5fdb29ba4bdc`,
  `dimos/simulation/sensors/mid360/pattern.py` and
  `data/go2_sim/mid360_pattern.npz`. Attribution is also in the data archive.
- Corrected NPZ SHA-256:
  `d17511cae43c5c23d51bb4004045ae23b4f13062de6ccace5b000f833e70e2b1`.
- Capture A: 36,234,662 bytes, SHA-256
  `088fac8a4ac81e9258125fbe512153cefcb51ee73c92ca0526c265cf0f426173`.
- Capture B: 36,236,100 bytes, SHA-256
  `ccbc55e44f4d45aa30ee7412902826a3f0af72af697641b36385b144e678d75b`.

The repository contains reduced angular/statistical evidence, not raw room
geometry, access credentials or private Go2 policy weights. Reproducing the
exact hardware numbers requires access to those local captures.
