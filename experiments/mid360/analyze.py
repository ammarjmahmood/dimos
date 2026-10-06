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

"""Offline angular/timing study, not a driver or runtime calibration service."""

import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import struct
from typing import Any

import dpkt
from matplotlib.backends.backend_agg import FigureCanvasAgg
from matplotlib.figure import Figure
import numpy as np
from numpy.typing import NDArray
from scipy.optimize import least_squares
from threadpoolctl import threadpool_limits

from dimos.sim2.demo_mid360_pattern import angular_points
from dimos.utils.data import get_data


def read_capture(path: Path) -> tuple[dict[str, NDArray[Any]], dict[str, Any]]:
    point_type = np.dtype([("xyz", "<i4", (3,)), ("reflectivity", "u1"), ("tag", "u1")])
    packets, imu = [], []
    time_types: Counter[int] = Counter()
    with path.open("rb") as source:
        for _, frame in dpkt.pcap.Reader(source):
            ethernet = dpkt.ethernet.Ethernet(frame)
            if not isinstance(ethernet.data, dpkt.ip.IP):
                continue
            udp = ethernet.data.data
            if not isinstance(udp, dpkt.udp.UDP) or udp.sport not in (56300, 56400):
                continue
            raw = udp.data
            if len(raw) < 36:
                raise ValueError("Truncated Livox header")
            length, span, count = struct.unpack_from("<HHH", raw, 1)
            if length != len(raw):
                raise ValueError("Livox payload length disagrees with header")
            counter = struct.unpack_from("<H", raw, 7)[0]
            stamp = struct.unpack_from("<Q", raw, 28)[0]
            time_types[raw[11]] += 1
            if raw[10] == 0 and count == 1 and length == 60:
                imu.append((stamp, np.frombuffer(raw, "<f4", 6, 36).copy()))
            elif raw[10] == 1 and count == 96 and length == 36 + 14 * count:
                interval, remainder = divmod(span * 100, count - 1)
                if remainder:
                    raise ValueError("Non-integral point interval")
                packets.append(
                    (stamp, counter, np.frombuffer(raw, point_type, count, 36), interval)
                )
            else:
                raise ValueError(
                    "Study expects Cartesian-high point packets and single IMU samples"
                )
    if len(packets) < 2 or len(imu) < 2:
        raise ValueError("Capture must contain points and IMU")
    times = np.array([p[0] for p in packets], dtype=np.int64)
    counters = np.array([p[1] for p in packets])
    intervals = np.array([p[3] for p in packets])
    if np.any(np.diff(counters) % 65536 != 1) or np.any(np.diff(times) != 96 * intervals[:-1]):
        raise ValueError("Missing/reordered point packets: cannot infer continuous channel indices")
    xyz = np.concatenate([p[2]["xyz"] for p in packets]).astype(np.float64) / 1000
    stamps = np.concatenate([p[0] + np.arange(96, dtype=np.int64) * p[3] for p in packets])
    tags = np.concatenate([p[2]["tag"] for p in packets])
    imu_stamps = np.array([p[0] for p in imu], dtype=np.int64)
    imuv = np.array([p[1] for p in imu], dtype=np.float64)
    valid = np.linalg.norm(xyz, axis=1) >= 0.16
    seconds = (stamps[-1] - stamps[0] + intervals[-1]) / 1e9
    summary = {
        "capture": path.name,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "bytes": path.stat().st_size,
        "sensor_seconds": seconds,
        "point_packets": len(packets),
        "point_slots": len(xyz),
        "slots_per_second": len(xyz) / seconds,
        "point_intervals_ns": sorted(set(intervals.tolist())),
        "counter_increments": dict(Counter((np.diff(counters) % 65536).tolist())),
        "time_types": dict(time_types),
        "returns_ge_016m": int(valid.sum()),
        "returns_per_second": float(valid.sum() / seconds),
        "tag_counts": dict(Counter(tags.tolist())),
        "observed_elevation_min_max_deg": np.quantile(
            angular_points(xyz[valid])[:, 1], [0, 1]
        ).tolist(),
        "imu_samples": len(imu),
        "imu_hz": (len(imu) - 1) / ((imu_stamps[-1] - imu_stamps[0]) / 1e9),
        "imu_interval_ms_min_median_max": np.quantile(
            np.diff(imu_stamps) / 1e6, [0, 0.5, 1]
        ).tolist(),
        "imu_mean_gyro_rad_s": imuv[:, :3].mean(axis=0).tolist(),
        "imu_std_gyro_rad_s": imuv[:, :3].std(axis=0).tolist(),
        "imu_mean_acceleration_g": imuv[:, 3:].mean(axis=0).tolist(),
        "imu_std_acceleration_g": imuv[:, 3:].std(axis=0).tolist(),
    }
    return {"xyz": xyz, "stamps": stamps, "tags": tags, "imu_stamps": imu_stamps}, summary


def analyze(args: argparse.Namespace) -> None:
    data, summary = read_capture(args.capture)
    xyz, stamps = data["xyz"], data["stamps"]
    times = (stamps - stamps[0]) / 1e9
    lengths = np.linalg.norm(xyz, axis=1)
    usable = (lengths > 0.5) & (data["tags"] == 0)
    with np.load(args.model, allow_pickle=False) as model:
        frequencies = np.array(args.frequencies or [float(model["f1"]), float(model["f2"])])
        m1, m2 = int(model["m1"]), int(model["m2"])
        pairs = np.array(
            [(m, q) for m in range(m1 + 1) for q in range(-m2, m2 + 1) if not (m == 0 and q < 0)]
        )
        coefs = np.zeros((len(pairs), 4, 3), dtype=complex)
        column = 0
        for index, (m, q) in enumerate(pairs):
            coefs[index] = model["coefs"][:, column]
            column += 1
            if m or q:
                coefs[index] -= 1j * model["coefs"][:, column]
                column += 1

    def predict(indices: NDArray[np.int64], parameters: NDArray[np.float64]) -> NDArray[np.float64]:
        # Channels carry 5 us offsets; one Fourier evaluation describes the four-laser group.
        t = times[indices] - (indices % 4) * 5e-6
        harmonic = np.exp(
            2j
            * np.pi
            * (t[:, None] * (pairs @ parameters[2:])[None, :] + (pairs @ parameters[:2])[None, :])
        )
        vectors = np.einsum("np,pnd->nd", harmonic, coefs[:, indices % 4]).real
        return vectors / np.linalg.norm(vectors, axis=1, keepdims=True)

    rng = np.random.default_rng(0)
    eligible = np.flatnonzero(usable & (times < args.train_seconds))
    indices = np.sort(rng.choice(eligible, 1200, replace=False))
    actual = xyz[indices] / lengths[indices, None]

    def residual(phases: NDArray[np.float64]) -> NDArray[np.float64]:
        return (predict(indices, np.r_[phases, frequencies]) - actual).ravel()

    seeds = [
        (np.linalg.norm(residual(np.array([a, b]))), a, b)
        for a in np.arange(0, 1, 1 / 8)
        for b in np.arange(0, 1, 1 / 8)
    ]
    _, a, b = min(seeds)
    fit = least_squares(residual, [a, b], max_nfev=50, ftol=1e-12, xtol=1e-12, gtol=1e-12)
    parameters = np.r_[fit.x, frequencies]
    if args.calibrate:
        fit = least_squares(
            lambda p: (predict(indices, p) - actual).ravel(),
            parameters,
            bounds=(
                [-np.inf, -np.inf, *(frequencies - 0.1)],
                [np.inf, np.inf, *(frequencies + 0.1)],
            ),
            max_nfev=100,
            ftol=1e-12,
            xtol=1e-12,
            gtol=1e-12,
        )
        parameters = fit.x
    summary["fit"] = {
        "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "training_seconds": args.train_seconds,
        "training_samples": len(indices),
        "rng_seed": 0,
        "selection": "tag=0, range>0.5m",
        "phase_cycles": parameters[:2].tolist(),
        "frequency_hz": parameters[2:].tolist(),
        "fit_frequencies": args.calibrate,
        "shape_coefficients_fitted": False,
    }
    windows, plot_times, plot_errors = [], [], []
    for low, high in ((0, 1), (1, 4), (4, 8), (8, 12)):
        eligible = np.flatnonzero(usable & (times >= low) & (times < high))
        ii = np.sort(rng.choice(eligible, min(12000, len(eligible)), replace=False))
        if not len(ii):
            raise ValueError("Study expects at least 12 seconds of sensor data")
        cosine = np.einsum("ij,ij->i", predict(ii, parameters), xyz[ii] / lengths[ii, None])
        errors = np.rad2deg(np.arccos(np.clip(cosine, -1, 1)))
        windows.append(
            {
                "seconds": [low, high],
                "samples": len(ii),
                "median_p95_max_degrees": np.quantile(errors, [0.5, 0.95, 1]).tolist(),
            }
        )
        plot_times.extend(times[ii])
        plot_errors.extend(errors)
    summary["windows"] = windows
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")

    window = np.flatnonzero(usable & (times >= 1) & (times < 1.1))
    figure = Figure(figsize=(14, 10), facecolor="white", layout="constrained")
    FigureCanvasAgg(figure)
    axes = figure.subplots(2, 2)
    for axis, title, points in (
        (axes[0, 0], "Real Mid360 returns", xyz[window]),
        (axes[0, 1], "Continuous model at the same firing indices", predict(window, parameters)),
    ):
        angles = angular_points(points)
        scatter = axis.scatter(
            *angles.T,
            c=(times[window] - 1) * 1000,
            s=1.5,
            linewidths=0,
            vmin=0,
            vmax=100,
            cmap="viridis",
        )
        axis.set(
            title=title,
            xlabel="Azimuth (degrees)",
            ylabel="Elevation (degrees)",
            xlim=(-180, 180),
            ylim=(-12, 62),
        )
    figure.colorbar(scatter, ax=list(axes[0]), label="Time within 100 ms (ms)")
    axes[1, 0].scatter(plot_times, plot_errors, s=1, alpha=0.1, linewidths=0, color="#27766d")
    axes[1, 0].axvspan(0, args.train_seconds, color="#dfb84d", alpha=0.2, label="Fit interval")
    for quantile, column, color in (("Median", 0, "#25343d"), ("95th percentile", 1, "#b44d34")):
        axes[1, 0].plot(
            [(w["seconds"][0] + w["seconds"][1]) / 2 for w in windows],
            [w["median_p95_max_degrees"][column] for w in windows],
            "o-",
            color=color,
            label=quantile,
        )
    axes[1, 0].set(
        title="Pointwise angular residual", xlabel="Time in capture (s)", ylabel="Error (degrees)"
    )
    axes[1, 0].legend()
    axes[1, 1].hist(np.diff(data["imu_stamps"]) / 1e6, bins=40, color="#27766d")
    axes[1, 1].axvline(5, color="#b44d34", label="Current simulation: exactly 5 ms")
    axes[1, 1].set(
        title="Real IMU acquisition intervals",
        xlabel="Sensor timestamp interval (ms)",
        ylabel="Count",
    )
    axes[1, 1].legend()
    for axis in axes.ravel():
        axis.grid(alpha=0.15)
        axis.set_axisbelow(True)
    figure.suptitle(f"Mid360 hardware comparison / {args.capture.stem}", fontsize=16)
    figure.supxlabel(
        "Both angle plots use the hardware return mask; holes are NOT predicted dropout.\n"
        "Shape coefficients are frozen; two starting phases are aligned once per capture. "
        + ("Two rotor rates fitted here." if args.calibrate else "Rotor rates frozen."),
        fontsize=10,
    )
    figure.savefig(args.output.with_suffix(".png"), dpi=160)
    print(
        json.dumps(
            {
                "summary": str(args.output.with_suffix(".json")),
                "fit": summary["fit"],
                "windows": windows,
            },
            indent=2,
        )
    )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("capture", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--model", type=Path, default=get_data("mid360_pattern/fourier.npz"))
    parser.add_argument("--train-seconds", type=float, default=1.0)
    parser.add_argument(
        "--calibrate", action="store_true", help="Fit two rotor rates as well as phases"
    )
    parser.add_argument("--frequencies", nargs=2, type=float)
    args = parser.parse_args()
    with threadpool_limits(limits=1):
        analyze(args)


if __name__ == "__main__":
    main()
