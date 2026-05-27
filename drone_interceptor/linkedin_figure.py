#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import os
from dataclasses import dataclass
from pathlib import Path
import tempfile

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "drone_interceptor_mpl"),
)

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


@dataclass
class RunRecord:
    run_label: str
    controller: str
    profile: str
    threat_response: bool
    spawn_distance_m: float
    random_seed: int
    captured: bool
    capture_time_s: float | None
    min_distance_m: float
    elapsed_s: float
    csv_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a polished LinkedIn-ready comparison figure."
    )
    parser.add_argument(
        "--results-dir",
        default="results/drone_interceptor",
        help="Directory containing run CSV/JSON files.",
    )
    parser.add_argument(
        "--profile",
        default="target_advantaged",
        help="Scenario profile to visualize.",
    )
    parser.add_argument(
        "--threat-response",
        default="true",
        choices=["true", "false"],
        help="Threat-response setting to visualize.",
    )
    parser.add_argument(
        "--spawn-distance",
        default="30.0",
        help="Spawn distance in meters to visualize.",
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Output PNG path. Defaults into <results-dir>/aggregate.",
    )
    return parser.parse_args()


def load_runs(results_dir: Path) -> list[RunRecord]:
    records: list[RunRecord] = []
    for json_path in sorted(results_dir.rglob("*.json")):
        if "aggregate" in json_path.parts:
            continue
        data = json.loads(json_path.read_text(encoding="utf-8"))
        if "profile_name" not in data:
            continue
        records.append(
            RunRecord(
                run_label=str(data["run_label"]),
                controller=str(data["controller_label"]),
                profile=str(data["profile_name"]),
                threat_response=bool(data["threat_response"]),
                spawn_distance_m=float(data["spawn_distance_m"]),
                random_seed=int(data["random_seed"]),
                captured=bool(data["captured"]),
                capture_time_s=data["capture_time_s"],
                min_distance_m=float(data["min_distance_m"]),
                elapsed_s=float(data["elapsed_s"]),
                csv_path=json_path.with_suffix(".csv"),
            )
        )
    return records


def load_samples(csv_path: Path) -> list[tuple[float, float]]:
    with csv_path.open("r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        return [
            (float(row["time_s"]), float(row["distance_m"]))
            for row in reader
        ]


def interpolate(samples: list[tuple[float, float]], time_grid: np.ndarray) -> np.ndarray:
    sample_times = np.array([sample[0] for sample in samples], dtype=float)
    sample_distances = np.array([sample[1] for sample in samples], dtype=float)
    return np.interp(time_grid, sample_times, sample_distances)


def distance_at_time(samples: list[tuple[float, float]], time_value: float) -> float:
    sample_times = np.array([sample[0] for sample in samples], dtype=float)
    sample_distances = np.array([sample[1] for sample in samples], dtype=float)
    return float(np.interp([time_value], sample_times, sample_distances)[0])


def format_spawn(spawn_distance: float) -> str:
    return f"{spawn_distance:.1f}".replace(".", "p")


def set_bar_axis_headroom(ax, values: list[float], minimum_top: float = 1.0) -> None:
    finite_values = [float(value) for value in values if not np.isnan(value)]
    if not finite_values:
        ax.set_ylim(0.0, minimum_top)
        return

    max_value = max(finite_values)
    top = max(max_value * 1.18, max_value + 0.12, minimum_top)
    ax.set_ylim(0.0, top)


def build_figure(
    baseline_runs: list[RunRecord],
    mpc_runs: list[RunRecord],
    output_path: Path,
) -> None:
    max_elapsed = max(run.elapsed_s for run in baseline_runs + mpc_runs)
    time_grid = np.linspace(0.0, max_elapsed, 400)
    capture_radius = 0.5

    plt.rcParams.update({
        "font.size": 11,
        "axes.titlesize": 18,
        "axes.labelsize": 12,
    })

    fig = plt.figure(figsize=(14, 9), facecolor="#f7f6f2")
    gs = fig.add_gridspec(2, 3, height_ratios=[2.15, 1.25], hspace=0.42, wspace=0.18)
    ax_top = fig.add_subplot(gs[0, :])
    ax_rate = fig.add_subplot(gs[1, 0])
    ax_time = fig.add_subplot(gs[1, 1])
    ax_distance = fig.add_subplot(gs[1, 2])

    fig.suptitle(
        "Pursuit Guidance vs Acceleration MPC\n"
        "Distance-to-target over time across matched randomized trials",
        x=0.06,
        y=0.98,
        ha="left",
        va="top",
        fontsize=22,
        fontweight="bold",
    )

    ax_top.set_facecolor("white")
    ax_rate.set_facecolor("white")
    ax_time.set_facecolor("white")
    ax_distance.set_facecolor("white")

    ax_top.axhline(
        capture_radius,
        color="#d44d5c",
        linestyle="--",
        linewidth=1.5,
        label="capture radius",
    )
    ax_top.fill_between(
        time_grid,
        0.0,
        capture_radius,
        color="#d44d5c",
        alpha=0.08,
    )

    controller_specs = [
        ("baseline", baseline_runs, "#275dad"),
        ("mpc", mpc_runs, "#f08a24"),
    ]

    for controller, runs, color in controller_specs:
        curves = []
        capture_x = []
        capture_y = []

        for run in runs:
            samples = load_samples(run.csv_path)
            curve = interpolate(samples, time_grid)
            curves.append(curve)
            ax_top.plot(time_grid, curve, color=color, alpha=0.16, linewidth=1.2)

            if run.capture_time_s is not None:
                capture_x.append(run.capture_time_s)
                capture_y.append(distance_at_time(samples, run.capture_time_s))

        mean_curve = np.mean(np.vstack(curves), axis=0)
        ax_top.plot(
            time_grid,
            mean_curve,
            color=color,
            linewidth=3.5,
            label=f"{controller} mean",
        )

        if capture_x:
            ax_top.scatter(
                capture_x,
                capture_y,
                color=color,
                edgecolors="white",
                linewidths=1.0,
                s=60,
                zorder=5,
                label=f"{controller} captures",
            )

    ax_top.set_title(
        "Main result: baseline closes the gap faster and actually converts those "
        "close approaches into captures",
        pad=10,
    )
    ax_top.set_xlabel("Time [s]")
    ax_top.set_ylabel("Distance to target [m]")
    ax_top.grid(True, alpha=0.22)
    ax_top.legend(ncol=3, frameon=False, loc="upper right")

    baseline_capture_rate = 100.0 * sum(run.captured for run in baseline_runs) / len(baseline_runs)
    mpc_capture_rate = 100.0 * sum(run.captured for run in mpc_runs) / len(mpc_runs)
    baseline_capture_times = [run.capture_time_s for run in baseline_runs if run.capture_time_s is not None]
    mpc_capture_times = [run.capture_time_s for run in mpc_runs if run.capture_time_s is not None]
    baseline_min = [run.min_distance_m for run in baseline_runs]
    mpc_min = [run.min_distance_m for run in mpc_runs]

    width = 0.36
    x_positions = np.arange(1)

    rate_baseline = [baseline_capture_rate]
    rate_mpc = [mpc_capture_rate]
    bars1 = ax_rate.bar(
        x_positions - width / 2,
        rate_baseline,
        width,
        color="#275dad",
        label="baseline",
    )
    bars2 = ax_rate.bar(
        x_positions + width / 2,
        rate_mpc,
        width,
        color="#f08a24",
        label="mpc",
    )
    ax_rate.set_xticks(x_positions)
    ax_rate.set_xticklabels(["Capture rate"])
    ax_rate.set_ylim(0.0, 110.0)
    ax_rate.set_title("Capture rate")
    ax_rate.grid(True, axis="y", alpha=0.22)
    ax_rate.legend(frameon=False)
    for bar in list(bars1) + list(bars2):
        height = bar.get_height()
        ax_rate.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.0f}%",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    time_baseline = [float(np.mean(baseline_capture_times)) if baseline_capture_times else np.nan]
    time_mpc = [float(np.mean(mpc_capture_times)) if mpc_capture_times else np.nan]
    bars3 = ax_time.bar(
        x_positions - width / 2,
        time_baseline,
        width,
        color="#275dad",
        label="baseline",
    )
    bars4 = ax_time.bar(
        x_positions + width / 2,
        [0.0 if np.isnan(time_mpc[0]) else time_mpc[0]],
        width,
        color="#f08a24",
        label="mpc",
    )
    ax_time.set_xticks(x_positions)
    ax_time.set_xticklabels(["Avg capture time"])
    ax_time.set_title("Time to capture")
    set_bar_axis_headroom(
        ax_time,
        [time_baseline[0], 0.0 if np.isnan(time_mpc[0]) else time_mpc[0]],
    )
    ax_time.grid(True, axis="y", alpha=0.22)
    ax_time.legend(frameon=False)
    for bar, value in zip(list(bars3) + list(bars4), time_baseline + time_mpc):
        label = "n/a" if np.isnan(value) else f"{value:.1f} s"
        y_value = 0.0 if np.isnan(value) else value
        ax_time.text(
            bar.get_x() + bar.get_width() / 2,
            y_value,
            label,
            ha="center",
            va="bottom",
            fontsize=10,
        )

    distance_baseline = [float(np.mean(baseline_min))]
    distance_mpc = [float(np.mean(mpc_min))]
    bars3 = ax_distance.bar(x_positions - width / 2, distance_baseline, width, color="#275dad", label="baseline")
    bars4 = ax_distance.bar(x_positions + width / 2, distance_mpc, width, color="#f08a24", label="mpc")
    ax_distance.set_xticks(x_positions)
    ax_distance.set_xticklabels(["Avg min distance"])
    ax_distance.set_title("Closest approach")
    set_bar_axis_headroom(ax_distance, [distance_baseline[0], distance_mpc[0]])
    ax_distance.grid(True, axis="y", alpha=0.22)
    ax_distance.legend(frameon=False)
    for bar in list(bars3) + list(bars4):
        height = bar.get_height()
        ax_distance.text(
            bar.get_x() + bar.get_width() / 2,
            height,
            f"{height:.2f} m",
            ha="center",
            va="bottom",
            fontsize=10,
        )

    seed_text = ", ".join(str(run.random_seed) for run in baseline_runs)
    fig.text(
        0.06,
        0.035,
        "Test design: 5 paired trials, same target random seed per pair, "
        "profile=target_advantaged, threat_response=on, spawn_distance=30.0 m, "
        f"seeds={seed_text}.",
        fontsize=10.5,
        color="#444444",
    )

    fig.text(
        0.06,
        0.012,
        "Capture markers show where distance first drops inside the 0.5 m capture radius.",
        fontsize=10,
        color="#666666",
    )

    fig.subplots_adjust(top=0.87, bottom=0.12)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    args = parse_args()
    results_dir = Path(args.results_dir).expanduser()
    threat_response = args.threat_response == "true"
    spawn_distance = float(args.spawn_distance)

    runs = load_runs(results_dir)
    filtered = [
        run for run in runs
        if run.profile == args.profile
        and run.threat_response == threat_response
        and abs(run.spawn_distance_m - spawn_distance) < 1e-6
    ]

    baseline_by_seed = {
        run.random_seed: run for run in filtered if run.controller == "baseline"
    }
    mpc_by_seed = {
        run.random_seed: run for run in filtered if run.controller == "mpc"
    }
    paired_seeds = sorted(set(baseline_by_seed) & set(mpc_by_seed))
    if not paired_seeds:
        raise SystemExit("No paired baseline/mpc runs found for the selected scenario.")

    baseline_runs = [baseline_by_seed[seed] for seed in paired_seeds]
    mpc_runs = [mpc_by_seed[seed] for seed in paired_seeds]

    if args.output is None:
        safe_profile = args.profile
        safe_threat = "threat_on" if threat_response else "threat_off"
        safe_spawn = format_spawn(spawn_distance)
        output_path = (
            results_dir
            / "aggregate"
            / f"linkedin__{safe_profile}__{safe_threat}__spawn_{safe_spawn}m.png"
        )
    else:
        output_path = Path(args.output).expanduser()

    build_figure(baseline_runs, mpc_runs, output_path)
    print(output_path)


if __name__ == "__main__":
    main()
