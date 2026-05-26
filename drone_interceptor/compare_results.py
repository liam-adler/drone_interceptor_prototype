#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import os
from collections import defaultdict
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


@dataclass
class RunSummary:
    name: str
    controller_label: str
    profile: str
    threat: str
    spawn_distance: str
    random_seed: int
    captured: bool
    capture_time_s: float | None
    min_distance_m: float
    final_distance_m: float
    elapsed_s: float
    sample_count: int
    csv_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate aggregate comparison plots from drone interceptor run summaries."
    )
    parser.add_argument(
        "--results-dir",
        default="results/drone_interceptor",
        help="Directory containing per-run JSON summaries.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for aggregate plots. Defaults to <results-dir>/aggregate.",
    )
    return parser.parse_args()


def parse_run_label(run_label: str) -> dict[str, str]:
    prefixes = {
        "mode_": "mode",
        "profile_": "profile",
        "spawn_": "spawn",
        "seed_": "seed",
    }
    parsed: dict[str, str] = {}

    for part in run_label.split("__"):
        if part in {"threat_on", "threat_off"}:
            parsed["threat"] = part
            continue

        for prefix, key in prefixes.items():
            if part.startswith(prefix):
                parsed[key] = part[len(prefix):]
                break

    return parsed


def load_run_summaries(results_dir: Path) -> list[RunSummary]:
    summaries: list[RunSummary] = []

    for json_path in sorted(results_dir.glob("*.json")):
        with json_path.open("r", encoding="utf-8") as json_file:
            data = json.load(json_file)

        run_label = str(data.get("run_label", json_path.stem))
        parsed = parse_run_label(run_label)

        csv_path = json_path.with_suffix(".csv")
        summaries.append(
            RunSummary(
                name=run_label,
                controller_label=str(data.get("controller_label", "unknown")),
                profile=str(data.get("profile_name", parsed.get("profile", "unknown"))),
                threat=(
                    "threat_on"
                    if bool(data.get("threat_response", parsed.get("threat") == "threat_on"))
                    else "threat_off"
                ),
                spawn_distance=format_spawn_distance(
                    data.get("spawn_distance_m", parsed.get("spawn", "unknown"))
                ),
                random_seed=int(data.get("random_seed", parsed.get("seed", 0))),
                captured=bool(data.get("captured", False)),
                capture_time_s=data.get("capture_time_s"),
                min_distance_m=float(data.get("min_distance_m", float("nan"))),
                final_distance_m=float(data.get("final_distance_m", float("nan"))),
                elapsed_s=float(data.get("elapsed_s", float("nan"))),
                sample_count=int(data.get("sample_count", 0)),
                csv_path=csv_path,
            )
        )

    return summaries


def group_key(summary: RunSummary) -> str:
    return (
        f"profile={summary.profile}, "
        f"{summary.threat}, "
        f"spawn={summary.spawn_distance}"
    )


def scenario_key(summary: RunSummary) -> str:
    return (
        f"profile={summary.profile}, "
        f"{summary.threat}, "
        f"spawn={summary.spawn_distance}, "
        f"seed={summary.random_seed}"
    )


def format_spawn_distance(value: object) -> str:
    if isinstance(value, (int, float)):
        return f"{float(value):.1f}m"
    text = str(value)
    if text.endswith("m"):
        return text
    return text


def load_distance_samples(csv_path: Path) -> list[tuple[float, float]]:
    if not csv_path.exists():
        return []

    with csv_path.open("r", encoding="utf-8") as csv_file:
        reader = csv.DictReader(csv_file)
        return [
            (float(row["time_s"]), float(row["distance_m"]))
            for row in reader
        ]


def distance_at_time(
    samples: list[tuple[float, float]],
    time_value: float,
) -> float:
    if not samples:
        return float("nan")

    if time_value <= samples[0][0]:
        return samples[0][1]

    for index in range(len(samples) - 1):
        t0, d0 = samples[index]
        t1, d1 = samples[index + 1]
        if t0 <= time_value <= t1:
            alpha = (time_value - t0) / max(t1 - t0, 1e-9)
            return d0 + alpha * (d1 - d0)

    return samples[-1][1]


def interpolate_distance_curve(
    samples: list[tuple[float, float]],
    time_grid: list[float],
) -> list[float]:
    if not samples:
        return [float("nan")] * len(time_grid)

    sample_times = [sample[0] for sample in samples]
    sample_distances = [sample[1] for sample in samples]
    interpolated: list[float] = []

    index = 0
    for time_value in time_grid:
        while index + 1 < len(sample_times) and sample_times[index + 1] < time_value:
            index += 1

        if time_value <= sample_times[0]:
            interpolated.append(sample_distances[0])
        elif time_value >= sample_times[-1]:
            interpolated.append(sample_distances[-1])
        else:
            t0 = sample_times[index]
            t1 = sample_times[index + 1]
            d0 = sample_distances[index]
            d1 = sample_distances[index + 1]
            alpha = (time_value - t0) / max(t1 - t0, 1e-9)
            interpolated.append(d0 + alpha * (d1 - d0))

    return interpolated


def create_distance_over_time_plots(
    summaries: list[RunSummary],
    output_dir: Path,
) -> list[Path]:
    grouped: dict[str, dict[str, list[RunSummary]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )

    for summary in summaries:
        grouped[group_key(summary)][summary.controller_label].append(summary)

    output_paths: list[Path] = []

    for key in sorted(grouped):
        baseline_runs = grouped[key]["baseline"]
        mpc_runs = grouped[key]["mpc"]
        runs = baseline_runs + mpc_runs
        if not runs:
            continue

        max_elapsed = max(run.elapsed_s for run in runs if run.elapsed_s == run.elapsed_s)
        if max_elapsed <= 0.0:
            continue

        time_grid = [
            max_elapsed * index / 299.0 for index in range(300)
        ]

        fig, ax = plt.subplots(figsize=(10, 5.5))
        capture_radius = 0.5
        ax.axhline(
            capture_radius,
            color="#d62728",
            linestyle="--",
            linewidth=1.4,
            label="capture radius",
        )
        ax.fill_between(
            time_grid,
            0.0,
            capture_radius,
            color="#d62728",
            alpha=0.06,
        )

        for controller, color, alpha in (
            ("baseline", "#1f77b4", 0.18),
            ("mpc", "#ff7f0e", 0.18),
        ):
            controller_runs = grouped[key][controller]
            curves: list[list[float]] = []
            capture_points: list[tuple[float, float]] = []
            for run in controller_runs:
                samples = load_distance_samples(run.csv_path)
                if not samples:
                    continue
                curve = interpolate_distance_curve(samples, time_grid)
                curves.append(curve)
                ax.plot(time_grid, curve, color=color, alpha=alpha, linewidth=1.0)
                if run.capture_time_s is not None:
                    capture_points.append(
                        (
                            run.capture_time_s,
                            distance_at_time(samples, run.capture_time_s),
                        )
                    )

            if curves:
                mean_curve = []
                for sample_index in range(len(time_grid)):
                    values = [
                        curve[sample_index]
                        for curve in curves
                        if curve[sample_index] == curve[sample_index]
                    ]
                    mean_curve.append(mean_or_nan(values))

                ax.plot(
                    time_grid,
                    mean_curve,
                    color=color,
                    linewidth=2.8,
                    label=f"{controller} mean",
                )

            if capture_points:
                ax.scatter(
                    [point[0] for point in capture_points],
                    [point[1] for point in capture_points],
                    color=color,
                    edgecolors="white",
                    linewidths=0.7,
                    s=34,
                    zorder=5,
                    label=f"{controller} captures",
                )

        ax.set_title(f"Distance Over Time - {key}")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Distance [m]")
        ax.grid(True, alpha=0.3)
        ax.legend()
        fig.tight_layout()

        safe_name = (
            key.replace(", ", "__")
            .replace("=", "_")
            .replace("/", "_")
        )
        output_path = output_dir / f"distance_over_time__{safe_name}.png"
        fig.savefig(output_path, dpi=160)
        plt.close(fig)
        output_paths.append(output_path)

    return output_paths


def create_summary_figure(summaries: list[RunSummary], output_dir: Path) -> Path:
    grouped: dict[str, dict[str, list[RunSummary]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )
    for summary in summaries:
        grouped[group_key(summary)][summary.controller_label].append(summary)

    if not grouped:
        raise ValueError("No summaries available to build summary figure.")

    first_key = sorted(grouped)[0]
    runs = grouped[first_key]["baseline"] + grouped[first_key]["mpc"]
    max_elapsed = max(run.elapsed_s for run in runs if run.elapsed_s == run.elapsed_s)
    time_grid = [max_elapsed * index / 299.0 for index in range(300)]

    fig = plt.figure(figsize=(12, 9))
    gs = fig.add_gridspec(2, 1, height_ratios=[2.2, 1.2], hspace=0.28)
    ax_top = fig.add_subplot(gs[0, 0])
    ax_bottom = fig.add_subplot(gs[1, 0])

    capture_radius = 0.5
    ax_top.axhline(
        capture_radius,
        color="#d62728",
        linestyle="--",
        linewidth=1.4,
        label="capture radius",
    )
    ax_top.fill_between(
        time_grid,
        0.0,
        capture_radius,
        color="#d62728",
        alpha=0.06,
    )

    for controller, color in (("baseline", "#1f77b4"), ("mpc", "#ff7f0e")):
        curves: list[list[float]] = []
        capture_points: list[tuple[float, float]] = []
        for run in grouped[first_key][controller]:
            samples = load_distance_samples(run.csv_path)
            if not samples:
                continue
            curve = interpolate_distance_curve(samples, time_grid)
            curves.append(curve)
            ax_top.plot(time_grid, curve, color=color, alpha=0.16, linewidth=1.0)
            if run.capture_time_s is not None:
                capture_points.append(
                    (
                        run.capture_time_s,
                        distance_at_time(samples, run.capture_time_s),
                    )
                )

        if curves:
            mean_curve = []
            for sample_index in range(len(time_grid)):
                values = [
                    curve[sample_index]
                    for curve in curves
                    if curve[sample_index] == curve[sample_index]
                ]
                mean_curve.append(mean_or_nan(values))

            ax_top.plot(
                time_grid,
                mean_curve,
                color=color,
                linewidth=3.0,
                label=f"{controller} mean",
            )

        if capture_points:
            ax_top.scatter(
                [point[0] for point in capture_points],
                [point[1] for point in capture_points],
                color=color,
                edgecolors="white",
                linewidths=0.7,
                s=42,
                zorder=5,
                label=f"{controller} captures",
            )

    ax_top.set_title(f"Summary Comparison - {first_key}")
    ax_top.set_xlabel("Time [s]")
    ax_top.set_ylabel("Distance [m]")
    ax_top.grid(True, alpha=0.3)
    ax_top.legend(ncol=2)

    metrics = ["Capture Rate [%]", "Capture Time [s]", "Min Distance [m]"]
    baseline_runs = grouped[first_key]["baseline"]
    mpc_runs = grouped[first_key]["mpc"]
    baseline_values = [
        100.0 * mean_or_nan([1 if run.captured else 0 for run in baseline_runs]),
        mean_or_nan(
            [run.capture_time_s for run in baseline_runs if run.capture_time_s is not None]
        ),
        mean_or_nan([run.min_distance_m for run in baseline_runs]),
    ]
    mpc_values = [
        100.0 * mean_or_nan([1 if run.captured else 0 for run in mpc_runs]),
        mean_or_nan(
            [run.capture_time_s for run in mpc_runs if run.capture_time_s is not None]
        ),
        mean_or_nan([run.min_distance_m for run in mpc_runs]),
    ]

    x_positions = list(range(len(metrics)))
    width = 0.36
    ax_bottom.bar(
        [x - width / 2 for x in x_positions],
        baseline_values,
        width=width,
        color="#1f77b4",
        label="baseline",
    )
    ax_bottom.bar(
        [x + width / 2 for x in x_positions],
        mpc_values,
        width=width,
        color="#ff7f0e",
        label="mpc",
    )
    ax_bottom.set_xticks(x_positions)
    ax_bottom.set_xticklabels(metrics)
    ax_bottom.set_title("Aggregate Metrics")
    ax_bottom.grid(True, axis="y", alpha=0.3)
    ax_bottom.legend()

    fig.tight_layout()
    output_path = output_dir / "summary_comparison.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def create_capture_time_plot(summaries: list[RunSummary], output_dir: Path) -> Path:
    grouped: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )

    for summary in summaries:
        if summary.capture_time_s is not None:
            grouped[group_key(summary)][summary.controller_label].append(
                summary.capture_time_s
            )

    keys = sorted(grouped)
    baseline_values = [
        mean_or_nan(grouped[key]["baseline"]) for key in keys
    ]
    mpc_values = [
        mean_or_nan(grouped[key]["mpc"]) for key in keys
    ]

    fig, ax = plt.subplots(figsize=(max(10, len(keys) * 1.5), 6))
    x_positions = list(range(len(keys)))
    width = 0.38

    ax.bar(
        [x - width / 2 for x in x_positions],
        baseline_values,
        width=width,
        label="baseline",
        color="#1f77b4",
    )
    ax.bar(
        [x + width / 2 for x in x_positions],
        mpc_values,
        width=width,
        label="mpc",
        color="#ff7f0e",
    )
    ax.set_title("Average Capture Time by Configuration")
    ax.set_ylabel("Capture Time [s]")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(keys, rotation=25, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()

    output_path = output_dir / "capture_time_comparison.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def create_min_distance_plot(summaries: list[RunSummary], output_dir: Path) -> Path:
    grouped: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )

    for summary in summaries:
        grouped[group_key(summary)][summary.controller_label].append(
            summary.min_distance_m
        )

    keys = sorted(grouped)
    baseline_values = [
        mean_or_nan(grouped[key]["baseline"]) for key in keys
    ]
    mpc_values = [
        mean_or_nan(grouped[key]["mpc"]) for key in keys
    ]

    fig, ax = plt.subplots(figsize=(max(10, len(keys) * 1.5), 6))
    x_positions = list(range(len(keys)))
    width = 0.38

    ax.bar(
        [x - width / 2 for x in x_positions],
        baseline_values,
        width=width,
        label="baseline",
        color="#2ca02c",
    )
    ax.bar(
        [x + width / 2 for x in x_positions],
        mpc_values,
        width=width,
        label="mpc",
        color="#d62728",
    )
    ax.set_title("Average Minimum Distance by Configuration")
    ax.set_ylabel("Minimum Distance [m]")
    ax.set_xticks(x_positions)
    ax.set_xticklabels(keys, rotation=25, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()

    output_path = output_dir / "min_distance_comparison.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def create_capture_rate_plot(summaries: list[RunSummary], output_dir: Path) -> Path:
    grouped: dict[str, dict[str, list[int]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )

    for summary in summaries:
        grouped[group_key(summary)][summary.controller_label].append(
            1 if summary.captured else 0
        )

    keys = sorted(grouped)
    baseline_values = [
        100.0 * mean_or_nan(grouped[key]["baseline"]) for key in keys
    ]
    mpc_values = [
        100.0 * mean_or_nan(grouped[key]["mpc"]) for key in keys
    ]

    fig, ax = plt.subplots(figsize=(max(10, len(keys) * 1.5), 6))
    x_positions = list(range(len(keys)))
    width = 0.38

    ax.bar(
        [x - width / 2 for x in x_positions],
        baseline_values,
        width=width,
        label="baseline",
        color="#9467bd",
    )
    ax.bar(
        [x + width / 2 for x in x_positions],
        mpc_values,
        width=width,
        label="mpc",
        color="#8c564b",
    )
    ax.set_title("Capture Rate by Configuration")
    ax.set_ylabel("Capture Rate [%]")
    ax.set_ylim(0.0, 105.0)
    ax.set_xticks(x_positions)
    ax.set_xticklabels(keys, rotation=25, ha="right")
    ax.grid(True, axis="y", alpha=0.3)
    ax.legend()
    fig.tight_layout()

    output_path = output_dir / "capture_rate_comparison.png"
    fig.savefig(output_path, dpi=160)
    plt.close(fig)
    return output_path


def write_summary_table(summaries: list[RunSummary], output_dir: Path) -> Path:
    grouped: dict[str, dict[str, list[RunSummary]]] = defaultdict(
        lambda: {"baseline": [], "mpc": []}
    )

    for summary in summaries:
        grouped[group_key(summary)][summary.controller_label].append(summary)

    lines = [
        "# Aggregate Comparison",
        "",
        "| Configuration | Controller | Runs | Capture Rate | Avg Capture Time [s] | Avg Min Distance [m] |",
        "|---|---:|---:|---:|---:|---:|",
    ]

    for key in sorted(grouped):
        for controller in ("baseline", "mpc"):
            runs = grouped[key][controller]
            capture_rate = 100.0 * mean_or_nan([1 if run.captured else 0 for run in runs])
            capture_times = [
                run.capture_time_s for run in runs if run.capture_time_s is not None
            ]
            avg_capture_time = mean_or_nan(capture_times)
            avg_min_distance = mean_or_nan([run.min_distance_m for run in runs])

            lines.append(
                "| "
                f"{key} | {controller} | {len(runs)} | "
                f"{format_value(capture_rate)}% | "
                f"{format_value(avg_capture_time)} | "
                f"{format_value(avg_min_distance)} |"
            )

    output_path = output_dir / "aggregate_summary.md"
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def mean_or_nan(values: list[float]) -> float:
    if not values:
        return float("nan")
    return sum(values) / len(values)


def format_value(value: float) -> str:
    if value != value:
        return "n/a"
    return f"{value:.2f}"


def main() -> None:
    args = parse_args()
    results_dir = Path(args.results_dir).expanduser()
    output_dir = (
        Path(args.output_dir).expanduser()
        if args.output_dir is not None
        else results_dir / "aggregate"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    summaries = load_run_summaries(results_dir)
    if not summaries:
        raise SystemExit(
            f"No JSON run summaries found in '{results_dir}'. Run the simulator first."
        )

    outputs = [
        *create_distance_over_time_plots(summaries, output_dir),
        create_summary_figure(summaries, output_dir),
        create_capture_time_plot(summaries, output_dir),
        create_min_distance_plot(summaries, output_dir),
        create_capture_rate_plot(summaries, output_dir),
        write_summary_table(summaries, output_dir),
    ]

    print("Wrote aggregate artifacts:")
    for output in outputs:
        print(output)


if __name__ == "__main__":
    main()
