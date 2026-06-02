#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from dataclasses import dataclass
import os
from pathlib import Path
import tempfile

os.environ.setdefault(
    "MPLCONFIGDIR",
    str(Path(tempfile.gettempdir()) / "drone_interceptor_mpl"),
)

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


@dataclass
class RunRecord:
    run_label: str
    controller_key: str
    captured: bool
    capture_time_s: float | None
    capture_event_count: int
    min_distance_m: float
    mean_distance_m: float
    final_distance_m: float
    elapsed_s: float
    csv_path: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate averaged overlay plots across multiple controller runs."
    )
    parser.add_argument(
        "--results-dir",
        default="results/drone_interceptor_average_runs",
        help="Directory containing run CSV/JSON artifacts.",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Directory for averaged plots. Defaults to <results-dir>/average_overlay.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Duration to plot in seconds.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=300,
        help="Number of interpolation samples along the time axis.",
    )
    return parser.parse_args()


def parse_run_label(run_label: str) -> dict[str, str]:
    parsed: dict[str, str] = {}
    prefixes = (
        ("mode_", "mode"),
        ("profile_", "profile"),
        ("guidance_", "guidance"),
        ("scenario_", "scenario"),
        ("preset_", "preset"),
        ("seed_", "seed"),
    )

    for part in run_label.split("__"):
        if part in {"threat_on", "threat_off"}:
            parsed["threat"] = part
            continue

        for prefix, key in prefixes:
            if part.startswith(prefix):
                parsed[key] = part[len(prefix):]
                break

    return parsed


def build_controller_key(summary: dict[str, object], run_label: str) -> str:
    parsed = parse_run_label(run_label)
    controller_mode = str(summary.get("controller_mode", parsed.get("mode", "unknown")))
    guidance_mode = parsed.get("guidance")
    preset = parsed.get("preset")

    if controller_mode == "mpc":
        return "mpc"
    if guidance_mode:
        return guidance_mode
    if preset and preset.startswith("baseline_"):
        return preset.removeprefix("baseline_")
    return controller_mode


def load_records(results_dir: Path) -> list[RunRecord]:
    records: list[RunRecord] = []
    for json_path in sorted(results_dir.glob("*.json")):
        with json_path.open("r", encoding="utf-8") as handle:
            summary = json.load(handle)

        run_label = str(summary.get("run_label", json_path.stem))
        csv_path = json_path.with_suffix(".csv")
        if not csv_path.exists():
            continue
        samples = load_distance_samples(csv_path)
        capture_radius = float(summary.get("capture_radius_m", 0.5))
        capture_event_count = int(
            summary.get(
                "capture_event_count",
                compute_capture_event_count(samples, capture_radius),
            )
        )

        records.append(
            RunRecord(
                run_label=run_label,
                controller_key=build_controller_key(summary, run_label),
                captured=bool(summary.get("captured", False)),
                capture_time_s=summary.get("capture_time_s"),
                capture_event_count=capture_event_count,
                min_distance_m=float(summary.get("min_distance_m", math.nan)),
                mean_distance_m=compute_mean_distance(samples),
                final_distance_m=float(summary.get("final_distance_m", math.nan)),
                elapsed_s=float(summary.get("elapsed_s", math.nan)),
                csv_path=csv_path,
            )
        )

    return records


def load_distance_samples(csv_path: Path) -> list[tuple[float, float]]:
    with csv_path.open("r", encoding="utf-8") as handle:
        reader = csv.DictReader(handle)
        return [(float(row["time_s"]), float(row["distance_m"])) for row in reader]


def compute_mean_distance(samples: list[tuple[float, float]]) -> float:
    if not samples:
        return math.nan
    return sum(distance for _, distance in samples) / len(samples)


def compute_capture_event_count(
    samples: list[tuple[float, float]],
    capture_radius: float,
) -> int:
    count = 0
    was_within_capture_radius = False

    for _, distance in samples:
        is_within_capture_radius = distance <= capture_radius
        if is_within_capture_radius and not was_within_capture_radius:
            count += 1
        was_within_capture_radius = is_within_capture_radius

    return count


def interpolate_curve(
    samples: list[tuple[float, float]],
    time_grid: list[float],
) -> list[float]:
    if not samples:
        return [math.nan] * len(time_grid)

    sample_times = [item[0] for item in samples]
    sample_distances = [item[1] for item in samples]
    values: list[float] = []
    index = 0

    for time_value in time_grid:
        while index + 1 < len(sample_times) and sample_times[index + 1] < time_value:
            index += 1

        if time_value <= sample_times[0]:
            values.append(sample_distances[0])
            continue
        if time_value >= sample_times[-1]:
            values.append(sample_distances[-1])
            continue

        t0 = sample_times[index]
        t1 = sample_times[index + 1]
        d0 = sample_distances[index]
        d1 = sample_distances[index + 1]
        alpha = (time_value - t0) / max(t1 - t0, 1e-9)
        values.append(d0 + alpha * (d1 - d0))

    return values


def mean_or_nan(values: list[float]) -> float:
    valid = [value for value in values if not math.isnan(value)]
    if not valid:
        return math.nan
    return sum(valid) / len(valid)


def std_or_zero(values: list[float], mean_value: float) -> float:
    valid = [value for value in values if not math.isnan(value)]
    if len(valid) < 2:
        return 0.0
    variance = sum((value - mean_value) ** 2 for value in valid) / len(valid)
    return math.sqrt(variance)


def plot_average_overlay(
    records: list[RunRecord],
    output_dir: Path,
    *,
    duration_s: float,
    samples: int,
) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    time_grid = [duration_s * index / max(samples - 1, 1) for index in range(samples)]
    grouped: dict[str, list[RunRecord]] = defaultdict(list)
    for record in records:
        grouped[record.controller_key].append(record)

    colors = {
        "pure_pursuit": "#4c78a8",
        "lead_pursuit": "#59a14f",
        "acceleration_aware_lead_pursuit": "#f28e2b",
        "mpc": "#e15759",
    }

    fig, (ax_curve, ax_metrics) = plt.subplots(
        2,
        1,
        figsize=(12, 10),
        gridspec_kw={"height_ratios": [2.4, 1.2]},
    )

    capture_radius = 0.5
    ax_curve.axhline(
        capture_radius,
        color="#aa2222",
        linestyle="--",
        linewidth=1.4,
        label="capture radius",
    )
    ax_curve.fill_between(time_grid, 0.0, capture_radius, color="#aa2222", alpha=0.06)

    summary_table: list[tuple[str, float, float, float, float, float]] = []

    for controller_key in sorted(grouped):
        controller_records = grouped[controller_key]
        controller_curves: list[list[float]] = []
        color = colors.get(controller_key, "#333333")

        for record in controller_records:
            samples_for_run = load_distance_samples(record.csv_path)
            curve = interpolate_curve(samples_for_run, time_grid)
            controller_curves.append(curve)
            ax_curve.plot(time_grid, curve, color=color, alpha=0.14, linewidth=1.0)

        mean_curve: list[float] = []
        lower_band: list[float] = []
        upper_band: list[float] = []

        for sample_index in range(len(time_grid)):
            values = [curve[sample_index] for curve in controller_curves]
            mean_value = mean_or_nan(values)
            std_value = std_or_zero(values, mean_value)
            mean_curve.append(mean_value)
            lower_band.append(mean_value - std_value)
            upper_band.append(mean_value + std_value)

        ax_curve.plot(
            time_grid,
            mean_curve,
            color=color,
            linewidth=2.8,
            label=f"{controller_key} mean",
        )
        ax_curve.fill_between(
            time_grid,
            lower_band,
            upper_band,
            color=color,
            alpha=0.12,
        )

        mean_capture_time = mean_or_nan(
            [
                float(record.capture_time_s)
                for record in controller_records
                if record.capture_time_s is not None
            ]
        )
        mean_min_distance = mean_or_nan(
            [record.min_distance_m for record in controller_records]
        )
        avg_capture_count_per_run = mean_or_nan(
            [float(record.capture_event_count) for record in controller_records]
        )
        mean_distance = mean_or_nan(
            [record.mean_distance_m for record in controller_records]
        )
        summary_table.append(
            (
                controller_key,
                avg_capture_count_per_run,
                mean_capture_time,
                mean_min_distance,
                mean_distance,
            )
        )

    ax_curve.set_title("Average Distance Over Time Across Controllers")
    ax_curve.set_xlabel("Time [s]")
    ax_curve.set_ylabel("Distance [m]")
    ax_curve.grid(True, alpha=0.3)
    ax_curve.legend()

    metric_names = [
        "Avg Captures / Run",
        "Capture Time [s]",
        "Min Distance [m]",
        "Avg Distance [m]",
    ]
    x_positions = list(range(len(metric_names)))
    bar_width = min(0.18, 0.8 / max(len(summary_table), 1))

    for index, (
        controller_key,
        avg_capture_count_per_run,
        capture_time,
        min_distance,
        mean_distance,
    ) in enumerate(
        summary_table
    ):
        offset = (index - (len(summary_table) - 1) / 2.0) * bar_width
        values = [
            avg_capture_count_per_run,
            capture_time,
            min_distance,
            mean_distance,
        ]
        ax_metrics.bar(
            [position + offset for position in x_positions],
            values,
            width=bar_width,
            label=controller_key,
            color=colors.get(controller_key, "#333333"),
        )
        ax_metrics.text(
            x_positions[0] + offset,
            avg_capture_count_per_run + max(0.03, 0.02 * avg_capture_count_per_run),
            f"{avg_capture_count_per_run:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
            color=colors.get(controller_key, "#333333"),
            fontweight="bold",
        )

    ax_metrics.set_xticks(x_positions)
    ax_metrics.set_xticklabels(metric_names)
    ax_metrics.set_title("Average Performance Across Runs")
    ax_metrics.grid(True, axis="y", alpha=0.3)
    ax_metrics.legend()

    fig.tight_layout()
    output_path = output_dir / "average_overlay_comparison.png"
    fig.savefig(output_path, dpi=180)
    plt.close(fig)
    return output_path


def write_summary_markdown(records: list[RunRecord], output_dir: Path) -> Path:
    grouped: dict[str, list[RunRecord]] = defaultdict(list)
    for record in records:
        grouped[record.controller_key].append(record)

    lines = [
        "# Average Overlay Summary",
        "",
        "| Controller | Runs | Avg Captures / Run | Mean Capture Time [s] | Mean Min Distance [m] | Mean Distance [m] | Mean Final Distance [m] |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]

    for controller_key in sorted(grouped):
        controller_records = grouped[controller_key]
        avg_capture_count_per_run = mean_or_nan(
            [float(record.capture_event_count) for record in controller_records]
        )
        mean_capture_time = mean_or_nan(
            [
                float(record.capture_time_s)
                for record in controller_records
                if record.capture_time_s is not None
            ]
        )
        mean_min_distance = mean_or_nan(
            [record.min_distance_m for record in controller_records]
        )
        mean_distance = mean_or_nan(
            [record.mean_distance_m for record in controller_records]
        )
        mean_final_distance = mean_or_nan(
            [record.final_distance_m for record in controller_records]
        )

        lines.append(
            "| "
            + " | ".join(
                [
                    controller_key,
                    str(len(controller_records)),
                    f"{avg_capture_count_per_run:.2f}",
                    "nan" if math.isnan(mean_capture_time) else f"{mean_capture_time:.2f}",
                    f"{mean_min_distance:.3f}",
                    f"{mean_distance:.3f}",
                    f"{mean_final_distance:.3f}",
                ]
            )
            + " |"
        )

    output_path = output_dir / "average_overlay_summary.md"
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return output_path


def main() -> int:
    args = parse_args()
    results_dir = Path(args.results_dir).expanduser().resolve()
    output_dir = (
        Path(args.output_dir).expanduser().resolve()
        if args.output_dir is not None
        else results_dir / "average_overlay"
    )

    if not results_dir.exists():
        raise FileNotFoundError(f"Results directory does not exist: {results_dir}")

    records = load_records(results_dir)
    if not records:
        raise RuntimeError(f"No run artifacts found in {results_dir}")

    plot_path = plot_average_overlay(
        records,
        output_dir,
        duration_s=args.duration_s,
        samples=args.samples,
    )
    summary_path = write_summary_markdown(records, output_dir)

    print(f"Wrote {plot_path}")
    print(f"Wrote {summary_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
