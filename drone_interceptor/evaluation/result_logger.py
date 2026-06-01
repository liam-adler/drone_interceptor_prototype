from __future__ import annotations

import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt


def write_distance_csv(
    csv_path: Path,
    samples: list[tuple[float, float]],
) -> None:
    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(["time_s", "distance_m"])
        writer.writerows(samples)


def write_summary_json(json_path: Path, summary: dict[str, object]) -> None:
    with json_path.open("w", encoding="utf-8") as json_file:
        json.dump(summary, json_file, indent=2)


def write_distance_plot(
    png_path: Path,
    *,
    run_label: str,
    capture_radius: float,
    samples: list[tuple[float, float]],
    capture_time: float | None,
) -> None:
    times = [sample[0] for sample in samples]
    distances = [sample[1] for sample in samples]

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(times, distances, color="#1f77b4", linewidth=2.0, label="distance")
    ax.axhline(
        capture_radius,
        color="#d62728",
        linestyle="--",
        linewidth=1.5,
        label="capture radius",
    )
    ax.fill_between(
        times,
        0.0,
        capture_radius,
        color="#d62728",
        alpha=0.08,
    )
    if capture_time is not None:
        capture_distance = capture_radius
        for time_value, distance_value in samples:
            if time_value >= capture_time:
                capture_distance = distance_value
                break

        ax.scatter(
            [capture_time],
            [capture_distance],
            color="#2ca02c",
            s=70,
            zorder=5,
            label="capture",
        )
        ax.axvline(
            capture_time,
            color="#2ca02c",
            linestyle=":",
            linewidth=1.4,
            alpha=0.9,
        )
    ax.set_title(f"Intercept Distance - {run_label}")
    ax.set_xlabel("Time [s]")
    ax.set_ylabel("Distance [m]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    fig.tight_layout()
    fig.savefig(png_path, dpi=160)
    plt.close(fig)
