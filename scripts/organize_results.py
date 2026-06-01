#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path

ARTIFACT_SUFFIXES = (".json", ".csv", ".png")
MoveRecord = tuple[Path, Path]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Organize drone interceptor result artifacts into cleaner folders."
    )
    parser.add_argument(
        "--results-dir",
        default="results/drone_interceptor",
        help="Top-level results directory.",
    )
    return parser.parse_args()


def build_destination_dir(results_dir: Path, json_data: dict[str, object]) -> Path:
    controller = str(json_data.get("controller_label", "unknown"))
    profile = str(json_data.get("profile_name", "unknown"))
    threat_label = (
        "threat_on" if bool(json_data.get("threat_response", False)) else "threat_off"
    )
    spawn = float(json_data.get("spawn_distance_m", 0.0))
    spawn_text = f"{spawn:.1f}".replace(".", "p") + "m"

    random_seed = json_data.get("random_seed")
    if random_seed is None:
        return results_dir / "archive" / "legacy_unseeded" / controller

    scenario_dir = f"profile_{profile}__{threat_label}__spawn_{spawn_text}"
    return (
        results_dir
        / "runs"
        / "paired_seeded"
        / scenario_dir
        / f"seed_{int(random_seed)}"
        / controller
    )


def iter_top_level_json_paths(results_dir: Path) -> list[Path]:
    return sorted(path for path in results_dir.glob("*.json") if path.is_file())


def load_result_metadata(json_path: Path) -> dict[str, object]:
    return json.loads(json_path.read_text(encoding="utf-8"))


def move_run_artifacts(
    results_dir: Path,
    json_path: Path,
    target_dir: Path,
) -> list[MoveRecord]:
    target_dir.mkdir(parents=True, exist_ok=True)

    moves: list[MoveRecord] = []
    stem = json_path.stem
    for suffix in ARTIFACT_SUFFIXES:
        source = results_dir / f"{stem}{suffix}"
        if not source.exists():
            continue

        destination = target_dir / source.name
        shutil.move(str(source), str(destination))
        moves.append((source, destination))

    return moves


def organize(results_dir: Path) -> list[MoveRecord]:
    moves: list[MoveRecord] = []

    for json_path in iter_top_level_json_paths(results_dir):
        data = load_result_metadata(json_path)
        target_dir = build_destination_dir(results_dir, data)
        moves.extend(move_run_artifacts(results_dir, json_path, target_dir))

    return moves


def main() -> None:
    args = parse_args()
    results_dir = Path(args.results_dir).expanduser()
    if not results_dir.exists():
        raise SystemExit(f"Results directory does not exist: {results_dir}")

    moves = organize(results_dir)
    if not moves:
        print("No top-level run artifacts found to organize.")
        return

    print("Moved artifacts:")
    for source, destination in moves:
        print(f"{source} -> {destination}")


if __name__ == "__main__":
    main()
