#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import time


CONTROLLERS: dict[str, list[str]] = {
    "baseline_pure_pursuit": [
        "controller_preset:=baseline_pure_pursuit",
    ],
    "baseline_lead_pursuit": [
        "controller_preset:=baseline_lead_pursuit",
    ],
    "baseline_acceleration_aware": [
        "controller_preset:=baseline_acceleration_aware",
    ],
    "mpc": [
        "controller_preset:=mpc_default",
    ],
}

DEFAULT_WORKSPACE = Path(__file__).resolve().parents[3]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run repeated drone interceptor experiments across guidance/controller "
            "methods."
        )
    )
    parser.add_argument(
        "--workspace",
        default=str(DEFAULT_WORKSPACE),
        help="ROS 2 workspace root. Defaults to the repository workspace.",
    )
    parser.add_argument(
        "--results-dir",
        default="results/drone_interceptor_average_runs",
        help="Directory where run artifacts will be written.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Wall-clock duration of each run in seconds.",
    )
    parser.add_argument(
        "--runs-per-controller",
        type=int,
        default=5,
        help="Number of runs per controller/guidance method.",
    )
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=sorted(CONTROLLERS),
        default=list(CONTROLLERS),
        help="Which controllers/guidance methods to run.",
    )
    parser.add_argument(
        "--profile",
        default="target_advantaged",
        help="Launch profile override.",
    )
    parser.add_argument(
        "--scenario",
        default="target_advantaged_default",
        help="Launch scenario override.",
    )
    parser.add_argument(
        "--spawn-distance",
        type=float,
        default=30.0,
        help="Initial separation in meters.",
    )
    parser.add_argument(
        "--threat-response",
        choices=("true", "false"),
        default="true",
        help="Whether threat response is enabled.",
    )
    parser.add_argument(
        "--seed-start",
        type=int,
        default=0,
        help="Random seed for the first run. Seeds increment by one.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable RViz and rqt_plot during runs.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print launch commands without executing them.",
    )
    return parser.parse_args()


def ensure_workspace(workspace: Path) -> None:
    if not workspace.exists():
        raise FileNotFoundError(f"Workspace does not exist: {workspace}")
    install_setup = workspace / "install" / "setup.bash"
    if not install_setup.exists():
        raise FileNotFoundError(
            f"Workspace overlay not found: {install_setup}. Build the package first."
        )


def build_launch_command(
    *,
    workspace: Path,
    output_dir: Path,
    duration_s: float,
    seed: int,
    controller: str,
    profile: str,
    scenario: str,
    spawn_distance: float,
    threat_response: str,
    headless: bool,
) -> str:
    ros_setup = "source /opt/ros/humble/setup.bash"
    ws_setup = f"source {shlex.quote(str(workspace / 'install' / 'setup.bash'))}"

    launch_args = [
        "ros2 launch drone_interceptor target_sim.launch.py",
        f"profile:={profile}",
        f"scenario:={scenario}",
        f"spawn_distance:={spawn_distance}",
        f"threat_response:={threat_response}",
        f"random_seed:={seed}",
        f"output_dir:={shlex.quote(str(output_dir))}",
    ]
    launch_args.extend(CONTROLLERS[controller])

    if headless:
        launch_args.append("open_rviz:=false")
        launch_args.append("open_plot:=false")

    launch_command = " ".join(launch_args)
    timeout_command = (
        "python3 -c "
        + shlex.quote(
            "import os, signal, subprocess, time; "
            f"proc = subprocess.Popen({launch_command!r}, shell=True, preexec_fn=os.setsid); "
            f"time.sleep({duration_s}); "
            "os.killpg(proc.pid, signal.SIGINT); "
            "proc.wait(timeout=20)"
        )
    )

    return " && ".join(
        [
            ros_setup,
            f"cd {shlex.quote(str(workspace))}",
            ws_setup,
            timeout_command,
        ]
    )


def run_single_experiment(command: str, workspace: Path) -> int:
    process = subprocess.run(
        ["/bin/bash", "-lc", command],
        cwd=workspace,
        check=False,
    )
    return int(process.returncode)


def main() -> int:
    args = parse_args()
    workspace = Path(args.workspace).expanduser().resolve()
    ensure_workspace(workspace)

    output_dir = (workspace / args.results_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    failures: list[str] = []
    seed = args.seed_start

    for controller in args.controllers:
        for run_index in range(args.runs_per_controller):
            command = build_launch_command(
                workspace=workspace,
                output_dir=output_dir,
                duration_s=args.duration_s,
                seed=seed,
                controller=controller,
                profile=args.profile,
                scenario=args.scenario,
                spawn_distance=args.spawn_distance,
                threat_response=args.threat_response,
                headless=args.headless,
            )

            print(
                f"[run_experiment] controller={controller} run={run_index + 1}/"
                f"{args.runs_per_controller} seed={seed}"
            )
            print(f"[run_experiment] command: {command}")

            if not args.dry_run:
                return_code = run_single_experiment(command, workspace)
                if return_code != 0:
                    failures.append(
                        f"{controller} run={run_index + 1} seed={seed} rc={return_code}"
                    )

            seed += 1
            time.sleep(1.0)

    if failures:
        print("[run_experiment] completed with failures:")
        for failure in failures:
            print(f"  - {failure}")
        return 1

    print(f"[run_experiment] completed. Results written to {output_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
