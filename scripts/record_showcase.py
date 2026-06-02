#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shlex
import shutil
import signal
import subprocess
import sys
import time


CONTROLLERS: dict[str, list[str]] = {
    "baseline_pure_pursuit": ["controller_preset:=baseline_pure_pursuit"],
    "baseline_lead_pursuit": ["controller_preset:=baseline_lead_pursuit"],
    "baseline_acceleration_aware": ["controller_preset:=baseline_acceleration_aware"],
    "mpc": ["controller_preset:=mpc_default"],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record one showcase run for each controller/guidance method."
    )
    parser.add_argument(
        "--workspace",
        default="/home/liam/ros2_drone_intercept_ws",
        help="ROS 2 workspace root.",
    )
    parser.add_argument(
        "--output-dir",
        default="recordings/drone_interceptor_showcase",
        help="Directory for saved video files.",
    )
    parser.add_argument(
        "--duration-s",
        type=float,
        default=60.0,
        help="Recording duration per controller.",
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
        help="Random seed for the first showcase run.",
    )
    parser.add_argument(
        "--open-plot",
        action="store_true",
        help="Keep rqt_plot open during recording.",
    )
    parser.add_argument(
        "--controllers",
        nargs="+",
        choices=sorted(CONTROLLERS),
        default=list(CONTROLLERS),
        help="Controllers to record.",
    )
    return parser.parse_args()


def ensure_prerequisites(workspace: Path) -> None:
    if shutil.which("ffmpeg") is None:
        raise RuntimeError("ffmpeg is required but was not found on PATH.")
    if shutil.which("xdpyinfo") is None:
        raise RuntimeError("xdpyinfo is required but was not found on PATH.")
    if not os.environ.get("DISPLAY"):
        raise RuntimeError("DISPLAY is not set; screen recording requires X11.")
    if not (workspace / "install" / "setup.bash").exists():
        raise RuntimeError(
            f"Workspace overlay missing: {workspace / 'install' / 'setup.bash'}"
        )


def detect_screen_size() -> str:
    result = subprocess.run(
        ["xdpyinfo"],
        check=True,
        capture_output=True,
        text=True,
    )
    for line in result.stdout.splitlines():
        if "dimensions:" in line:
            return line.split("dimensions:")[1].strip().split()[0]
    raise RuntimeError("Could not detect screen dimensions from xdpyinfo output.")


def build_launch_command(
    *,
    workspace: Path,
    controller: str,
    seed: int,
    profile: str,
    scenario: str,
    spawn_distance: float,
    threat_response: str,
    open_plot: bool,
) -> str:
    parts = [
        "source /opt/ros/humble/setup.bash",
        f"cd {shlex.quote(str(workspace))}",
        f"source {shlex.quote(str(workspace / 'install' / 'setup.bash'))}",
        (
            "ros2 launch drone_interceptor target_sim.launch.py "
            f"profile:={profile} "
            f"scenario:={scenario} "
            f"spawn_distance:={spawn_distance} "
            f"threat_response:={threat_response} "
            f"random_seed:={seed} "
            f"open_plot:={'true' if open_plot else 'false'} "
            + " ".join(CONTROLLERS[controller])
        ),
    ]
    return " && ".join(parts)


def build_ffmpeg_command(
    *,
    output_path: Path,
    duration_s: float,
    display: str,
    screen_size: str,
) -> list[str]:
    input_display = f"{display}.0+0,0" if "." not in display else f"{display}+0,0"
    return [
        "ffmpeg",
        "-y",
        "-video_size",
        screen_size,
        "-framerate",
        "30",
        "-f",
        "x11grab",
        "-i",
        input_display,
        "-t",
        str(duration_s),
        "-pix_fmt",
        "yuv420p",
        str(output_path),
    ]


def terminate_process_group(process: subprocess.Popen[bytes], sig: int) -> None:
    try:
        os.killpg(process.pid, sig)
    except ProcessLookupError:
        return


def record_one_run(
    *,
    workspace: Path,
    output_path: Path,
    duration_s: float,
    controller: str,
    seed: int,
    profile: str,
    scenario: str,
    spawn_distance: float,
    threat_response: str,
    open_plot: bool,
    display: str,
    screen_size: str,
) -> None:
    launch_command = build_launch_command(
        workspace=workspace,
        controller=controller,
        seed=seed,
        profile=profile,
        scenario=scenario,
        spawn_distance=spawn_distance,
        threat_response=threat_response,
        open_plot=open_plot,
    )
    launch_process = subprocess.Popen(
        ["/bin/bash", "-lc", launch_command],
        cwd=workspace,
        preexec_fn=os.setsid,
    )

    # Give RViz and the launch graph a moment to appear before recording.
    time.sleep(3.0)

    ffmpeg_command = build_ffmpeg_command(
        output_path=output_path,
        duration_s=duration_s,
        display=display,
        screen_size=screen_size,
    )
    record_process = subprocess.Popen(ffmpeg_command)

    try:
        record_process.wait(timeout=duration_s + 10.0)
    finally:
        terminate_process_group(launch_process, signal.SIGINT)
        try:
            launch_process.wait(timeout=20.0)
        except subprocess.TimeoutExpired:
            terminate_process_group(launch_process, signal.SIGTERM)
            launch_process.wait(timeout=10.0)


def main() -> int:
    args = parse_args()
    workspace = Path(args.workspace).expanduser().resolve()
    ensure_prerequisites(workspace)

    output_dir = (workspace / args.output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    display = os.environ["DISPLAY"]
    screen_size = detect_screen_size()
    seed = args.seed_start

    for controller in args.controllers:
        output_path = output_dir / f"{controller}.mp4"
        print(
            f"[record_showcase] recording {controller} for {args.duration_s:.1f}s "
            f"to {output_path}"
        )
        record_one_run(
            workspace=workspace,
            output_path=output_path,
            duration_s=args.duration_s,
            controller=controller,
            seed=seed,
            profile=args.profile,
            scenario=args.scenario,
            spawn_distance=args.spawn_distance,
            threat_response=args.threat_response,
            open_plot=args.open_plot,
            display=display,
            screen_size=screen_size,
        )
        seed += 1
        time.sleep(2.0)

    print(f"[record_showcase] saved recordings in {output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
