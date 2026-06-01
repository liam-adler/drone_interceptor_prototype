import os
from datetime import datetime
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


PROFILES = {
    "matched": {
        "target_min_speed": 1.0,
        "target_max_speed": 4.0,
        "target_max_accel": 3.0,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_faster": {
        "target_min_speed": 1.5,
        "target_max_speed": 4.5,
        "target_max_accel": 3.0,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_more_maneuverable": {
        "target_min_speed": 1.0,
        "target_max_speed": 4.0,
        "target_max_accel": 3.8,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
    "target_advantaged": {
        "target_min_speed": 1.5,
        "target_max_speed": 4.5,
        "target_max_accel": 3.5,
        "interceptor_max_speed": 4.0,
        "interceptor_max_accel": 3.0,
    },
}


UNSET = "__unset__"


def _parse_bool(value: str) -> bool:
    return value.lower() in {"1", "true", "yes", "on"}


def _load_yaml(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"Expected mapping in config file '{path}'")
    return data


def _load_launch_config(package_share: str) -> dict[str, dict]:
    config_dir = Path(package_share) / "config"
    defaults = _load_yaml(config_dir / "default.yaml").get("launch_defaults", {})
    scenarios = _load_yaml(config_dir / "target_scenarios.yaml").get("scenarios", {})
    controller_presets = _load_yaml(config_dir / "guidance_methods.yaml").get(
        "controller_presets", {}
    )
    experiments = _load_yaml(config_dir / "experiments.yaml").get("experiments", {})

    return {
        "defaults": defaults,
        "scenarios": scenarios,
        "controller_presets": controller_presets,
        "experiments": experiments,
    }


def _get_launch_value(context, name: str) -> str | None:
    value = LaunchConfiguration(name).perform(context)
    return None if value == UNSET else value


def _resolve_runtime_config(context, launch_config: dict[str, dict]) -> dict[str, object]:
    resolved: dict[str, object] = dict(launch_config["defaults"])

    experiment_name = _get_launch_value(context, "experiment")
    if experiment_name is not None:
        experiment = launch_config["experiments"].get(experiment_name)
        if experiment is None:
            valid = ", ".join(sorted(launch_config["experiments"]))
            raise RuntimeError(
                f"Unknown experiment '{experiment_name}'. Choose one of: {valid}"
            )
        if not isinstance(experiment, dict):
            raise RuntimeError(f"Experiment '{experiment_name}' must be a mapping")
        resolved["experiment"] = experiment_name
        resolved.update(experiment)

    scenario_name = _get_launch_value(context, "scenario")
    if scenario_name is None:
        scenario_name = resolved.get("scenario")
    if scenario_name is not None:
        scenario = launch_config["scenarios"].get(str(scenario_name))
        if scenario is None:
            valid = ", ".join(sorted(launch_config["scenarios"]))
            raise RuntimeError(
                f"Unknown scenario '{scenario_name}'. Choose one of: {valid}"
            )
        if not isinstance(scenario, dict):
            raise RuntimeError(f"Scenario '{scenario_name}' must be a mapping")
        resolved["scenario"] = scenario_name
        resolved.update(scenario)

    controller_preset = _get_launch_value(context, "controller_preset")
    if controller_preset is None:
        controller_preset = resolved.get("controller_preset")
    if controller_preset is not None:
        preset = launch_config["controller_presets"].get(str(controller_preset))
        if preset is None:
            valid = ", ".join(sorted(launch_config["controller_presets"]))
            raise RuntimeError(
                f"Unknown controller_preset '{controller_preset}'. Choose one of: {valid}"
            )
        if not isinstance(preset, dict):
            raise RuntimeError(
                f"Controller preset '{controller_preset}' must be a mapping"
            )
        resolved["controller_preset"] = controller_preset
        resolved.update(preset)

    for key in (
        "profile",
        "controller_mode",
        "guidance_mode",
        "threat_response",
        "spawn_distance",
        "random_seed",
        "open_rviz",
        "open_plot",
        "output_dir",
    ):
        explicit_value = _get_launch_value(context, key)
        if explicit_value is not None:
            resolved[key] = explicit_value

    return resolved


def _make_node(executable: str, name: str, parameters: dict | None = None) -> Node:
    return Node(
        package="drone_interceptor",
        executable=executable,
        name=name,
        output="screen",
        parameters=[parameters or {}],
    )


def _make_dynamics_node(
    *,
    executable: str,
    name: str,
    initial_x: float,
    max_speed: float,
    max_accel: float,
) -> Node:
    return _make_node(
        executable=executable,
        name=name,
        parameters={
            "initial_x": initial_x,
            "initial_y": 0.0,
            "initial_z": 2.0,
            "max_speed": max_speed,
            "max_accel": max_accel,
        },
    )


def _make_distance_monitor_node(interceptor_state_topic: str) -> Node:
    return _make_node(
        executable="distance_monitor_node",
        name="distance_monitor_node",
        parameters={
            "target_state_topic": "/target/state",
            "interceptor_state_topic": interceptor_state_topic,
            "distance_topic": "/intercept/distance",
        },
    )


def _make_estimator_node(
    *,
    input_state_topic: str,
    measurement_topic: str,
    estimated_state_topic: str,
    noise_enabled: bool,
    position_noise_stddev: float,
    filter_enabled: bool,
    process_acceleration_stddev: float,
    measurement_position_stddev: float,
    initial_position_stddev: float,
    initial_velocity_stddev: float,
    random_seed: int,
) -> Node:
    return _make_node(
        executable="estimator_node",
        name="estimator_node",
        parameters={
            "input_state_topic": input_state_topic,
            "measurement_topic": measurement_topic,
            "estimated_state_topic": estimated_state_topic,
            "noise_enabled": noise_enabled,
            "position_noise_stddev": position_noise_stddev,
            "filter_enabled": filter_enabled,
            "process_acceleration_stddev": process_acceleration_stddev,
            "measurement_position_stddev": measurement_position_stddev,
            "initial_position_stddev": initial_position_stddev,
            "initial_velocity_stddev": initial_velocity_stddev,
            "random_seed": random_seed,
        },
    )


def _make_results_logger_node(
    *,
    controller_label: str,
    output_dir: str,
    run_label: str,
    profile_name: str,
    controller_mode: str,
    threat_response: bool,
    spawn_distance: float,
    random_seed: int,
) -> Node:
    return _make_node(
        executable="results_logger_node",
        name="results_logger_node",
        parameters={
            "distance_topic": "/intercept/distance",
            "controller_label": controller_label,
            "output_dir": output_dir,
            "run_label": run_label,
            "profile_name": profile_name,
            "controller_mode": controller_mode,
            "threat_response": threat_response,
            "spawn_distance": spawn_distance,
            "random_seed": random_seed,
        },
    )


def _build_nodes(context):
    package_share = get_package_share_directory("drone_interceptor")
    launch_config = _load_launch_config(package_share)
    runtime = _resolve_runtime_config(context, launch_config)

    profile_name = str(runtime.get("profile", "target_advantaged"))
    if profile_name not in PROFILES:
        valid_profiles = ", ".join(sorted(PROFILES))
        raise RuntimeError(
            f"Unknown profile '{profile_name}'. Choose one of: {valid_profiles}"
        )

    profile = PROFILES[profile_name]
    controller_mode = str(runtime.get("controller_mode", "baseline"))
    if controller_mode not in {"baseline", "mpc"}:
        raise RuntimeError(
            "Unknown controller_mode "
            f"'{controller_mode}'. Choose one of: baseline, mpc"
        )
    guidance_mode = str(runtime.get("guidance_mode", "lead_pursuit"))

    spawn_distance = float(runtime.get("spawn_distance", 30.0))
    spawn_distance_text = str(runtime.get("spawn_distance", 30.0))
    random_seed = int(runtime.get("random_seed", 0))
    open_rviz = _parse_bool(str(runtime.get("open_rviz", "true")))
    open_plot = _parse_bool(str(runtime.get("open_plot", "true")))
    threat_response = _parse_bool(str(runtime.get("threat_response", "true")))
    output_dir = str(runtime.get("output_dir", "results/drone_interceptor"))
    target_input_state_topic = str(runtime.get("target_input_state_topic", "/target/state"))
    target_measurement_topic = str(
        runtime.get("target_measurement_topic", "/target/state_noisy")
    )
    target_estimated_state_topic = str(
        runtime.get("target_estimated_state_topic", "/target/estimated_state")
    )
    target_noise_enabled = _parse_bool(
        str(runtime.get("target_noise_enabled", "false"))
    )
    target_noise_position_stddev = float(
        runtime.get("target_noise_position_stddev", 0.0)
    )
    target_filter_enabled = _parse_bool(
        str(runtime.get("target_filter_enabled", "true"))
    )
    target_filter_process_accel_stddev = float(
        runtime.get("target_filter_process_accel_stddev", 2.0)
    )
    target_filter_measurement_position_stddev = float(
        runtime.get("target_filter_measurement_position_stddev", -1.0)
    )
    target_filter_initial_position_stddev = float(
        runtime.get("target_filter_initial_position_stddev", 1.0)
    )
    target_filter_initial_velocity_stddev = float(
        runtime.get("target_filter_initial_velocity_stddev", 1.0)
    )
    scenario_name = runtime.get("scenario")
    controller_preset = runtime.get("controller_preset")
    experiment_name = runtime.get("experiment")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    threat_text = "threat_on" if threat_response else "threat_off"
    spawn_text = spawn_distance_text.replace(".", "p")
    scenario_text = (
        f"__scenario_{scenario_name}" if scenario_name is not None else ""
    )
    preset_text = (
        f"__preset_{controller_preset}" if controller_preset is not None else ""
    )
    experiment_text = (
        f"__experiment_{experiment_name}" if experiment_name is not None else ""
    )
    run_label = (
        f"{timestamp}__mode_{controller_mode}__profile_{profile_name}"
        f"__guidance_{guidance_mode}"
        f"{scenario_text}{preset_text}{experiment_text}"
        f"__{threat_text}__spawn_{spawn_text}m__seed_{random_seed}"
    )

    interceptor_state_topic = (
        "/interceptor/state"
        if controller_mode == "baseline"
        else "/interceptor_mpc/state"
    )
    rviz_config_name = (
        "target_sim.rviz" if controller_mode == "baseline" else "target_sim_mpc.rviz"
    )
    rviz_config = os.path.join(package_share, "rviz", rviz_config_name)
    summary_lines = [
        "Resolved launch config:",
        f"  experiment={experiment_name if experiment_name is not None else 'none'}",
        f"  scenario={scenario_name if scenario_name is not None else 'none'}",
        f"  controller_preset={controller_preset if controller_preset is not None else 'none'}",
        f"  profile={profile_name}",
        f"  controller_mode={controller_mode}",
        f"  guidance_mode={guidance_mode}",
        f"  spawn_distance={spawn_distance}",
        f"  threat_response={threat_response}",
        f"  random_seed={random_seed}",
        f"  target_input_state_topic={target_input_state_topic}",
        f"  target_measurement_topic={target_measurement_topic}",
        f"  target_estimated_state_topic={target_estimated_state_topic}",
        f"  target_noise_enabled={target_noise_enabled}",
        f"  target_noise_position_stddev={target_noise_position_stddev}",
        f"  target_filter_enabled={target_filter_enabled}",
        f"  open_rviz={open_rviz}",
        f"  open_plot={open_plot}",
        f"  output_dir={output_dir}",
    ]

    nodes = [
        LogInfo(msg="\n".join(summary_lines)),
        _make_dynamics_node(
            executable="target_dynamics_node",
            name="target_dynamics_node",
            initial_x=0.0,
            max_speed=profile["target_max_speed"],
            max_accel=profile["target_max_accel"],
        ),
        _make_node(
            executable="target_behavior_node",
            name="target_behavior_node",
            parameters={
                "min_speed": profile["target_min_speed"],
                "max_speed": profile["target_max_speed"],
                "enable_threat_response": threat_response,
                "threat_radius": 12.0,
                "escape_speed_min": 0.9,
                "interceptor_state_topic": interceptor_state_topic,
                "random_seed": random_seed,
            },
        ),
        _make_estimator_node(
            input_state_topic=target_input_state_topic,
            measurement_topic=target_measurement_topic,
            estimated_state_topic=target_estimated_state_topic,
            noise_enabled=target_noise_enabled,
            position_noise_stddev=target_noise_position_stddev,
            filter_enabled=target_filter_enabled,
            process_acceleration_stddev=target_filter_process_accel_stddev,
            measurement_position_stddev=target_filter_measurement_position_stddev,
            initial_position_stddev=target_filter_initial_position_stddev,
            initial_velocity_stddev=target_filter_initial_velocity_stddev,
            random_seed=random_seed,
        ),
    ]

    if controller_mode == "baseline":
        nodes.extend([
            _make_dynamics_node(
                executable="interceptor_dynamics_node",
                name="interceptor_dynamics_node",
                initial_x=-spawn_distance,
                max_speed=profile["interceptor_max_speed"],
                max_accel=profile["interceptor_max_accel"],
            ),
            _make_node(
                executable="interceptor_guidance_node",
                name="interceptor_guidance_node",
                parameters={
                    "mode": guidance_mode,
                    "target_state_topic": target_estimated_state_topic,
                    "interceptor_speed": profile["interceptor_max_speed"],
                    "interceptor_max_acceleration": profile["interceptor_max_accel"],
                },
            ),
            _make_distance_monitor_node("/interceptor/state"),
            _make_results_logger_node(
                controller_label="baseline",
                output_dir=output_dir,
                run_label=run_label,
                profile_name=profile_name,
                controller_mode=controller_mode,
                threat_response=threat_response,
                spawn_distance=spawn_distance,
                random_seed=random_seed,
            ),
        ])
    else:
        nodes.extend([
            _make_dynamics_node(
                executable="interceptor_mpc_dynamics_node",
                name="interceptor_mpc_dynamics_node",
                initial_x=-spawn_distance,
                max_speed=profile["interceptor_max_speed"],
                max_accel=profile["interceptor_max_accel"],
            ),
            _make_node(
                executable="interceptor_mpc_node",
                name="interceptor_mpc_node",
                parameters={
                    "target_state_topic": target_estimated_state_topic,
                    "max_speed": profile["interceptor_max_speed"],
                    "max_accel": profile["interceptor_max_accel"],
                },
            ),
            _make_distance_monitor_node("/interceptor_mpc/state"),
            _make_results_logger_node(
                controller_label="mpc",
                output_dir=output_dir,
                run_label=run_label,
                profile_name=profile_name,
                controller_mode=controller_mode,
                threat_response=threat_response,
                spawn_distance=spawn_distance,
                random_seed=random_seed,
            ),
        ])

    if open_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            )
        )

    if open_plot:
        nodes.append(
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "run",
                            "rqt_plot",
                            "rqt_plot",
                        ],
                        output="screen",
                    )
                ],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scenario",
            default_value=UNSET,
            description="Named scenario from config/target_scenarios.yaml",
        ),
        DeclareLaunchArgument(
            "controller_preset",
            default_value=UNSET,
            description="Named controller preset from config/guidance_methods.yaml",
        ),
        DeclareLaunchArgument(
            "experiment",
            default_value=UNSET,
            description="Named experiment from config/experiments.yaml",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value=UNSET,
            description=(
                "Vehicle capability profile override: matched, target_faster, "
                "target_more_maneuverable, target_advantaged"
            ),
        ),
        DeclareLaunchArgument(
            "controller_mode",
            default_value=UNSET,
            description="Interceptor controller override: baseline or mpc",
        ),
        DeclareLaunchArgument(
            "guidance_mode",
            default_value=UNSET,
            description=(
                "Baseline guidance override: pure_pursuit, lead_pursuit, "
                "or acceleration_aware_lead_pursuit"
            ),
        ),
        DeclareLaunchArgument(
            "threat_response",
            default_value=UNSET,
            description="Enable close-range target speed boost override",
        ),
        DeclareLaunchArgument(
            "spawn_distance",
            default_value=UNSET,
            description="Initial separation distance override in meters",
        ),
        DeclareLaunchArgument(
            "random_seed",
            default_value=UNSET,
            description="Random seed override for target behavior",
        ),
        DeclareLaunchArgument(
            "open_rviz",
            default_value=UNSET,
            description="Open RViz override",
        ),
        DeclareLaunchArgument(
            "open_plot",
            default_value=UNSET,
            description="Open rqt_plot override",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value=UNSET,
            description="Output directory override for run artifacts",
        ),
        OpaqueFunction(function=_build_nodes),
    ])
