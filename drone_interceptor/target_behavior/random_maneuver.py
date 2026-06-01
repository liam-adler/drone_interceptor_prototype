from __future__ import annotations

import numpy as np

from drone_interceptor.core.math_utils import HEADING_ZERO_TOLERANCE, safe_normalize

DEFAULT_HEADING = np.array([1.0, 0.0, 0.0], dtype=float)


def sample_random_heading(rng: np.random.Generator) -> np.ndarray:
    return np.array(
        [
            rng.uniform(-1.0, 1.0),
            rng.uniform(-1.0, 1.0),
            rng.uniform(-0.2, 0.2),
        ],
        dtype=float,
    )


def normalized_heading(direction: np.ndarray) -> np.ndarray | None:
    return safe_normalize(direction, tolerance=HEADING_ZERO_TOLERANCE)


def sample_cruise_speed(
    rng: np.random.Generator,
    *,
    min_speed: float,
    max_speed: float,
) -> float:
    return float(rng.uniform(min_speed, max_speed))


def compute_desired_speed(
    *,
    current_cruise_speed: float,
    max_speed: float,
    enable_threat_response: bool,
    threat_radius: float,
    escape_speed_min_ratio: float,
    target_position: np.ndarray | None,
    interceptor_position: np.ndarray | None,
) -> float:
    if (
        not enable_threat_response
        or target_position is None
        or interceptor_position is None
    ):
        return current_cruise_speed

    distance = float(np.linalg.norm(target_position - interceptor_position))
    if distance >= threat_radius:
        return current_cruise_speed

    distance_ratio = max(distance, 0.0) / max(threat_radius, HEADING_ZERO_TOLERANCE)
    urgency = 1.0 - distance_ratio
    escape_floor = max_speed * escape_speed_min_ratio
    boosted_speed = current_cruise_speed + urgency * (max_speed - current_cruise_speed)
    return float(np.clip(boosted_speed, escape_floor, max_speed))
