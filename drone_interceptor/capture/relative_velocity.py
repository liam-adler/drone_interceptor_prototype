from __future__ import annotations

import numpy as np


def relative_velocity(
    velocity_a: np.ndarray,
    velocity_b: np.ndarray,
) -> np.ndarray:
    return velocity_a - velocity_b


def relative_speed(
    velocity_a: np.ndarray,
    velocity_b: np.ndarray,
) -> float:
    return float(np.linalg.norm(relative_velocity(velocity_a, velocity_b)))
