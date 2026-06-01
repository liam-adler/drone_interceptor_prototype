from __future__ import annotations

import numpy as np


def compute_distance(position_a: np.ndarray, position_b: np.ndarray) -> float:
    return float(np.linalg.norm(position_a - position_b))


def is_within_capture_radius(
    position_a: np.ndarray,
    position_b: np.ndarray,
    capture_radius: float,
) -> bool:
    return compute_distance(position_a, position_b) <= capture_radius
