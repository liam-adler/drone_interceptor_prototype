from __future__ import annotations

import numpy as np


def build_position_measurement_matrix() -> np.ndarray:
    """Return the linear measurement model for position-only measurements."""
    measurement_matrix = np.zeros((3, 6), dtype=float)
    measurement_matrix[:, 0:3] = np.eye(3, dtype=float)
    return measurement_matrix


def build_measurement_covariance(
    *,
    position_stddev: float,
    minimum_variance: float = 1e-9,
) -> np.ndarray:
    """Build a diagonal covariance for direct position measurements."""
    position_variance = max(float(position_stddev) ** 2, minimum_variance)
    return np.eye(3, dtype=float) * position_variance
