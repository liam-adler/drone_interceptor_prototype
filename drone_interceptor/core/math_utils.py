from __future__ import annotations

import numpy as np

Array3 = np.ndarray
NORM_TOLERANCE = 1e-9
HEADING_ZERO_TOLERANCE = 1e-6


def clamp_norm(vector: Array3, max_norm: float) -> Array3:
    norm = float(np.linalg.norm(vector))
    if norm < NORM_TOLERANCE:
        return np.zeros(3, dtype=float)
    if norm <= max_norm:
        return vector.copy()
    return vector / norm * max_norm


def safe_normalize(
    vector: Array3,
    *,
    tolerance: float = NORM_TOLERANCE,
) -> Array3 | None:
    norm = float(np.linalg.norm(vector))
    if norm < tolerance:
        return None
    return vector / norm
