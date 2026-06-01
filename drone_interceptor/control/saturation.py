from __future__ import annotations

import numpy as np

from drone_interceptor.core.math_utils import Array3, clamp_norm


def saturate_vector_norm(vector: Array3, max_norm: float) -> Array3:
    return clamp_norm(vector, max_norm)


def saturate_velocity(velocity: Array3, max_speed: float) -> Array3:
    return clamp_norm(velocity, max_speed)


def saturate_acceleration(acceleration: Array3, max_accel: float) -> Array3:
    return clamp_norm(acceleration, max_accel)
