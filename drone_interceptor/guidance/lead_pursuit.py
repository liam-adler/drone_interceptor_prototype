from __future__ import annotations

import numpy as np


def compute_lead_pursuit_aim_point(
    *,
    target_position: np.ndarray,
    target_velocity: np.ndarray,
    interceptor_position: np.ndarray,
    interceptor_speed: float,
    min_prediction_time: float,
    max_prediction_time: float,
    norm_tolerance: float,
) -> np.ndarray:
    """
    Constant-velocity lead pursuit.

    Solves approximately:

        ||target_position + target_velocity * t - interceptor_position||
        =
        interceptor_speed * t

    If no good solution exists, falls back to a bounded prediction time.
    """

    relative_position = target_position - interceptor_position

    a = float(np.dot(target_velocity, target_velocity) - interceptor_speed**2)
    b = float(2.0 * np.dot(relative_position, target_velocity))
    c = float(np.dot(relative_position, relative_position))

    candidate_times: list[float] = []

    if abs(a) < norm_tolerance:
        if abs(b) > norm_tolerance:
            t_go = -c / b
            if t_go > 0.0:
                candidate_times.append(t_go)
    else:
        discriminant = b**2 - 4.0 * a * c

        if discriminant >= 0.0:
            sqrt_discriminant = float(np.sqrt(discriminant))

            t1 = (-b - sqrt_discriminant) / (2.0 * a)
            t2 = (-b + sqrt_discriminant) / (2.0 * a)

            if t1 > 0.0:
                candidate_times.append(t1)

            if t2 > 0.0:
                candidate_times.append(t2)

    if candidate_times:
        t_go = min(candidate_times)
    else:
        distance = float(np.linalg.norm(relative_position))
        t_go = distance / max(interceptor_speed, 1e-6)

    t_go = float(np.clip(t_go, min_prediction_time, max_prediction_time))
    return target_position + target_velocity * t_go
