from __future__ import annotations

import numpy as np

from drone_interceptor.estimation.kalman_filter import (
    ConstantVelocityKalmanFilter,
    KalmanFilterConfig,
)


def test_constant_velocity_kalman_filter_tracks_state() -> None:
    filter_ = ConstantVelocityKalmanFilter(
        KalmanFilterConfig(
            process_acceleration_stddev=0.4,
            measurement_position_stddev=0.2,
            initial_position_stddev=1.0,
            initial_velocity_stddev=1.0,
        )
    )

    true_position = np.array([0.0, 0.0, 2.0], dtype=float)
    true_velocity = np.array([1.5, -0.5, 0.2], dtype=float)
    dt = 0.1

    filter_.initialize(position=true_position, velocity=np.zeros(3, dtype=float))

    for _ in range(30):
        true_position = true_position + true_velocity * dt
        measured_position = true_position + np.array([0.05, -0.03, 0.02], dtype=float)
        filter_.predict(dt)
        filter_.update(position=measured_position)

    assert np.allclose(filter_.position, true_position, atol=0.12)
    assert np.allclose(filter_.velocity, true_velocity, atol=0.12)
