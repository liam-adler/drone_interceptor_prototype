from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from drone_interceptor.estimation.measurement_model import (
    build_position_measurement_matrix,
    build_measurement_covariance,
)


STATE_DIMENSION = 6


@dataclass(frozen=True)
class KalmanFilterConfig:
    process_acceleration_stddev: float
    measurement_position_stddev: float
    initial_position_stddev: float
    initial_velocity_stddev: float


class ConstantVelocityKalmanFilter:
    """Linear Kalman filter with a constant-velocity process model."""

    def __init__(self, config: KalmanFilterConfig) -> None:
        self.config = config
        self.state = np.zeros(STATE_DIMENSION, dtype=float)
        self.covariance = np.eye(STATE_DIMENSION, dtype=float)
        self.measurement_matrix = build_position_measurement_matrix()
        self.measurement_covariance = build_measurement_covariance(
            position_stddev=config.measurement_position_stddev,
        )
        self.initialized = False

    def initialize(self, *, position: np.ndarray, velocity: np.ndarray) -> None:
        self.state[0:3] = np.asarray(position, dtype=float)
        self.state[3:6] = np.asarray(velocity, dtype=float)

        self.covariance = np.zeros((STATE_DIMENSION, STATE_DIMENSION), dtype=float)
        self.covariance[0:3, 0:3] = (
            np.eye(3, dtype=float)
            * max(self.config.initial_position_stddev**2, 1e-9)
        )
        self.covariance[3:6, 3:6] = (
            np.eye(3, dtype=float)
            * max(self.config.initial_velocity_stddev**2, 1e-9)
        )
        self.initialized = True

    def predict(self, dt: float) -> None:
        dt = max(float(dt), 0.0)
        if dt <= 0.0:
            return

        transition = np.eye(STATE_DIMENSION, dtype=float)
        transition[0:3, 3:6] = np.eye(3, dtype=float) * dt

        dt2 = dt * dt
        dt3 = dt2 * dt
        dt4 = dt2 * dt2
        acceleration_variance = max(
            float(self.config.process_acceleration_stddev) ** 2,
            1e-9,
        )

        process_noise = np.zeros((STATE_DIMENSION, STATE_DIMENSION), dtype=float)
        position_block = np.eye(3, dtype=float) * (0.25 * dt4 * acceleration_variance)
        cross_block = np.eye(3, dtype=float) * (0.5 * dt3 * acceleration_variance)
        velocity_block = np.eye(3, dtype=float) * (dt2 * acceleration_variance)
        process_noise[0:3, 0:3] = position_block
        process_noise[0:3, 3:6] = cross_block
        process_noise[3:6, 0:3] = cross_block
        process_noise[3:6, 3:6] = velocity_block

        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T + process_noise

    def update(self, *, position: np.ndarray) -> None:
        measurement = np.asarray(position, dtype=float)

        innovation = measurement - self.measurement_matrix @ self.state
        innovation_covariance = (
            self.measurement_matrix
            @ self.covariance
            @ self.measurement_matrix.T
            + self.measurement_covariance
        )
        kalman_gain = np.linalg.solve(
            innovation_covariance.T,
            self.measurement_matrix @ self.covariance,
        ).T

        self.state = self.state + kalman_gain @ innovation
        identity = np.eye(STATE_DIMENSION, dtype=float)
        self.covariance = (
            identity - kalman_gain @ self.measurement_matrix
        ) @ self.covariance

    @property
    def position(self) -> np.ndarray:
        return self.state[0:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6].copy()
