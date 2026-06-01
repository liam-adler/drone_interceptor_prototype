from __future__ import annotations

from dataclasses import dataclass

import numpy as np


Array3 = np.ndarray


@dataclass
class PointMassState:
    """
    Simple 3D point-mass state.

    position: [x, y, z]
    velocity: [vx, vy, vz]
    """

    position: Array3
    velocity: Array3


class SimplePointMassDynamics:
    """
    Acceleration-limited point-mass dynamics.

    Input:
        desired velocity command

    Output:
        updated position and velocity
    """

    def __init__(
        self,
        max_speed: float = 3.0,
        max_accel: float = 2.0,
        min_z: float = 0.0,
        max_z: float = 20.0,
    ) -> None:
        if max_speed <= 0.0:
            raise ValueError("max_speed must be positive")

        if max_accel <= 0.0:
            raise ValueError("max_accel must be positive")

        if min_z > max_z:
            raise ValueError("min_z must be <= max_z")

        self.max_speed = float(max_speed)
        self.max_accel = float(max_accel)
        self.min_z = float(min_z)
        self.max_z = float(max_z)

    def step_velocity_command(
        self,
        state: PointMassState,
        velocity_command: Array3,
        dt: float,
    ) -> PointMassState:
        if dt <= 0.0:
            return PointMassState(
                position=state.position.copy(),
                velocity=state.velocity.copy(),
            )

        position = self._as_vector3(state.position)
        velocity = self._as_vector3(state.velocity)
        velocity_command = self._as_vector3(velocity_command)

        velocity_command = self._limit_norm(velocity_command, self.max_speed)

        desired_accel = (velocity_command - velocity) / dt
        accel = self._limit_norm(desired_accel, self.max_accel)

        new_velocity = velocity + accel * dt
        new_velocity = self._limit_norm(new_velocity, self.max_speed)

        new_position = position + new_velocity * dt

        new_position[2] = float(np.clip(new_position[2], self.min_z, self.max_z))

        if new_position[2] <= self.min_z and new_velocity[2] < 0.0:
            new_velocity[2] = 0.0

        if new_position[2] >= self.max_z and new_velocity[2] > 0.0:
            new_velocity[2] = 0.0

        return PointMassState(
            position=new_position,
            velocity=new_velocity,
        )

    def step_acceleration_command(
        self,
        state: PointMassState,
        acceleration_command: Array3,
        dt: float,
    ) -> PointMassState:
        if dt <= 0.0:
            return PointMassState(
                position=state.position.copy(),
                velocity=state.velocity.copy(),
            )

        position = self._as_vector3(state.position)
        velocity = self._as_vector3(state.velocity)
        acceleration_command = self._as_vector3(acceleration_command)

        accel = self._limit_norm(acceleration_command, self.max_accel)

        new_velocity = velocity + accel * dt
        new_velocity = self._limit_norm(new_velocity, self.max_speed)

        new_position = position + new_velocity * dt

        new_position[2] = float(np.clip(new_position[2], self.min_z, self.max_z))

        if new_position[2] <= self.min_z and new_velocity[2] < 0.0:
            new_velocity[2] = 0.0

        if new_position[2] >= self.max_z and new_velocity[2] > 0.0:
            new_velocity[2] = 0.0

        return PointMassState(
            position=new_position,
            velocity=new_velocity,
        )

    @staticmethod
    def _as_vector3(vector: Array3) -> Array3:
        arr = np.asarray(vector, dtype=float)

        if arr.shape != (3,):
            raise ValueError(f"Expected vector shape (3,), got {arr.shape}")

        return arr.copy()

    @staticmethod
    def _limit_norm(vector: Array3, max_norm: float) -> Array3:
        norm = float(np.linalg.norm(vector))

        if norm < 1e-9:
            return np.zeros(3, dtype=float)

        if norm <= max_norm:
            return vector.copy()

        return vector / norm * max_norm
