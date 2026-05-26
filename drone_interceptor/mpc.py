from __future__ import annotations

from dataclasses import dataclass

import numpy as np


Array3 = np.ndarray


@dataclass
class MpcWeights:
    position: float = 7.0
    relative_velocity: float = 0.25
    non_closing_rate: float = 10.0
    capture_set: float = 22.0
    control: float = 0.008
    control_delta: float = 0.025
    terminal_position: float = 70.0
    terminal_relative_velocity: float = 0.6
    terminal_capture_set: float = 80.0


@dataclass
class MpcConfig:
    horizon_steps: int = 24
    dt: float = 0.12
    iterations: int = 48
    step_size: float = 0.045
    max_speed: float = 4.0
    max_accel: float = 3.0
    min_z: float = 0.3
    max_z: float = 20.0
    capture_radius: float = 0.5


@dataclass
class MpcSolution:
    first_acceleration: Array3
    predicted_positions: list[Array3]
    predicted_target_positions: list[Array3]
    cost: float


class AccelerationMPC:
    """
    Small projected-gradient MPC for a double-integrator interceptor model.

    State:
        x = [px, py, pz, vx, vy, vz]

    Control:
        u = [ax, ay, az]
    """

    def __init__(self, config: MpcConfig, weights: MpcWeights) -> None:
        self.config = config
        self.weights = weights
        self._last_solution = np.zeros((self.config.horizon_steps, 3), dtype=float)

    def solve(
        self,
        interceptor_position: Array3,
        interceptor_velocity: Array3,
        target_position: Array3,
        target_velocity: Array3,
    ) -> MpcSolution:
        target_positions = self._predict_target_positions(
            target_position=target_position,
            target_velocity=target_velocity,
        )
        target_velocities = [
            target_velocity.copy() for _ in range(self.config.horizon_steps)
        ]

        controls = self._warm_start()

        for _ in range(self.config.iterations):
            states = self._rollout(
                interceptor_position=interceptor_position,
                interceptor_velocity=interceptor_velocity,
                controls=controls,
            )
            gradients = self._compute_gradients(
                states=states,
                controls=controls,
                target_positions=target_positions,
                target_velocities=target_velocities,
            )
            controls -= self.config.step_size * gradients
            controls = self._project_controls(controls)

        states = self._rollout(
            interceptor_position=interceptor_position,
            interceptor_velocity=interceptor_velocity,
            controls=controls,
        )
        self._last_solution = controls.copy()

        return MpcSolution(
            first_acceleration=controls[0].copy(),
            predicted_positions=[state[:3].copy() for state in states[1:]],
            predicted_target_positions=[target.copy() for target in target_positions],
            cost=self._compute_total_cost(
                states=states,
                controls=controls,
                target_positions=target_positions,
                target_velocities=target_velocities,
            ),
        )

    def _warm_start(self) -> np.ndarray:
        controls = np.zeros((self.config.horizon_steps, 3), dtype=float)
        controls[:-1] = self._last_solution[1:]
        controls[-1] = self._last_solution[-1]
        return controls

    def _predict_target_positions(
        self,
        target_position: Array3,
        target_velocity: Array3,
    ) -> list[Array3]:
        dt = self.config.dt
        return [
            target_position + target_velocity * dt * (step + 1)
            for step in range(self.config.horizon_steps)
        ]

    def _rollout(
        self,
        interceptor_position: Array3,
        interceptor_velocity: Array3,
        controls: np.ndarray,
    ) -> list[np.ndarray]:
        dt = self.config.dt
        states: list[np.ndarray] = [
            np.concatenate(
                [
                    interceptor_position.astype(float, copy=True),
                    interceptor_velocity.astype(float, copy=True),
                ]
            )
        ]

        for control in controls:
            previous = states[-1]
            position = previous[:3]
            velocity = previous[3:]

            next_velocity = velocity + control * dt
            next_velocity = self._limit_norm(next_velocity, self.config.max_speed)
            next_position = position + next_velocity * dt
            next_position[2] = float(
                np.clip(next_position[2], self.config.min_z, self.config.max_z)
            )

            if next_position[2] <= self.config.min_z and next_velocity[2] < 0.0:
                next_velocity[2] = 0.0
            if next_position[2] >= self.config.max_z and next_velocity[2] > 0.0:
                next_velocity[2] = 0.0

            states.append(np.concatenate([next_position, next_velocity]))

        return states

    def _compute_gradients(
        self,
        states: list[np.ndarray],
        controls: np.ndarray,
        target_positions: list[Array3],
        target_velocities: list[Array3],
    ) -> np.ndarray:
        dt = self.config.dt
        a_t = np.block(
            [
                [np.eye(3), dt * np.eye(3)],
                [np.zeros((3, 3)), np.eye(3)],
            ]
        )
        b_t = np.vstack([dt * dt * np.eye(3), dt * np.eye(3)])

        gradients = np.zeros_like(controls)
        lambda_next = self._terminal_cost_gradient(
            state=states[-1],
            target_position=target_positions[-1],
            target_velocity=target_velocities[-1],
        )

        for index in range(self.config.horizon_steps - 1, -1, -1):
            state = states[index + 1]
            control = controls[index]
            target_position = target_positions[index]
            target_velocity = target_velocities[index]

            stage_state_gradient = self._stage_cost_gradient_state(
                state=state,
                target_position=target_position,
                target_velocity=target_velocity,
            )
            stage_control_gradient = self._stage_cost_gradient_control(
                controls=controls,
                index=index,
            )

            lambda_current = stage_state_gradient + a_t.T @ lambda_next
            gradients[index] = stage_control_gradient + b_t.T @ lambda_next
            lambda_next = lambda_current

        return gradients

    def _stage_cost_gradient_state(
        self,
        state: np.ndarray,
        target_position: Array3,
        target_velocity: Array3,
    ) -> np.ndarray:
        relative_position = state[:3] - target_position
        relative_velocity = state[3:] - target_velocity
        distance = float(np.linalg.norm(relative_position))

        position_gradient = 2.0 * self.weights.position * relative_position
        velocity_gradient = 2.0 * self.weights.relative_velocity * relative_velocity

        position_gradient += self._capture_set_gradient(
            relative_position=relative_position,
            distance=distance,
            weight=self.weights.capture_set,
        )

        non_closing_position_gradient, non_closing_velocity_gradient = (
            self._non_closing_rate_gradient(
                relative_position=relative_position,
                relative_velocity=relative_velocity,
                distance=distance,
                weight=self.weights.non_closing_rate,
            )
        )

        return np.concatenate(
            [
                position_gradient + non_closing_position_gradient,
                velocity_gradient + non_closing_velocity_gradient,
            ]
        )

    def _stage_cost_gradient_control(
        self,
        controls: np.ndarray,
        index: int,
    ) -> Array3:
        control = controls[index]
        gradient = 2.0 * self.weights.control * control

        previous_control = (
            controls[index - 1] if index > 0 else np.zeros(3, dtype=float)
        )
        gradient += 2.0 * self.weights.control_delta * (control - previous_control)

        if index + 1 < len(controls):
            next_control = controls[index + 1]
            gradient += 2.0 * self.weights.control_delta * (control - next_control)

        return gradient

    def _terminal_cost_gradient(
        self,
        state: np.ndarray,
        target_position: Array3,
        target_velocity: Array3,
    ) -> np.ndarray:
        relative_position = state[:3] - target_position
        relative_velocity = state[3:] - target_velocity
        distance = float(np.linalg.norm(relative_position))

        position_gradient = 2.0 * self.weights.terminal_position * relative_position
        velocity_gradient = (
            2.0 * self.weights.terminal_relative_velocity * relative_velocity
        )
        position_gradient += self._capture_set_gradient(
            relative_position=relative_position,
            distance=distance,
            weight=self.weights.terminal_capture_set,
        )

        non_closing_position_gradient, non_closing_velocity_gradient = (
            self._non_closing_rate_gradient(
                relative_position=relative_position,
                relative_velocity=relative_velocity,
                distance=distance,
                weight=self.weights.non_closing_rate,
            )
        )

        return np.concatenate(
            [
                position_gradient + non_closing_position_gradient,
                velocity_gradient + non_closing_velocity_gradient,
            ]
        )

    def _compute_total_cost(
        self,
        states: list[np.ndarray],
        controls: np.ndarray,
        target_positions: list[Array3],
        target_velocities: list[Array3],
    ) -> float:
        cost = 0.0

        for index, control in enumerate(controls):
            state = states[index + 1]
            relative_position = state[:3] - target_positions[index]
            relative_velocity = state[3:] - target_velocities[index]
            distance = float(np.linalg.norm(relative_position))
            control_delta = control - (
                controls[index - 1] if index > 0 else np.zeros(3, dtype=float)
            )

            cost += self.weights.position * float(relative_position @ relative_position)
            cost += self.weights.relative_velocity * float(
                relative_velocity @ relative_velocity
            )
            cost += self.weights.capture_set * self._capture_excess_cost(distance)
            cost += self.weights.non_closing_rate * self._non_closing_rate_cost(
                relative_position=relative_position,
                relative_velocity=relative_velocity,
                distance=distance,
            )
            cost += self.weights.control * float(control @ control)
            cost += self.weights.control_delta * float(control_delta @ control_delta)

        terminal_position_error = states[-1][:3] - target_positions[-1]
        terminal_velocity_error = states[-1][3:] - target_velocities[-1]
        terminal_distance = float(np.linalg.norm(terminal_position_error))
        cost += self.weights.terminal_position * float(
            terminal_position_error @ terminal_position_error
        )
        cost += self.weights.terminal_relative_velocity * float(
            terminal_velocity_error @ terminal_velocity_error
        )
        cost += self.weights.terminal_capture_set * self._capture_excess_cost(
            terminal_distance
        )
        cost += self.weights.non_closing_rate * self._non_closing_rate_cost(
            relative_position=terminal_position_error,
            relative_velocity=terminal_velocity_error,
            distance=terminal_distance,
        )

        return cost

    def _capture_excess_cost(self, distance: float) -> float:
        excess = max(distance - self.config.capture_radius, 0.0)
        return excess * excess

    def _capture_set_gradient(
        self,
        relative_position: Array3,
        distance: float,
        weight: float,
    ) -> Array3:
        if distance <= self.config.capture_radius or distance < 1e-9:
            return np.zeros(3, dtype=float)

        excess = distance - self.config.capture_radius
        return 2.0 * weight * excess * relative_position / distance

    def _non_closing_rate_cost(
        self,
        relative_position: Array3,
        relative_velocity: Array3,
        distance: float,
    ) -> float:
        if distance < 1e-9:
            return 0.0

        closing_rate = float(np.dot(relative_position, relative_velocity) / distance)
        penalty = max(closing_rate, 0.0)
        return penalty * penalty

    def _non_closing_rate_gradient(
        self,
        relative_position: Array3,
        relative_velocity: Array3,
        distance: float,
        weight: float,
    ) -> tuple[Array3, Array3]:
        if distance < 1e-9:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        closing_rate = float(np.dot(relative_position, relative_velocity) / distance)
        if closing_rate <= 0.0:
            return np.zeros(3, dtype=float), np.zeros(3, dtype=float)

        d_closing_d_position = (
            relative_velocity / distance
            - closing_rate * relative_position / max(distance * distance, 1e-9)
        )
        d_closing_d_velocity = relative_position / distance

        position_gradient = 2.0 * weight * closing_rate * d_closing_d_position
        velocity_gradient = 2.0 * weight * closing_rate * d_closing_d_velocity
        return position_gradient, velocity_gradient

    def _project_controls(self, controls: np.ndarray) -> np.ndarray:
        projected = np.zeros_like(controls)
        for index, control in enumerate(controls):
            projected[index] = self._limit_norm(control, self.config.max_accel)
        return projected

    @staticmethod
    def _limit_norm(vector: Array3, max_norm: float) -> Array3:
        norm = float(np.linalg.norm(vector))

        if norm < 1e-9:
            return np.zeros(3, dtype=float)

        if norm <= max_norm:
            return vector.copy()

        return vector / norm * max_norm
