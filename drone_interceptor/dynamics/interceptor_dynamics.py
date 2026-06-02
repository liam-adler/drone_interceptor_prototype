"""Backward-compatible wrapper for the point-mass dynamics model module."""

from drone_interceptor.dynamics.point_mass_dynamics_model import (
    PointMassState,
    SimplePointMassDynamics,
)

__all__ = ["PointMassState", "SimplePointMassDynamics"]
