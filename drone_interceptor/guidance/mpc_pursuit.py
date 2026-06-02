"""Backward-compatible wrapper for the MPC guidance module."""

from drone_interceptor.guidance.mpc_guidance import (
    AccelerationMPC,
    MpcConfig,
    MpcSolution,
    MpcWeights,
)

__all__ = ["AccelerationMPC", "MpcConfig", "MpcSolution", "MpcWeights"]
