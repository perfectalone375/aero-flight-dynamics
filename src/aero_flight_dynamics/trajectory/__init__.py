"""Trajectory simulation and 6-DoF attitude dynamics module."""

from aero_flight_dynamics.trajectory.quaternions import (
    AttitudeDynamicsSimulator,
    AttitudeSimulationResult,
    AttitudeState,
    Quaternion,
    RigidBodyInertia,
)
from aero_flight_dynamics.trajectory.simulation import FlightSimulator, SimulationResult
from aero_flight_dynamics.trajectory.staging import (
    RocketStage,
    StagedRocketSimulator,
    StagedSimulationResult,
)
from aero_flight_dynamics.trajectory.state import FlightState
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle

__all__ = [
    "FlightState",
    "PointMassVehicle",
    "FlightSimulator",
    "SimulationResult",
    "RocketStage",
    "StagedRocketSimulator",
    "StagedSimulationResult",
    "Quaternion",
    "RigidBodyInertia",
    "AttitudeState",
    "AttitudeSimulationResult",
    "AttitudeDynamicsSimulator",
]
