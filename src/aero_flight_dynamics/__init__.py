"""
aero_flight_dynamics: High-fidelity aerospace flight dynamics, atmospheric physics,
and trajectory simulation library.
"""

from aero_flight_dynamics.aerodynamics.airfoil import NACA4Airfoil
from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.atmosphere.models import AtmosphericState
from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.propulsion.solid_motor import MotorBurnProfile, SolidRocketMotor
from aero_flight_dynamics.telemetry.recorder import FlightRecorder, TelemetrySummary
from aero_flight_dynamics.trajectory.simulation import FlightSimulator, SimulationResult
from aero_flight_dynamics.trajectory.staging import (
    RocketStage,
    StagedRocketSimulator,
    StagedSimulationResult,
)
from aero_flight_dynamics.trajectory.state import FlightState
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle

__version__ = "0.1.0"
__author__ = "Gopi Naga Vignesh Thirupati"
__license__ = "MIT"

__all__ = [
    "AtmosphericState",
    "USStandardAtmosphere1976",
    "AeroCoefficients",
    "NACA4Airfoil",
    "SolidRocketMotor",
    "MotorBurnProfile",
    "FlightState",
    "PointMassVehicle",
    "FlightSimulator",
    "SimulationResult",
    "RocketStage",
    "StagedRocketSimulator",
    "StagedSimulationResult",
    "FlightRecorder",
    "TelemetrySummary",
]
