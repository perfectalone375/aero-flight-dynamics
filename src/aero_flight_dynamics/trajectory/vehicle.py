"""
Point-mass vehicle model combining structural, aerodynamic, and propulsive parameters.
"""

from dataclasses import dataclass, field
from typing import Optional

from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.propulsion.solid_motor import SolidRocketMotor


@dataclass
class PointMassVehicle:
    """
    Parametric flight vehicle model for trajectory simulation.
    """
    name: str = "Suborbital Research Vehicle"
    structural_mass_kg: float = 25.0       # Vehicle body, payload, avionics mass
    reference_area_m2: float = 0.0314      # Cross-sectional aerodynamic reference area (S)
    characteristic_length_m: float = 2.5   # Characteristic length (chord or diameter)
    aerodynamics: AeroCoefficients = field(default_factory=AeroCoefficients)
    motor: Optional[SolidRocketMotor] = None

    def total_mass(self, time_s: float) -> float:
        """
        Total vehicle instantaneous mass = structural_mass + motor_mass(t).
        """
        if self.motor is None:
            return self.structural_mass_kg
        return self.structural_mass_kg + self.motor.current_motor_mass(time_s)
