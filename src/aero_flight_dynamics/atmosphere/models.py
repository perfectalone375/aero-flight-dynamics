"""
Atmospheric state data structures and base interfaces.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class AtmosphericState:
    """
    Immutable representation of atmospheric properties at a given altitude.
    """
    altitude_m: float
    geopotential_altitude_m: float
    temperature_k: float
    pressure_pa: float
    density_kg_m3: float
    speed_of_sound_m_s: float
    dynamic_viscosity_pa_s: float

    @property
    def kinematic_viscosity_m2_s(self) -> float:
        """Kinematic viscosity = dynamic_viscosity / density."""
        if self.density_kg_m3 <= 0:
            return float("inf")
        return self.dynamic_viscosity_pa_s / self.density_kg_m3

    def mach_number(self, velocity_m_s: float) -> float:
        """Compute Mach number for a given velocity."""
        if self.speed_of_sound_m_s <= 0:
            return 0.0
        return abs(velocity_m_s) / self.speed_of_sound_m_s

    def dynamic_pressure(self, velocity_m_s: float) -> float:
        """Compute dynamic pressure q = 0.5 * rho * v^2 in Pascals."""
        return 0.5 * self.density_kg_m3 * (velocity_m_s ** 2)

    def reynolds_number(self, velocity_m_s: float, characteristic_length_m: float) -> float:
        """Compute Reynolds number Re = (rho * v * L) / mu."""
        if self.dynamic_viscosity_pa_s <= 0:
            return 0.0
        return (self.density_kg_m3 * abs(velocity_m_s) * characteristic_length_m) / self.dynamic_viscosity_pa_s
