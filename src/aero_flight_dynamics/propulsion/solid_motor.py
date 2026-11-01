"""
Rocket propulsion systems and motor burn profiles.
"""

import math
from enum import Enum
from dataclasses import dataclass


class MotorBurnProfile(Enum):
    """Grain burning profiles for solid rocket motors."""
    NEUTRAL = "neutral"          # Constant thrust
    PROGRESSIVE = "progressive"  # Thrust increases with burn time
    REGRESSIVE = "regressive"    # Thrust decays with burn time
    BOOST_SUSTAIN = "boost_sustain"  # High initial thrust followed by lower sustain


@dataclass
class SolidRocketMotor:
    """
    Solid rocket motor model with time-dependent mass depletion and
    ambient pressure altitude compensation on specific impulse (Isp).
    """
    name: str = "Standard SRM"
    dry_mass_kg: float = 15.0            # Motor casing, nozzle, hardware mass
    propellant_mass_kg: float = 45.0     # Active solid propellant mass
    burn_time_s: float = 6.0             # Total burn duration
    isp_sea_level_s: float = 220.0       # Sea level specific impulse (seconds)
    isp_vacuum_s: float = 250.0          # Vacuum specific impulse (seconds)
    profile: MotorBurnProfile = MotorBurnProfile.NEUTRAL

    # Constants
    G0: float = 9.80665                  # Gravitational acceleration (m/s^2)
    P_SEA_LEVEL: float = 101325.0        # Reference sea-level atmospheric pressure (Pa)

    @property
    def total_initial_mass_kg(self) -> float:
        """Total motor mass before ignition."""
        return self.dry_mass_kg + self.propellant_mass_kg

    @property
    def average_mass_flow_rate_kg_s(self) -> float:
        """Average mass flow rate = propellant_mass / burn_time."""
        if self.burn_time_s <= 0:
            return 0.0
        return self.propellant_mass_kg / self.burn_time_s

    def effective_isp(self, ambient_pressure_pa: float) -> float:
        """
        Compute pressure-compensated Isp between sea-level and vacuum.
        Linear nozzle expansion interpolation.
        """
        clamped_p = max(0.0, min(self.P_SEA_LEVEL, ambient_pressure_pa))
        pressure_ratio = clamped_p / self.P_SEA_LEVEL
        # At P=0 (vacuum), isp = isp_vacuum; at P=101325 (sea level), isp = isp_sea_level
        return self.isp_vacuum_s - pressure_ratio * (self.isp_vacuum_s - self.isp_sea_level_s)

    def current_propellant_mass(self, time_s: float) -> float:
        """Compute remaining propellant mass at simulation time t."""
        if time_s <= 0.0:
            return self.propellant_mass_kg
        if time_s >= self.burn_time_s:
            return 0.0

        fraction = time_s / self.burn_time_s

        if self.profile == MotorBurnProfile.NEUTRAL:
            consumed = self.propellant_mass_kg * fraction
        elif self.profile == MotorBurnProfile.PROGRESSIVE:
            # Quadratic growth in consumption
            consumed = self.propellant_mass_kg * (fraction ** 1.5)
        elif self.profile == MotorBurnProfile.REGRESSIVE:
            # Slower consumption later
            consumed = self.propellant_mass_kg * (1.0 - (1.0 - fraction) ** 1.5)
        elif self.profile == MotorBurnProfile.BOOST_SUSTAIN:
            # 60% consumed in first 30% of burn time
            if fraction <= 0.3:
                consumed = self.propellant_mass_kg * 0.6 * (fraction / 0.3)
            else:
                sustain_frac = (fraction - 0.3) / 0.7
                consumed = self.propellant_mass_kg * (0.6 + 0.4 * sustain_frac)
        else:
            consumed = self.propellant_mass_kg * fraction

        return max(0.0, self.propellant_mass_kg - consumed)

    def current_motor_mass(self, time_s: float) -> float:
        """Total current motor mass = casing + remaining propellant."""
        return self.dry_mass_kg + self.current_propellant_mass(time_s)

    def mass_flow_rate(self, time_s: float) -> float:
        """
        Instantaneous mass depletion rate dm/dt (kg/s).
        """
        if time_s < 0.0 or time_s >= self.burn_time_s:
            return 0.0

        # Small numerical delta for accurate derivative across any profile
        dt = 0.001
        t1 = max(0.0, time_s - dt)
        t2 = min(self.burn_time_s, time_s + dt)
        m1 = self.current_propellant_mass(t1)
        m2 = self.current_propellant_mass(t2)
        return max(0.0, (m1 - m2) / (t2 - t1))

    def thrust(self, time_s: float, ambient_pressure_pa: float = 101325.0) -> float:
        """
        Compute instantaneous thrust in Newtons: F_thrust = m_dot * g0 * Isp_eff.
        """
        if time_s < 0.0 or time_s >= self.burn_time_s:
            return 0.0

        m_dot = self.mass_flow_rate(time_s)
        isp = self.effective_isp(ambient_pressure_pa)
        return m_dot * self.G0 * isp

    @property
    def total_impulse_sea_level(self) -> float:
        """Estimated total impulse at sea level (N*s)."""
        return self.propellant_mass_kg * self.G0 * self.isp_sea_level_s
