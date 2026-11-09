"""
Implementation of the 1976 U.S. Standard Atmosphere model.
Accurate from sea level up to 86,000 meters geopotential altitude,
with smooth asymptotic degradation for space/vacuum altitudes.
"""

import math
from typing import List, Tuple

from aero_flight_dynamics.atmosphere.models import AtmosphericState


class USStandardAtmosphere1976:
    """
    U.S. Standard Atmosphere (1976) calculator.
    Provides thermodynamic and transport properties of dry air.
    """

    # Physical constants
    R_EARTH: float = 6356766.0  # Earth radius in meters
    G0: float = 9.80665  # Standard gravity at sea level (m/s^2)
    R_SPECIFIC_AIR: float = 287.05287  # Specific gas constant for air (J/(kg*K))
    GAMMA: float = 1.4  # Ratio of specific heats
    SUTHERLAND_S: float = 110.4  # Sutherland constant (K)
    SUTHERLAND_BETA: float = 1.458e-6  # Sutherland coefficient (kg/(m*s*K^0.5))

    # Sea-level reference parameters
    T0: float = 288.15  # Sea level temperature (K)
    P0: float = 101325.0  # Sea level pressure (Pa)
    RHO0: float = 1.2250  # Sea level density (kg/m^3)

    # Defined geopotential altitude boundaries (meters) and lapse rates (K/m)
    # [(H_base, lapse_rate)]
    LAYERS: List[Tuple[float, float]] = [
        (0.0, -0.0065),  # Troposphere
        (11000.0, 0.0),  # Tropopause
        (20000.0, 0.0010),  # Stratosphere layer 1
        (32000.0, 0.0028),  # Stratosphere layer 2
        (47000.0, 0.0),  # Stratopause
        (51000.0, -0.0028),  # Mesosphere layer 1
        (71000.0, -0.0020),  # Mesosphere layer 2
        (84852.0, 0.0),  # Mesopause boundary
    ]

    def __init__(self) -> None:
        # Precompute base temperatures and pressures at each boundary layer
        self._layer_t_base: List[float] = []
        self._layer_p_base: List[float] = []

        current_t = self.T0
        current_p = self.P0

        for i, (h_base, lapse) in enumerate(self.LAYERS):
            self._layer_t_base.append(current_t)
            self._layer_p_base.append(current_p)

            if i < len(self.LAYERS) - 1:
                next_h = self.LAYERS[i + 1][0]
                dh = next_h - h_base
                if abs(lapse) < 1e-12:
                    current_p = current_p * math.exp(-self.G0 * dh / (self.R_SPECIFIC_AIR * current_t))
                else:
                    t_next = current_t + lapse * dh
                    current_p = current_p * ((t_next / current_t) ** (-self.G0 / (self.R_SPECIFIC_AIR * lapse)))
                    current_t = t_next

    def geometric_to_geopotential(self, geometric_alt_m: float) -> float:
        """Convert geometric altitude (z) to geopotential altitude (H)."""
        if geometric_alt_m <= -self.R_EARTH:
            return 0.0
        return (self.R_EARTH * geometric_alt_m) / (self.R_EARTH + geometric_alt_m)

    def geopotential_to_geometric(self, geopotential_alt_m: float) -> float:
        """Convert geopotential altitude (H) to geometric altitude (z)."""
        return (self.R_EARTH * geopotential_alt_m) / (self.R_EARTH - geopotential_alt_m)

    def compute(self, altitude_m: float) -> AtmosphericState:
        """
        Compute atmospheric state for a given geometric altitude in meters.
        """
        # Clamp negative altitudes safely to sea-level or zero
        if altitude_m < 0.0:
            altitude_m = 0.0

        # Calculate geopotential altitude
        h_geopotential = self.geometric_to_geopotential(altitude_m)

        # Above 86 km (upper mesosphere / thermosphere / exosphere)
        max_layer_h = self.LAYERS[-1][0]
        if h_geopotential >= max_layer_h:
            t = self._layer_t_base[-1]
            dh = h_geopotential - max_layer_h
            # Exponential decay into near-vacuum
            p = self._layer_p_base[-1] * math.exp(-self.G0 * dh / (self.R_SPECIFIC_AIR * t))
            rho = p / (self.R_SPECIFIC_AIR * t)
            a = math.sqrt(self.GAMMA * self.R_SPECIFIC_AIR * t)
            mu = (self.SUTHERLAND_BETA * (t**1.5)) / (t + self.SUTHERLAND_S)
            return AtmosphericState(
                altitude_m=altitude_m,
                geopotential_altitude_m=h_geopotential,
                temperature_k=t,
                pressure_pa=p,
                density_kg_m3=rho,
                speed_of_sound_m_s=a,
                dynamic_viscosity_pa_s=mu,
            )

        # Find layer
        idx = 0
        for i in range(len(self.LAYERS) - 1, -1, -1):
            if h_geopotential >= self.LAYERS[i][0]:
                idx = i
                break

        h_base, lapse = self.LAYERS[idx]
        t_base = self._layer_t_base[idx]
        p_base = self._layer_p_base[idx]
        dh = h_geopotential - h_base

        if abs(lapse) < 1e-12:
            # Isothermal layer
            t = t_base
            p = p_base * math.exp(-self.G0 * dh / (self.R_SPECIFIC_AIR * t_base))
        else:
            # Gradient layer
            t = t_base + lapse * dh
            p = p_base * ((t / t_base) ** (-self.G0 / (self.R_SPECIFIC_AIR * lapse)))

        # Ideal gas law: rho = P / (R * T)
        rho = p / (self.R_SPECIFIC_AIR * t)

        # Speed of sound: a = sqrt(gamma * R * T)
        a = math.sqrt(self.GAMMA * self.R_SPECIFIC_AIR * t)

        # Sutherland's law for dynamic viscosity
        mu = (self.SUTHERLAND_BETA * (t**1.5)) / (t + self.SUTHERLAND_S)

        return AtmosphericState(
            altitude_m=altitude_m,
            geopotential_altitude_m=h_geopotential,
            temperature_k=t,
            pressure_pa=p,
            density_kg_m3=rho,
            speed_of_sound_m_s=a,
            dynamic_viscosity_pa_s=mu,
        )
