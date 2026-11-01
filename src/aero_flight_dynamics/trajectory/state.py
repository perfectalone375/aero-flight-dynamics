"""
Flight state representation and coordinate systems.
"""

import math
from dataclasses import dataclass


@dataclass
class FlightState:
    """
    State vector for a flight vehicle in the Earth-Centered Local Vertical frame.
    x: downrange position (meters)
    y: crossrange position (meters)
    z: altitude above sea level (meters)
    vx, vy, vz: velocity components (m/s)
    """
    time_s: float = 0.0
    x_m: float = 0.0
    y_m: float = 0.0
    z_m: float = 0.0
    vx_m_s: float = 0.0
    vy_m_s: float = 0.0
    vz_m_s: float = 0.0
    mass_kg: float = 100.0

    # Dynamic / diagnostic states (populated during integration)
    ax_m_s2: float = 0.0
    ay_m_s2: float = 0.0
    az_m_s2: float = 0.0
    mach: float = 0.0
    dynamic_pressure_pa: float = 0.0
    thrust_n: float = 0.0
    drag_n: float = 0.0
    lift_n: float = 0.0

    @property
    def speed_m_s(self) -> float:
        """Total scalar velocity magnitude."""
        return math.sqrt(self.vx_m_s ** 2 + self.vy_m_s ** 2 + self.vz_m_s ** 2)

    @property
    def flight_path_angle_rad(self) -> float:
        """Flight path angle gamma = atan2(vz, sqrt(vx^2 + vy^2))."""
        v_horiz = math.sqrt(self.vx_m_s ** 2 + self.vy_m_s ** 2)
        return math.atan2(self.vz_m_s, v_horiz)

    @property
    def total_acceleration_m_s2(self) -> float:
        """Total scalar acceleration magnitude."""
        return math.sqrt(self.ax_m_s2 ** 2 + self.ay_m_s2 ** 2 + self.az_m_s2 ** 2)

    @property
    def g_load(self) -> float:
        """Load factor in standard gees."""
        return self.total_acceleration_m_s2 / 9.80665
