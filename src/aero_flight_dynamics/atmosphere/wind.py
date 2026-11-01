"""
Atmospheric wind profile, boundary layer shear, and discrete gust models.
Compliant with MIL-F-8785C aerospace atmospheric disturbance standards.
"""

import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class WindModel:
    """
    Parametric wind shear and discrete gust model.
    """
    surface_wind_speed_m_s: float = 3.0   # Wind speed at reference altitude (m/s)
    reference_altitude_m: float = 10.0    # Reference anemometer altitude (m)
    shear_exponent: float = 0.14          # Atmospheric boundary layer power-law exponent
    wind_azimuth_deg: float = 270.0       # Wind direction from which wind blows (degrees)
    gust_amplitude_m_s: float = 5.0       # Peak gust velocity amplitude (m/s)
    gust_altitude_start_m: float = 300.0  # Altitude where discrete gust initiates (m)
    gust_length_m: float = 200.0          # Spatial wavelength of "1 - cosine" gust (m)

    def mean_wind_speed(self, altitude_m: float) -> float:
        """
        Calculates mean horizontal wind speed using power-law boundary layer shear:
        v(z) = v_ref * (z / z_ref)^alpha, capped at jet stream boundary.
        """
        alt = max(1.0, altitude_m)
        if alt > 15000.0:
            # Jet stream taper into stratosphere
            return self.surface_wind_speed_m_s * ((15000.0 / self.reference_altitude_m) ** self.shear_exponent)
        return self.surface_wind_speed_m_s * ((alt / self.reference_altitude_m) ** self.shear_exponent)

    def discrete_gust_speed(self, altitude_m: float) -> float:
        """
        Calculates standard FAA '1 - cosine' discrete gust component:
        v_gust = (v_m / 2) * (1 - cos(2 * pi * (z - z0) / L_g)).
        """
        if self.gust_altitude_start_m <= altitude_m <= (self.gust_altitude_start_m + self.gust_length_m):
            dz = altitude_m - self.gust_altitude_start_m
            return 0.5 * self.gust_amplitude_m_s * (1.0 - math.cos(2.0 * math.pi * dz / self.gust_length_m))
        return 0.0

    def wind_velocity_vector(self, altitude_m: float) -> Tuple[float, float, float]:
        """
        Calculates total 3D wind velocity vector (wx, wy, wz) in meters/second.
        """
        speed = self.mean_wind_speed(altitude_m) + self.discrete_gust_speed(altitude_m)
        az_rad = math.radians(self.wind_azimuth_deg)

        # Wind blows towards opposite direction (180 deg shifted)
        wx = speed * math.cos(az_rad + math.pi)
        wy = speed * math.sin(az_rad + math.pi)
        wz = 0.0  # Vertical updraft/downdraft baseline

        return wx, wy, wz
