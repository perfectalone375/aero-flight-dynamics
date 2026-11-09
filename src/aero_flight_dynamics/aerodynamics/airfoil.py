"""
NACA 4-digit airfoil geometry and aerodynamic characteristics generator.
"""

import math
from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class AirfoilGeometry:
    """Stores coordinate distribution of an airfoil."""

    x: List[float]
    y_upper: List[float]
    y_lower: List[float]
    camber_line: List[float]
    chord_m: float


class NACA4Airfoil:
    """
    Parametric NACA 4-digit airfoil analysis (e.g. NACA 0012, 2412, 4415).
    """

    def __init__(self, designation: str, chord_m: float = 1.0) -> None:
        """
        Args:
            designation: 4-digit NACA string, e.g. "2412"
            chord_m: Chord length in meters
        """
        if len(designation) != 4 or not designation.isdigit():
            raise ValueError(f"Invalid NACA 4-digit designation: {designation}")

        self.designation = designation
        self.chord_m = chord_m

        # Parse digits
        # 1st digit: Max camber as percentage of chord
        self.max_camber_m = int(designation[0]) / 100.0
        # 2nd digit: Distance of max camber from leading edge in tenths of chord
        self.max_camber_pos_p = int(designation[1]) / 10.0
        # 3rd & 4th digits: Max thickness as percentage of chord
        self.thickness_ratio_t = int(designation[2:]) / 100.0

    def thickness_distribution(self, x_c: float) -> float:
        """
        Calculate half-thickness y_t / c at normalized station x/c.
        Standard NASA formulation.
        """
        x_c = max(0.0, min(1.0, x_c))
        a0 = 0.2969
        a1 = -0.1260
        a2 = -0.3516
        a3 = 0.2843
        a4 = -0.1015  # Closed trailing edge formulation -0.1036 or standard -0.1015

        yt_c = (
            5.0
            * self.thickness_ratio_t
            * (a0 * math.sqrt(x_c) + a1 * x_c + a2 * (x_c**2) + a3 * (x_c**3) + a4 * (x_c**4))
        )
        return yt_c

    def camber_and_slope(self, x_c: float) -> Tuple[float, float]:
        """
        Calculate mean camber line y_c / c and local slope dy_c / dx at station x/c.
        """
        x_c = max(0.0, min(1.0, x_c))
        m = self.max_camber_m
        p = self.max_camber_pos_p

        if m == 0.0 or p == 0.0:
            # Symmetric airfoil
            return 0.0, 0.0

        if x_c < p:
            yc_c = (m / (p**2)) * (2 * p * x_c - (x_c**2))
            dyc_dx = (2 * m / (p**2)) * (p - x_c)
        else:
            yc_c = (m / ((1.0 - p) ** 2)) * ((1.0 - 2 * p) + 2 * p * x_c - (x_c**2))
            dyc_dx = (2 * m / ((1.0 - p) ** 2)) * (p - x_c)

        return yc_c, dyc_dx

    def generate_coordinates(self, num_points: int = 100) -> AirfoilGeometry:
        """Generate upper and lower surface coordinates using cosine spacing."""
        x_coords = []
        y_upper = []
        y_lower = []
        camber = []

        for i in range(num_points + 1):
            beta = math.pi * i / num_points
            x_c = 0.5 * (1.0 - math.cos(beta))  # Cosine clustering at LE and TE
            yc_c, dyc_dx = self.camber_and_slope(x_c)
            theta = math.atan(dyc_dx)
            yt_c = self.thickness_distribution(x_c)

            yu = (yc_c + yt_c * math.cos(theta)) * self.chord_m
            yl = (yc_c - yt_c * math.cos(theta)) * self.chord_m

            x_coords.append(x_c * self.chord_m)
            y_upper.append(yu)
            y_lower.append(yl)
            camber.append(yc_c * self.chord_m)

        return AirfoilGeometry(
            x=x_coords,
            y_upper=y_upper,
            y_lower=y_lower,
            camber_line=camber,
            chord_m=self.chord_m,
        )

    def zero_lift_angle_rad(self) -> float:
        """
        Estimate zero-lift angle of attack alpha_0 (radians) using thin-airfoil theory.
        """
        if self.max_camber_m == 0.0:
            return 0.0
        # Linear approximation for 4-digit series: alpha_0 ~ -0.11 * (max_camber_percent) rad
        return -0.11 * (self.max_camber_m * 100.0) * (math.pi / 180.0)

    def theoretical_lift_coefficient(self, alpha_rad: float) -> float:
        """
        Thin-airfoil theory lift coefficient: Cl = 2*pi * (alpha - alpha_0)
        """
        alpha_0 = self.zero_lift_angle_rad()
        return 2.0 * math.pi * (alpha_rad - alpha_0)
