"""
Dual-deployment parachute recovery subsystem for suborbital flight vehicles.
Models high-speed drogue deployment at apogee and low-altitude main chute deployment.
"""

import math
from dataclasses import dataclass, field
from enum import Enum


class ParachuteState(Enum):
    """Deployment lifecycle state of a recovery parachute."""

    STOWED = "stowed"
    REEFED = "reefed"
    FULLY_DEPLOYED = "fully_deployed"


@dataclass
class Parachute:
    """
    Parametric model of a decelerator parachute.
    """

    name: str = "Main Canopy"
    diameter_m: float = 1.8  # Effective canopy diameter
    cd: float = 1.5  # Drag coefficient of parachute canopy (hemispherical ~1.5)
    deployment_altitude_m: float = 400.0  # Altitude threshold for deployment (descent phase)
    inflation_time_s: float = 1.2  # Time taken for canopy to reach full inflation
    state: ParachuteState = ParachuteState.STOWED

    @property
    def reference_area_m2(self) -> float:
        """Cross-sectional projected canopy area S = pi * (d / 2)^2."""
        return math.pi * ((self.diameter_m / 2.0) ** 2)

    @property
    def drag_area_cd_s(self) -> float:
        """Effective drag area = CD * S."""
        return self.cd * self.reference_area_m2

    def current_drag_area(self, deployment_timer_s: float) -> float:
        """
        Calculates effective drag area accounting for inflation progression.
        """
        if self.state == ParachuteState.STOWED:
            return 0.0

        if deployment_timer_s >= self.inflation_time_s:
            return self.drag_area_cd_s

        # Quadratic inflation progression
        fraction = min(1.0, max(0.0, deployment_timer_s / self.inflation_time_s))
        return self.drag_area_cd_s * (fraction**2)


def default_drogue() -> Parachute:
    return Parachute(
        name="Drogue Chute", diameter_m=0.45, cd=0.97, deployment_altitude_m=float("inf"), inflation_time_s=0.5
    )


def default_main() -> Parachute:
    return Parachute(name="Main Chute", diameter_m=3.8, cd=1.55, deployment_altitude_m=450.0, inflation_time_s=1.5)


@dataclass
class DualRecoverySystem:
    """
    Two-stage recovery system:
    1. Drogue chute deploys at apogee to stabilize descent and prevent tumbling.
    2. Main chute deploys at a low altitude (e.g. 400m) to brake down to safe landing speed (< 7 m/s).
    """

    drogue: Parachute = field(default_factory=default_drogue)
    main: Parachute = field(default_factory=default_main)

    def total_drag_area(
        self, altitude_m: float, is_descending: bool, drogue_timer_s: float, main_timer_s: float
    ) -> float:
        """
        Calculates combined recovery drag area based on descent state and altitude.
        """
        total_cd_s = 0.0

        # Drogue activates at apogee when descent begins
        if is_descending:
            if self.drogue.state == ParachuteState.STOWED:
                self.drogue.state = ParachuteState.FULLY_DEPLOYED
            total_cd_s += self.drogue.current_drag_area(drogue_timer_s)

        # Main parachute activates below threshold altitude
        if is_descending and altitude_m <= self.main.deployment_altitude_m:
            if self.main.state == ParachuteState.STOWED:
                self.main.state = ParachuteState.FULLY_DEPLOYED
            total_cd_s += self.main.current_drag_area(main_timer_s)

        return total_cd_s

    def estimate_touchdown_velocity(self, vehicle_mass_kg: float, air_density_kg_m3: float = 1.225) -> float:
        """
        Calculates steady-state terminal velocity at ground touchdown:
        v_terminal = sqrt((2 * m * g) / (rho * CD * S)).
        """
        g = 9.80665
        cd_s_total = self.drogue.drag_area_cd_s + self.main.drag_area_cd_s
        if cd_s_total <= 0:
            return float("inf")
        return math.sqrt((2.0 * vehicle_mass_kg * g) / (air_density_kg_m3 * cd_s_total))
