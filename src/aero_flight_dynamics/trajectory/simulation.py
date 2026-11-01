"""
Flight dynamics numerical simulation engine using Runge-Kutta 4th order (RK4) integration.
"""

import math
from dataclasses import dataclass, field
from typing import List, Tuple

from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.trajectory.state import FlightState
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle


@dataclass
class SimulationResult:
    """Summary metrics and full trajectory state time history."""
    vehicle_name: str
    history: List[FlightState] = field(default_factory=list)
    apogee_m: float = 0.0
    apogee_time_s: float = 0.0
    max_mach: float = 0.0
    max_dynamic_pressure_pa: float = 0.0  # Max-Q
    max_g_load: float = 0.0
    burnout_time_s: float = 0.0
    burnout_altitude_m: float = 0.0
    burnout_velocity_m_s: float = 0.0
    downrange_distance_m: float = 0.0
    total_flight_time_s: float = 0.0


class FlightSimulator:
    """
    3-Degree-of-Freedom trajectory simulation engine.
    Solves translational equations of motion under gravitational,
    aerodynamic, and propulsive force vectors.
    """

    G0: float = 9.80665
    R_EARTH: float = 6356766.0

    def __init__(
        self,
        vehicle: PointMassVehicle,
        atmosphere: USStandardAtmosphere1976 = None,
    ) -> None:
        self.vehicle = vehicle
        self.atmosphere = atmosphere if atmosphere is not None else USStandardAtmosphere1976()

    def gravity_at_altitude(self, altitude_m: float) -> float:
        """Inverse-square gravitational acceleration at altitude z."""
        if altitude_m <= -self.R_EARTH:
            return self.G0
        return self.G0 * ((self.R_EARTH / (self.R_EARTH + altitude_m)) ** 2)

    def _derivatives(
        self,
        t: float,
        pos_vel: Tuple[float, float, float, float, float, float],
        pitch_angle_rad: float,
        angle_of_attack_rad: float = 0.0,
    ) -> Tuple[Tuple[float, float, float, float, float, float], dict]:
        """
        Evaluates state derivatives d/dt [x, y, z, vx, vy, vz].
        Returns ((dx, dy, dz, dvx, dvy, dvz), diagnostics_dict).
        """
        x, y, z, vx, vy, vz = pos_vel
        speed = math.sqrt(vx ** 2 + vy ** 2 + vz ** 2)

        # Atmosphere
        atm = self.atmosphere.compute(max(0.0, z))
        mach = atm.mach_number(speed)
        q = atm.dynamic_pressure(speed)

        # Vehicle mass
        mass = self.vehicle.total_mass(t)

        # Propulsion thrust
        thrust_mag = 0.0
        if self.vehicle.motor is not None:
            thrust_mag = self.vehicle.motor.thrust(t, atm.pressure_pa)

        # Aerodynamic forces
        cd = self.vehicle.aerodynamics.drag_coefficient(angle_of_attack_rad, mach)
        cl = self.vehicle.aerodynamics.lift_coefficient(angle_of_attack_rad, mach)

        drag_mag = q * self.vehicle.reference_area_m2 * cd
        lift_mag = q * self.vehicle.reference_area_m2 * cl

        # Thrust direction (aligned with vehicle body pitch along X-Z plane)
        thrust_x = thrust_mag * math.cos(pitch_angle_rad)
        thrust_y = 0.0
        thrust_z = thrust_mag * math.sin(pitch_angle_rad)

        # Drag acts opposite to velocity vector
        if speed > 1e-6:
            drag_x = -drag_mag * (vx / speed)
            drag_y = -drag_mag * (vy / speed)
            drag_z = -drag_mag * (vz / speed)
        else:
            drag_x = 0.0
            drag_y = 0.0
            drag_z = 0.0

        # Lift acts normal to velocity in vertical plane
        gamma = math.atan2(vz, max(1e-6, math.sqrt(vx ** 2 + vy ** 2)))
        lift_x = -lift_mag * math.sin(gamma) * (vx / max(1e-6, math.sqrt(vx ** 2 + vy ** 2)))
        lift_y = 0.0
        lift_z = lift_mag * math.cos(gamma)

        # Gravity acts downward along -Z
        g = self.gravity_at_altitude(z)

        # Newton's Second Law: F = m * a -> a = F / m
        ax = (thrust_x + drag_x + lift_x) / mass
        ay = (thrust_y + drag_y + lift_y) / mass
        az = (thrust_z + drag_z + lift_z) / mass - g

        diagnostics = {
            "mach": mach,
            "q": q,
            "thrust": thrust_mag,
            "drag": drag_mag,
            "lift": lift_mag,
            "ax": ax,
            "ay": ay,
            "az": az,
            "mass": mass,
        }

        return (vx, vy, vz, ax, ay, az), diagnostics

    def simulate(
        self,
        launch_elevation_deg: float = 85.0,
        launch_azimuth_deg: float = 90.0,
        initial_altitude_m: float = 0.0,
        initial_velocity_m_s: float = 0.0,
        dt: float = 0.05,
        max_time_s: float = 400.0,
    ) -> SimulationResult:
        """
        Executes Runge-Kutta 4 (RK4) integration from launch to ground touchdown or timeout.
        """
        elev_rad = math.radians(launch_elevation_deg)
        az_rad = math.radians(launch_azimuth_deg)

        # Initial conditions
        vx0 = initial_velocity_m_s * math.cos(elev_rad) * math.cos(az_rad)
        vy0 = initial_velocity_m_s * math.cos(elev_rad) * math.sin(az_rad)
        vz0 = initial_velocity_m_s * math.sin(elev_rad)

        state = (0.0, 0.0, initial_altitude_m, vx0, vy0, vz0)
        t = 0.0

        history: List[FlightState] = []
        max_mach = 0.0
        max_q = 0.0
        max_g = 0.0
        apogee_m = initial_altitude_m
        apogee_time_s = 0.0

        burn_time = self.vehicle.motor.burn_time_s if self.vehicle.motor else 0.0
        burnout_recorded = False
        burnout_alt = 0.0
        burnout_vel = 0.0

        pitch_rad = elev_rad

        while t <= max_time_s:
            # Gravity turn guidance: pitch tracks flight path angle after clearing launch rail
            speed_horiz = math.sqrt(state[3] ** 2 + state[4] ** 2)
            if state[2] > 20.0 and speed_horiz > 5.0:
                pitch_rad = math.atan2(state[5], speed_horiz)

            # RK4 Stage 1
            k1, diag1 = self._derivatives(t, state, pitch_rad)

            # RK4 Stage 2
            s2 = tuple(state[i] + 0.5 * dt * k1[i] for i in range(6))
            k2, _ = self._derivatives(t + 0.5 * dt, s2, pitch_rad)

            # RK4 Stage 3
            s3 = tuple(state[i] + 0.5 * dt * k2[i] for i in range(6))
            k3, _ = self._derivatives(t + 0.5 * dt, s3, pitch_rad)

            # RK4 Stage 4
            s4 = tuple(state[i] + dt * k3[i] for i in range(6))
            k4, _ = self._derivatives(t + dt, s4, pitch_rad)

            # Record current step
            fs = FlightState(
                time_s=t,
                x_m=state[0],
                y_m=state[1],
                z_m=state[2],
                vx_m_s=state[3],
                vy_m_s=state[4],
                vz_m_s=state[5],
                mass_kg=diag1["mass"],
                ax_m_s2=diag1["ax"],
                ay_m_s2=diag1["ay"],
                az_m_s2=diag1["az"],
                mach=diag1["mach"],
                dynamic_pressure_pa=diag1["q"],
                thrust_n=diag1["thrust"],
                drag_n=diag1["drag"],
                lift_n=diag1["lift"],
            )
            history.append(fs)

            # Metric tracking
            if fs.z_m > apogee_m:
                apogee_m = fs.z_m
                apogee_time_s = t

            if fs.mach > max_mach:
                max_mach = fs.mach
            if fs.dynamic_pressure_pa > max_q:
                max_q = fs.dynamic_pressure_pa
            if fs.g_load > max_g:
                max_g = fs.g_load

            if not burnout_recorded and t >= burn_time:
                burnout_recorded = True
                burnout_alt = fs.z_m
                burnout_vel = fs.speed_m_s

            # Ground impact check after apogee
            if t > 2.0 and state[2] <= 0.0 and state[5] < 0:
                break

            # Advance RK4 state
            state = tuple(
                state[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i])
                for i in range(6)
            )
            t += dt

        downrange = math.sqrt(history[-1].x_m ** 2 + history[-1].y_m ** 2)

        return SimulationResult(
            vehicle_name=self.vehicle.name,
            history=history,
            apogee_m=apogee_m,
            apogee_time_s=apogee_time_s,
            max_mach=max_mach,
            max_dynamic_pressure_pa=max_q,
            max_g_load=max_g,
            burnout_time_s=burn_time,
            burnout_altitude_m=burnout_alt,
            burnout_velocity_m_s=burnout_vel,
            downrange_distance_m=downrange,
            total_flight_time_s=history[-1].time_s,
        )
