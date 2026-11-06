"""
Multi-stage rocket trajectory dynamics, stage separation, and mass depletion models.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.trajectory.state import FlightState


@dataclass
class RocketStage:
    """Individual stage configuration for a multi-stage rocket vehicle."""
    stage_number: int
    name: str
    dry_mass_kg: float
    propellant_mass_kg: float
    burn_time_s: float
    isp_sea_level_s: float
    isp_vacuum_s: float
    ignition_delay_s: float = 0.0      # Coast time between previous burnout and ignition
    jettison_delay_s: float = 0.5      # Time after burnout before dropping stage dry mass

    P_SEA_LEVEL: float = 101325.0

    @property
    def total_initial_mass_kg(self) -> float:
        return self.dry_mass_kg + self.propellant_mass_kg

    def effective_isp(self, ambient_pressure_pa: float) -> float:
        clamped_p = max(0.0, min(self.P_SEA_LEVEL, ambient_pressure_pa))
        ratio = clamped_p / self.P_SEA_LEVEL
        return self.isp_vacuum_s - ratio * (self.isp_vacuum_s - self.isp_sea_level_s)

    def thrust(self, burn_elapsed_s: float, ambient_pressure_pa: float) -> float:
        if burn_elapsed_s < 0.0 or burn_elapsed_s > self.burn_time_s:
            return 0.0
        isp = self.effective_isp(ambient_pressure_pa)
        mdot = self.propellant_mass_kg / self.burn_time_s
        return mdot * 9.80665 * isp


@dataclass
class StagedSimulationResult:
    """Telemetry and staging event record for a multi-stage flight."""
    vehicle_name: str
    apogee_altitude_m: float
    apogee_time_s: float
    max_mach: float
    max_q_pa: float
    staging_events: List[Dict[str, Any]] = field(default_factory=list)
    final_flight_time_s: float = 0.0
    history: List[FlightState] = field(default_factory=list)


class StagedRocketSimulator:
    """
    Simulates multi-stage suborbital rocket flights with discrete stage jettison events.
    """

    G0: float = 9.80665
    R_EARTH: float = 6356766.0

    def __init__(
        self,
        stages: List[RocketStage],
        payload_mass_kg: float = 15.0,
        reference_area_m2: float = 0.04,
        aerodynamics: Optional[AeroCoefficients] = None,
        atmosphere: Optional[USStandardAtmosphere1976] = None,
    ) -> None:
        if not stages:
            raise ValueError("Vehicle must have at least one stage.")
        self.stages = sorted(stages, key=lambda s: s.stage_number)
        self.payload_mass_kg = payload_mass_kg
        self.reference_area_m2 = reference_area_m2
        self.aerodynamics = aerodynamics or AeroCoefficients()
        self.atmosphere = atmosphere or USStandardAtmosphere1976()

    def simulate(
        self,
        launch_elevation_deg: float = 85.0,
        dt: float = 0.05,
        max_time_s: float = 300.0,
    ) -> StagedSimulationResult:
        elev_rad = math.radians(launch_elevation_deg)
        x, z = 0.0, 0.0
        vx = 0.0
        vz = 0.0

        t = 0.0
        pitch_rad = elev_rad
        events: List[Dict[str, Any]] = []

        # Staging timeline tracking
        current_stage_idx = 0
        stage_start_time = 0.0
        stage_state = "ignited"  # "ignited", "burnout_coast", "jettisoned"
        active_stages = list(self.stages)

        max_mach = 0.0
        max_q = 0.0
        apogee_m = 0.0
        apogee_time_s = 0.0

        events.append({
            "time_s": 0.0,
            "event": f"Ignition of Stage 1 ({self.stages[0].name})",
            "altitude_m": 0.0,
        })

        while t <= max_time_s:
            # Check staging logic
            thrust_total = 0.0
            atm = self.atmosphere.compute(max(0.0, z))

            if current_stage_idx < len(self.stages):
                curr_stage = self.stages[current_stage_idx]
                burn_elapsed = t - stage_start_time

                if burn_elapsed <= curr_stage.burn_time_s:
                    thrust_total = curr_stage.thrust(burn_elapsed, atm.pressure_pa)
                elif stage_state == "ignited":
                    stage_state = "burnout_coast"
                    events.append({
                        "time_s": round(t, 2),
                        "event": f"Burnout of Stage {curr_stage.stage_number} ({curr_stage.name})",
                        "altitude_m": round(z, 1),
                        "velocity_m_s": round(math.sqrt(vx**2 + vz**2), 1),
                    })

                # Check jettison & ignition of next stage
                if burn_elapsed >= (curr_stage.burn_time_s + curr_stage.jettison_delay_s):
                    if curr_stage in active_stages:
                        active_stages.remove(curr_stage)
                        events.append({
                            "time_s": round(t, 2),
                            "event": f"Jettison of Stage {curr_stage.stage_number} casing",
                            "altitude_m": round(z, 1),
                        })

                    # Ignite next stage if delay met
                    next_idx = current_stage_idx + 1
                    if next_idx < len(self.stages):
                        next_stage = self.stages[next_idx]
                        if burn_elapsed >= (curr_stage.burn_time_s + next_stage.ignition_delay_s):
                            current_stage_idx = next_idx
                            stage_start_time = t
                            stage_state = "ignited"
                            events.append({
                                "time_s": round(t, 2),
                                "event": f"Ignition of Stage {next_stage.stage_number} ({next_stage.name})",
                                "altitude_m": round(z, 1),
                                "velocity_m_s": round(math.sqrt(vx**2 + vz**2), 1),
                            })

            # Calculate vehicle total mass
            m_total = self.payload_mass_kg
            for s in active_stages:
                if s == self.stages[current_stage_idx] and (t - stage_start_time) < s.burn_time_s:
                    prop_left = s.propellant_mass_kg * (1.0 - (t - stage_start_time) / s.burn_time_s)
                    m_total += s.dry_mass_kg + prop_left
                elif s == self.stages[current_stage_idx]:
                    m_total += s.dry_mass_kg
                else:
                    m_total += s.total_initial_mass_kg

            speed = math.sqrt(vx**2 + vz**2)
            mach = atm.mach_number(speed)
            q = atm.dynamic_pressure(speed)

            max_mach = max(max_mach, mach)
            max_q = max(max_q, q)
            if z > apogee_m:
                apogee_m = z
                apogee_time_s = t

            # Forces
            cd = self.aerodynamics.drag_coefficient(0.0, mach)
            drag = q * self.reference_area_m2 * cd

            # Gravity turn guidance
            if z > 50.0 and speed > 10.0:
                pitch_rad = math.atan2(vz, max(1.0, vx))

            thrust_x = thrust_total * math.cos(pitch_rad)
            thrust_z = thrust_total * math.sin(pitch_rad)

            drag_x = -drag * (vx / speed) if speed > 1e-4 else 0.0
            drag_z = -drag * (vz / speed) if speed > 1e-4 else 0.0

            g = self.G0 * ((self.R_EARTH / (self.R_EARTH + max(0.0, z))) ** 2)
            ax = (thrust_x + drag_x) / m_total
            az = (thrust_z + drag_z) / m_total - g

            # Euler-Heun forward step
            vx += ax * dt
            vz += az * dt
            x += vx * dt
            z += vz * dt
            t += dt

            # Ground impact after apogee
            if z < 0.0 and t > 5.0:
                z = 0.0
                break

        return StagedSimulationResult(
            vehicle_name="2-Stage Suborbital Explorer",
            apogee_altitude_m=round(apogee_m, 1),
            apogee_time_s=round(apogee_time_s, 2),
            max_mach=round(max_mach, 2),
            max_q_pa=round(max_q, 1),
            staging_events=events,
            final_flight_time_s=round(t, 2),
        )

