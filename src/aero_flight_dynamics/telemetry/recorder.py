"""
Telemetry recording, analysis, CSV/JSON export, and flight plotting.
"""

import csv
import json
from dataclasses import asdict, dataclass

from aero_flight_dynamics.trajectory.simulation import SimulationResult


@dataclass
class TelemetrySummary:
    """High-level performance summary of a flight test mission."""

    vehicle_name: str
    apogee_altitude_km: float
    apogee_time_s: float
    max_mach: float
    max_dynamic_pressure_kpa: float
    max_g_load: float
    burnout_altitude_km: float
    burnout_velocity_m_s: float
    downrange_distance_km: float
    total_flight_time_s: float


class FlightRecorder:
    """
    Records and processes flight telemetry packets from trajectory simulations.
    Supports CSV export, structured JSON export, and automated report generation.
    """

    def __init__(self, result: SimulationResult) -> None:
        self.result = result

    def summary(self) -> TelemetrySummary:
        """Computes summary flight statistics in standard engineering units."""
        return TelemetrySummary(
            vehicle_name=self.result.vehicle_name,
            apogee_altitude_km=round(self.result.apogee_m / 1000.0, 3),
            apogee_time_s=round(self.result.apogee_time_s, 2),
            max_mach=round(self.result.max_mach, 2),
            max_dynamic_pressure_kpa=round(self.result.max_dynamic_pressure_pa / 1000.0, 2),
            max_g_load=round(self.result.max_g_load, 2),
            burnout_altitude_km=round(self.result.burnout_altitude_m / 1000.0, 3),
            burnout_velocity_m_s=round(self.result.burnout_velocity_m_s, 1),
            downrange_distance_km=round(self.result.downrange_distance_m / 1000.0, 3),
            total_flight_time_s=round(self.result.total_flight_time_s, 2),
        )

    def export_csv(self, filepath: str) -> None:
        """Export raw high-rate trajectory time history to CSV."""
        if not self.result.history:
            return

        fieldnames = [
            "time_s",
            "x_m",
            "y_m",
            "z_m",
            "vx_m_s",
            "vy_m_s",
            "vz_m_s",
            "speed_m_s",
            "mach",
            "dynamic_pressure_pa",
            "mass_kg",
            "thrust_n",
            "drag_n",
            "lift_n",
            "ax_m_s2",
            "az_m_s2",
            "g_load",
        ]

        with open(filepath, mode="w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(fieldnames)
            for state in self.result.history:
                writer.writerow(
                    [
                        round(state.time_s, 4),
                        round(state.x_m, 2),
                        round(state.y_m, 2),
                        round(state.z_m, 2),
                        round(state.vx_m_s, 2),
                        round(state.vy_m_s, 2),
                        round(state.vz_m_s, 2),
                        round(state.speed_m_s, 2),
                        round(state.mach, 3),
                        round(state.dynamic_pressure_pa, 1),
                        round(state.mass_kg, 2),
                        round(state.thrust_n, 1),
                        round(state.drag_n, 1),
                        round(state.lift_n, 1),
                        round(state.ax_m_s2, 2),
                        round(state.az_m_s2, 2),
                        round(state.g_load, 2),
                    ]
                )

    def export_json(self, filepath: str) -> None:
        """Export summary and sampled time-series data to JSON."""
        summary_dict = asdict(self.summary())
        # Downsample trajectory points to max 200 points for efficient JSON storage
        step = max(1, len(self.result.history) // 200)
        samples = [
            {
                "t": round(s.time_s, 2),
                "alt_m": round(s.z_m, 1),
                "downrange_m": round(s.x_m, 1),
                "speed_m_s": round(s.speed_m_s, 1),
                "mach": round(s.mach, 2),
                "q_pa": round(s.dynamic_pressure_pa, 1),
            }
            for s in self.result.history[::step]
        ]

        payload = {
            "mission_summary": summary_dict,
            "trajectory_samples": samples,
        }

        with open(filepath, mode="w", encoding="utf-8") as f:
            json.dump(payload, f, indent=2)

    def generate_markdown_report(self) -> str:
        """Generate a formatted markdown telemetry assessment report."""
        s = self.summary()
        report = f"""# Flight Telemetry Assessment Report: {s.vehicle_name}

| Mission Parameter | Value | Standard Unit |
| :--- | :--- | :--- |
| **Apogee Altitude** | **{s.apogee_altitude_km}** | km ({round(s.apogee_altitude_km * 3280.84, 1)} ft) |
| **Apogee Time** | {s.apogee_time_s} | seconds |
| **Maximum Mach Number** | **Mach {s.max_mach}** | - |
| **Max Dynamic Pressure (Max-Q)** | {s.max_dynamic_pressure_kpa} | kPa |
| **Peak Axial Load** | {s.max_g_load} | G |
| **Burnout Altitude** | {s.burnout_altitude_km} | km |
| **Burnout Velocity** | {s.burnout_velocity_m_s} | m/s |
| **Downrange Distance** | {s.downrange_distance_km} | km |
| **Total Flight Duration** | {s.total_flight_time_s} | seconds |

*Generated by `aero_flight_dynamics` Open-Source Aerospace Simulation Engine.*
"""
        return report

    def plot_trajectory(self, output_filepath: str) -> bool:
        """
        Generates 4-panel flight dynamics charts using matplotlib.
        Returns True if successful, False if matplotlib unavailable.
        """
        try:
            import matplotlib

            matplotlib.use("Agg")  # Non-interactive backend
            import matplotlib.pyplot as plt

            times = [s.time_s for s in self.result.history]
            altitudes_km = [s.z_m / 1000.0 for s in self.result.history]
            machs = [s.mach for s in self.result.history]
            q_kpa = [s.dynamic_pressure_pa / 1000.0 for s in self.result.history]
            downrange_km = [s.x_m / 1000.0 for s in self.result.history]

            fig, axs = plt.subplots(2, 2, figsize=(12, 8))
            fig.suptitle(f"Flight Dynamics Profile: {self.result.vehicle_name}", fontsize=14, fontweight="bold")

            # 1. Altitude vs Time
            axs[0, 0].plot(times, altitudes_km, color="#0066cc", lw=2)
            axs[0, 0].set_title("Altitude Profile")
            axs[0, 0].set_xlabel("Time (s)")
            axs[0, 0].set_ylabel("Altitude (km)")
            axs[0, 0].grid(True, alpha=0.3)

            # 2. Velocity & Mach vs Time
            axs[0, 1].plot(times, machs, color="#d9534f", lw=2)
            axs[0, 1].set_title("Mach Number Profile")
            axs[0, 1].set_xlabel("Time (s)")
            axs[0, 1].set_ylabel("Mach")
            axs[0, 1].axhline(1.0, color="gray", linestyle="--", label="Mach 1.0 (Transonic)")
            axs[0, 1].legend()
            axs[0, 1].grid(True, alpha=0.3)

            # 3. Dynamic Pressure (Max-Q) vs Time
            axs[1, 0].plot(times, q_kpa, color="#f0ad4e", lw=2)
            axs[1, 0].set_title("Dynamic Pressure (Q)")
            axs[1, 0].set_xlabel("Time (s)")
            axs[1, 0].set_ylabel("Dynamic Pressure (kPa)")
            axs[1, 0].grid(True, alpha=0.3)

            # 4. Trajectory (Altitude vs Downrange)
            axs[1, 1].plot(downrange_km, altitudes_km, color="#5cb85c", lw=2)
            axs[1, 1].set_title("Trajectory Profile (X-Z)")
            axs[1, 1].set_xlabel("Downrange (km)")
            axs[1, 1].set_ylabel("Altitude (km)")
            axs[1, 1].grid(True, alpha=0.3)

            plt.tight_layout()
            plt.savefig(output_filepath, dpi=200)
            plt.close()
            return True
        except ImportError:
            return False
