"""
Example: Suborbital Sounding Rocket Mission Simulation.
Simulates high-altitude atmospheric probe ascending to upper mesosphere.
"""

from aero_flight_dynamics import (
    PointMassVehicle,
    SolidRocketMotor,
    MotorBurnProfile,
    AeroCoefficients,
    FlightSimulator,
    FlightRecorder,
)


def run_mission():
    print("=== Suborbital Sounding Rocket Mission Simulation ===")

    # 1. Define solid propulsion system
    motor = SolidRocketMotor(
        name="Castor-Mini Solid Rocket Motor",
        dry_mass_kg=25.0,
        propellant_mass_kg=75.0,
        burn_time_s=8.5,
        isp_sea_level_s=240.0,
        isp_vacuum_s=270.0,
        profile=MotorBurnProfile.NEUTRAL,
    )

    # 2. Configure aerospace vehicle airframe
    vehicle = PointMassVehicle(
        name="AeroProbe-1",
        structural_mass_kg=35.0,  # Payload and avionics
        reference_area_m2=0.045,  # 24 cm diameter fuselage
        characteristic_length_m=3.5,
        aerodynamics=AeroCoefficients(
            cd0=0.22,
            aspect_ratio=5.0,
            oswald_efficiency=0.88,
            mach_critical=0.78,
        ),
        motor=motor,
    )

    print(f"Vehicle: {vehicle.name}")
    print(f"Initial Gross Liftoff Weight (GLOW): {vehicle.total_mass(0.0):.1f} kg")
    print(f"Burnout Mass: {vehicle.total_mass(10.0):.1f} kg")

    # 3. Execute 3-DoF trajectory simulation
    simulator = FlightSimulator(vehicle=vehicle)
    result = simulator.simulate(
        launch_elevation_deg=86.0,  # Near-vertical launch
        launch_azimuth_deg=90.0,
        dt=0.05,
    )

    # 4. Generate flight telemetry report
    recorder = FlightRecorder(result)
    summary = recorder.summary()

    print("\n--- Mission Results ---")
    print(f"Apogee Reached : {summary.apogee_altitude_km} km ({summary.apogee_altitude_km * 1000:,.0f} m)")
    print(f"Time to Apogee : {summary.apogee_time_s} s")
    print(f"Top Speed      : Mach {summary.max_mach} ({summary.burnout_velocity_m_s} m/s)")
    print(f"Max-Q Load     : {summary.max_dynamic_pressure_kpa} kPa")
    print(f"Peak G-Force   : {summary.max_g_load} G")

    # Export telemetry files
    recorder.export_csv("telemetry_mission.csv")
    recorder.export_json("telemetry_mission.json")
    recorder.plot_trajectory("flight_mission_plot.png")
    print("\nTelemetry exported: telemetry_mission.csv, telemetry_mission.json, flight_mission_plot.png")


if __name__ == "__main__":
    run_mission()
