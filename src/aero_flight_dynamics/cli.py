"""
Command Line Interface (CLI) for aero_flight_dynamics.
"""

import argparse
import sys

from aero_flight_dynamics.aerodynamics.airfoil import NACA4Airfoil
from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.propulsion.solid_motor import MotorBurnProfile, SolidRocketMotor
from aero_flight_dynamics.telemetry.recorder import FlightRecorder
from aero_flight_dynamics.trajectory.simulation import FlightSimulator
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle


def handle_atmosphere(args: argparse.Namespace) -> None:
    atm = USStandardAtmosphere1976()
    state = atm.compute(args.altitude)
    print("=" * 60)
    print(f"  U.S. STANDARD ATMOSPHERE (1976) AT {args.altitude:,.1f} METERS")
    print("=" * 60)
    print(f"  Geopotential Alt : {state.geopotential_altitude_m:,.1f} m")
    print(f"  Temperature      : {state.temperature_k:.2f} K ({state.temperature_k - 273.15:.2f} deg C)")
    print(f"  Pressure         : {state.pressure_pa:,.2f} Pa ({state.pressure_pa / 100.0:.2f} hPa)")
    print(f"  Density          : {state.density_kg_m3:.5f} kg/m^3")
    print(f"  Speed of Sound   : {state.speed_of_sound_m_s:.2f} m/s")
    print(f"  Dynamic Visc.    : {state.dynamic_viscosity_pa_s:.4e} Pa*s")
    print("=" * 60)


def handle_airfoil(args: argparse.Namespace) -> None:
    airfoil = NACA4Airfoil(args.designation, chord_m=args.chord)
    cl = airfoil.theoretical_lift_coefficient(args.alpha * (3.1415926535 / 180.0))
    alpha0_deg = airfoil.zero_lift_angle_rad() * (180.0 / 3.1415926535)
    print("=" * 60)
    print(f"  NACA {args.designation} PARAMETRIC ANALYSIS")
    print("=" * 60)
    print(f"  Chord Length     : {airfoil.chord_m:.2f} m")
    print(f"  Max Camber       : {airfoil.max_camber_m * 100.0:.1f}% chord")
    print(f"  Max Thickness    : {airfoil.thickness_ratio_t * 100.0:.1f}% chord")
    print(f"  Zero-Lift Angle  : {alpha0_deg:.2f} deg")
    print(f"  CL at {args.alpha:.1f} deg    : {cl:.4f}")
    print("=" * 60)


def handle_simulate(args: argparse.Namespace) -> None:
    # Build motor
    motor = SolidRocketMotor(
        name="Composite Solid Propellant Motor",
        dry_mass_kg=args.dry_mass * 0.25,
        propellant_mass_kg=args.propellant_mass,
        burn_time_s=args.burn_time,
        isp_sea_level_s=args.isp,
        isp_vacuum_s=args.isp * 1.12,
        profile=MotorBurnProfile.NEUTRAL,
    )

    # Build vehicle
    vehicle = PointMassVehicle(
        name=args.name,
        structural_mass_kg=args.dry_mass,
        reference_area_m2=args.area,
        characteristic_length_m=args.length,
        aerodynamics=AeroCoefficients(cd0=args.cd0),
        motor=motor,
    )

    if hasattr(sys.stdout, "reconfigure"):
        try:
            sys.stdout.reconfigure(encoding="utf-8")
        except Exception:
            pass

    print(f"[*] Initializing trajectory simulation for '{args.name}'...")
    sim = FlightSimulator(vehicle=vehicle)
    result = sim.simulate(
        launch_elevation_deg=args.elevation,
        launch_azimuth_deg=90.0,
        dt=0.05,
    )

    recorder = FlightRecorder(result)
    print(recorder.generate_markdown_report())

    if args.csv:
        recorder.export_csv(args.csv)
        print(f"  [+] Trajectory CSV exported to: {args.csv}")

    if args.json:
        recorder.export_json(args.json)
        print(f"  [+] Telemetry JSON exported to: {args.json}")

    if args.plot:
        success = recorder.plot_trajectory(args.plot)
        if success:
            print(f"  [+] Trajectory flight chart saved to: {args.plot}")


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="aeroflight",
        description="aero_flight_dynamics: Open-source aerospace flight dynamics & telemetry toolkit."
    )
    subparsers = parser.add_subparsers(dest="command", required=True)

    # Subcommand: atmosphere
    p_atm = subparsers.add_parser("atmosphere", help="Query U.S. Standard Atmosphere (1976) properties.")
    p_atm.add_argument("-z", "--altitude", type=float, default=0.0, help="Geometric altitude in meters (default: 0.0)")

    # Subcommand: airfoil
    p_foil = subparsers.add_parser("airfoil", help="Analyze NACA 4-digit airfoil geometry.")
    p_foil.add_argument("-d", "--designation", type=str, default="2412", help="NACA 4-digit code (e.g. 0012, 2412)")
    p_foil.add_argument("-c", "--chord", type=float, default=1.0, help="Chord length in meters")
    p_foil.add_argument("-a", "--alpha", type=float, default=4.0, help="Angle of attack in degrees")

    # Subcommand: simulate
    p_sim = subparsers.add_parser("simulate", help="Run 3-DoF rocket/vehicle trajectory simulation.")
    p_sim.add_argument("--name", type=str, default="AeroProbe-1", help="Vehicle mission name")
    p_sim.add_argument("--dry-mass", type=float, default=40.0, help="Dry mass in kg (default: 40.0)")
    p_sim.add_argument("--propellant-mass", type=float, default=60.0, help="Solid propellant mass in kg (default: 60.0)")
    p_sim.add_argument("--burn-time", type=float, default=8.0, help="Burn duration in seconds (default: 8.0)")
    p_sim.add_argument("--isp", type=float, default=230.0, help="Sea-level specific impulse in seconds")
    p_sim.add_argument("--area", type=float, default=0.045, help="Aerodynamic reference area S in m^2")
    p_sim.add_argument("--length", type=float, default=3.2, help="Vehicle characteristic length in meters")
    p_sim.add_argument("--cd0", type=float, default=0.28, help="Zero-lift drag coefficient CD0")
    p_sim.add_argument("--elevation", type=float, default=85.0, help="Launch rail elevation in degrees")
    p_sim.add_argument("--csv", type=str, default=None, help="Output path for trajectory CSV export")
    p_sim.add_argument("--json", type=str, default=None, help="Output path for telemetry JSON export")
    p_sim.add_argument("--plot", type=str, default=None, help="Output path for trajectory plot PNG")

    args = parser.parse_args()

    if args.command == "atmosphere":
        handle_atmosphere(args)
    elif args.command == "airfoil":
        handle_airfoil(args)
    elif args.command == "simulate":
        handle_simulate(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
