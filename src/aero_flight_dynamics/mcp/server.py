"""
Model Context Protocol (MCP) Server for aero-flight-dynamics.
Exposes computational aerospace tools to AI agents (Claude Desktop, Cursor, Antigravity)
over standard JSON-RPC 2.0 protocol via standard I/O.
"""

import json
import sys
import math
from typing import Any, Dict
from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.aerodynamics.airfoil import NACA4Airfoil
from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.propulsion.solid_motor import SolidRocketMotor, MotorBurnProfile
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle
from aero_flight_dynamics.trajectory.simulation import FlightSimulator
from aero_flight_dynamics.trajectory.staging import RocketStage, StagedRocketSimulator
from aero_flight_dynamics.telemetry.recorder import FlightRecorder
from aero_flight_dynamics.recovery.parachute import DualRecoverySystem, Parachute


TOOLS = [
    {
        "name": "simulate_flight",
        "description": "Simulate a 3-DoF suborbital rocket trajectory using Runge-Kutta 4th order (RK4) numerical integration.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "mission_name": {"type": "string", "description": "Mission or vehicle designation", "default": "AeroProbe"},
                "launch_elevation_deg": {"type": "number", "description": "Launch rail angle in degrees (default: 85.0)", "default": 85.0},
                "dry_mass_kg": {"type": "number", "description": "Vehicle structural/payload dry mass in kg", "default": 35.0},
                "propellant_mass_kg": {"type": "number", "description": "Solid propellant mass in kg", "default": 65.0},
                "burn_time_s": {"type": "number", "description": "Motor burn duration in seconds", "default": 7.5},
                "isp_sea_level_s": {"type": "number", "description": "Specific impulse at sea level (s)", "default": 230.0},
                "cd0": {"type": "number", "description": "Zero-lift parasitic drag coefficient", "default": 0.25}
            }
        }
    },
    {
        "name": "query_atmosphere",
        "description": "Query the 1976 U.S. Standard Atmosphere model for temperature, pressure, density, and sound speed at altitude.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "altitude_m": {"type": "number", "description": "Geometric altitude in meters above sea level"}
            },
            "required": ["altitude_m"]
        }
    },
    {
        "name": "evaluate_airfoil",
        "description": "Perform parametric thin-airfoil theory and geometry analysis for any NACA 4-digit profile.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "designation": {"type": "string", "description": "4-digit NACA designation (e.g. '0012', '2412', '4415')"},
                "alpha_deg": {"type": "number", "description": "Angle of attack in degrees", "default": 4.0},
                "chord_m": {"type": "number", "description": "Chord length in meters", "default": 1.0}
            },
            "required": ["designation"]
        }
    },
    {
        "name": "evaluate_parachute_recovery",
        "description": "Calculate terminal touchdown velocity and drag area for dual-deployment parachute recovery.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "vehicle_mass_kg": {"type": "number", "description": "Landing mass of vehicle in kg"},
                "main_diameter_m": {"type": "number", "description": "Main canopy diameter in meters", "default": 2.0},
                "drogue_diameter_m": {"type": "number", "description": "Drogue canopy diameter in meters", "default": 0.45}
            },
            "required": ["vehicle_mass_kg"]
        }
    },
    {
        "name": "simulate_staged_flight",
        "description": "Simulate a multi-stage suborbital rocket with booster burnout, stage separation/jettison, and sustainer ignition.",
        "inputSchema": {
            "type": "object",
            "properties": {
                "launch_elevation_deg": {"type": "number", "default": 85.0},
                "payload_mass_kg": {"type": "number", "default": 15.0},
                "booster_propellant_kg": {"type": "number", "default": 80.0},
                "booster_burn_s": {"type": "number", "default": 5.0},
                "sustainer_propellant_kg": {"type": "number", "default": 30.0},
                "sustainer_burn_s": {"type": "number", "default": 6.0}
            }
        }
    }
]


def handle_simulate_flight(args: Dict[str, Any]) -> Dict[str, Any]:
    mission_name = args.get("mission_name", "AeroProbe")
    elevation = float(args.get("launch_elevation_deg", 85.0))
    dry_mass = float(args.get("dry_mass_kg", 35.0))
    prop_mass = float(args.get("propellant_mass_kg", 65.0))
    burn_time = float(args.get("burn_time_s", 7.5))
    isp = float(args.get("isp_sea_level_s", 230.0))
    cd0 = float(args.get("cd0", 0.25))

    motor = SolidRocketMotor(
        name="MCP Motor",
        dry_mass_kg=dry_mass * 0.25,
        propellant_mass_kg=prop_mass,
        burn_time_s=burn_time,
        isp_sea_level_s=isp,
        isp_vacuum_s=isp * 1.12,
        profile=MotorBurnProfile.NEUTRAL,
    )

    vehicle = PointMassVehicle(
        name=mission_name,
        structural_mass_kg=dry_mass,
        reference_area_m2=0.038,
        characteristic_length_m=3.0,
        aerodynamics=AeroCoefficients(cd0=cd0),
        motor=motor,
    )

    sim = FlightSimulator(vehicle=vehicle)
    result = sim.simulate(launch_elevation_deg=elevation, dt=0.05)
    recorder = FlightRecorder(result)
    summary = recorder.summary()

    return {
        "status": "success",
        "mission_name": summary.vehicle_name,
        "apogee_altitude_km": summary.apogee_altitude_km,
        "apogee_time_s": summary.apogee_time_s,
        "max_mach": summary.max_mach,
        "max_q_kpa": summary.max_dynamic_pressure_kpa,
        "max_g_load": summary.max_g_load,
        "burnout_velocity_m_s": summary.burnout_velocity_m_s,
        "downrange_km": summary.downrange_distance_km,
        "flight_time_s": summary.total_flight_time_s,
    }


def handle_query_atmosphere(args: Dict[str, Any]) -> Dict[str, Any]:
    alt = float(args.get("altitude_m", 0.0))
    atm = USStandardAtmosphere1976()
    state = atm.compute(alt)

    return {
        "altitude_m": state.altitude_m,
        "geopotential_altitude_m": round(state.geopotential_altitude_m, 1),
        "temperature_k": round(state.temperature_k, 2),
        "temperature_c": round(state.temperature_k - 273.15, 2),
        "pressure_pa": round(state.pressure_pa, 2),
        "density_kg_m3": round(state.density_kg_m3, 5),
        "speed_of_sound_m_s": round(state.speed_of_sound_m_s, 2),
        "dynamic_viscosity_pa_s": state.dynamic_viscosity_pa_s,
    }


def handle_evaluate_airfoil(args: Dict[str, Any]) -> Dict[str, Any]:
    designation = str(args.get("designation", "2412"))
    alpha_deg = float(args.get("alpha_deg", 4.0))
    chord_m = float(args.get("chord_m", 1.0))

    airfoil = NACA4Airfoil(designation, chord_m=chord_m)
    cl = airfoil.theoretical_lift_coefficient(math.radians(alpha_deg))
    alpha0_deg = math.degrees(airfoil.zero_lift_angle_rad())

    return {
        "designation": designation,
        "chord_m": chord_m,
        "max_camber_percent": round(airfoil.max_camber_m * 100.0, 1),
        "max_thickness_percent": round(airfoil.thickness_ratio_t * 100.0, 1),
        "zero_lift_angle_deg": round(alpha0_deg, 2),
        "cl_at_angle": round(cl, 4),
    }


def handle_evaluate_parachute(args: Dict[str, Any]) -> Dict[str, Any]:
    mass = float(args.get("vehicle_mass_kg", 50.0))
    main_d = float(args.get("main_diameter_m", 3.8))
    drogue_d = float(args.get("drogue_diameter_m", 0.45))

    recovery = DualRecoverySystem(
        drogue=Parachute(name="Drogue", diameter_m=drogue_d, cd=0.97),
        main=Parachute(name="Main", diameter_m=main_d, cd=1.55),
    )

    touchdown_v = recovery.estimate_touchdown_velocity(vehicle_mass_kg=mass)

    return {
        "vehicle_mass_kg": mass,
        "touchdown_velocity_m_s": round(touchdown_v, 2),
        "touchdown_velocity_km_h": round(touchdown_v * 3.6, 2),
        "is_safe_landing": touchdown_v <= 7.5,
        "total_drag_area_m2": round(recovery.drogue.drag_area_cd_s + recovery.main.drag_area_cd_s, 2),
    }


def handle_simulate_staged_flight(args: Dict[str, Any]) -> Dict[str, Any]:
    elev = float(args.get("launch_elevation_deg", 85.0))
    payload = float(args.get("payload_mass_kg", 15.0))
    b_prop = float(args.get("booster_propellant_kg", 80.0))
    b_burn = float(args.get("booster_burn_s", 5.0))
    s_prop = float(args.get("sustainer_propellant_kg", 30.0))
    s_burn = float(args.get("sustainer_burn_s", 6.0))

    stages = [
        RocketStage(
            stage_number=1,
            name="Stage 1 Booster",
            dry_mass_kg=20.0,
            propellant_mass_kg=b_prop,
            burn_time_s=b_burn,
            isp_sea_level_s=240.0,
            isp_vacuum_s=270.0,
            ignition_delay_s=0.0,
            jettison_delay_s=0.5,
        ),
        RocketStage(
            stage_number=2,
            name="Stage 2 Sustainer",
            dry_mass_kg=8.0,
            propellant_mass_kg=s_prop,
            burn_time_s=s_burn,
            isp_sea_level_s=260.0,
            isp_vacuum_s=295.0,
            ignition_delay_s=1.0,
            jettison_delay_s=0.0,
        ),
    ]

    sim = StagedRocketSimulator(stages=stages, payload_mass_kg=payload)
    res = sim.simulate(launch_elevation_deg=elev)

    return {
        "status": "success",
        "vehicle_name": res.vehicle_name,
        "apogee_altitude_km": round(res.apogee_altitude_m / 1000.0, 2),
        "apogee_time_s": res.apogee_time_s,
        "max_mach": res.max_mach,
        "max_q_kpa": round(res.max_q_pa / 1000.0, 2),
        "total_events": len(res.staging_events),
        "staging_timeline": res.staging_events,
    }


def process_message(msg: Dict[str, Any]) -> Dict[str, Any]:
    method = msg.get("method")
    msg_id = msg.get("id")

    if method == "initialize":
        return {
            "jsonrpc": "2.0",
            "id": msg_id,
            "result": {
                "protocolVersion": "2024-11-05",
                "capabilities": {"tools": {}},
                "serverInfo": {
                    "name": "aero-flight-dynamics-mcp",
                    "version": "0.1.0"
                }
            }
        }

    elif method == "tools/list":
        return {
            "jsonrpc": "2.0",
            "id": msg_id,
            "result": {"tools": TOOLS}
        }

    elif method == "tools/call":
        params = msg.get("params", {})
        tool_name = params.get("name")
        tool_args = params.get("arguments", {})

        try:
            if tool_name == "simulate_flight":
                res = handle_simulate_flight(tool_args)
            elif tool_name == "query_atmosphere":
                res = handle_query_atmosphere(tool_args)
            elif tool_name == "evaluate_airfoil":
                res = handle_evaluate_airfoil(tool_args)
            elif tool_name == "evaluate_parachute_recovery":
                res = handle_evaluate_parachute(tool_args)
            elif tool_name == "simulate_staged_flight":
                res = handle_simulate_staged_flight(tool_args)
            else:
                return {
                    "jsonrpc": "2.0",
                    "id": msg_id,
                    "error": {"code": -32601, "message": f"Tool '{tool_name}' not found."}
                }

            return {
                "jsonrpc": "2.0",
                "id": msg_id,
                "result": {
                    "content": [{"type": "text", "text": json.dumps(res, indent=2)}]
                }
            }
        except Exception as e:
            return {
                "jsonrpc": "2.0",
                "id": msg_id,
                "error": {"code": -32000, "message": str(e)}
            }

    return {
        "jsonrpc": "2.0",
        "id": msg_id,
        "error": {"code": -32601, "message": f"Method '{method}' not supported."}
    }


def run_stdio_server() -> None:
    """Standard JSON-RPC line-delimited server over stdin/stdout."""
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            req = json.loads(line)
            resp = process_message(req)
            sys.stdout.write(json.dumps(resp) + "\n")
            sys.stdout.flush()
        except json.JSONDecodeError:
            err = {"jsonrpc": "2.0", "id": None, "error": {"code": -32700, "message": "Parse error"}}
            sys.stdout.write(json.dumps(err) + "\n")
            sys.stdout.flush()


if __name__ == "__main__":
    run_stdio_server()
