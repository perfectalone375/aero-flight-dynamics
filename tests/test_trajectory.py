"""
Integration tests for 3-DoF trajectory simulation and vehicle dynamics.
"""

import pytest

from aero_flight_dynamics.trajectory.simulation import FlightSimulator
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle


def test_vehicle_mass_tracking(standard_vehicle: PointMassVehicle):
    # Total initial mass = structural_mass (20) + motor_dry (10) + motor_prop (30) = 60 kg
    assert standard_vehicle.total_mass(0.0) == 60.0

    # Post burnout mass = structural_mass (20) + motor_dry (10) = 30 kg
    assert standard_vehicle.total_mass(6.0) == 30.0


def test_trajectory_simulation_execution(standard_vehicle: PointMassVehicle):
    sim = FlightSimulator(vehicle=standard_vehicle)
    result = sim.simulate(
        launch_elevation_deg=85.0,
        launch_azimuth_deg=90.0,
        dt=0.05,
        max_time_s=150.0,
    )

    # Validate output metrics
    assert result.apogee_m > 1000.0, "Rocket should reach at least 1 km altitude"
    assert result.apogee_time_s > result.burnout_time_s, "Apogee must occur after motor burnout"
    assert result.max_mach > 0.5, "Rocket should reach significant subsonic or supersonic speed"
    assert result.max_dynamic_pressure_pa > 0.0, "Max-Q should be strictly positive"
    assert result.total_flight_time_s > 10.0, "Flight duration should be greater than 10 seconds"
    assert len(result.history) > 100, "Trajectory history should contain substantial state vectors"


def test_gravity_gradient_at_high_altitude(standard_vehicle: PointMassVehicle):
    sim = FlightSimulator(vehicle=standard_vehicle)
    g_sea_level = sim.gravity_at_altitude(0.0)
    g_100km = sim.gravity_at_altitude(100000.0)

    assert pytest.approx(g_sea_level, abs=1e-3) == 9.80665
    assert g_100km < g_sea_level
    # At 100 km, g is approximately 9.5 m/s^2
    assert 9.4 < g_100km < 9.6
