"""
Pytest fixtures and test configurations for aero_flight_dynamics.
"""

import pytest
from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976
from aero_flight_dynamics.aerodynamics.airfoil import NACA4Airfoil
from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients
from aero_flight_dynamics.propulsion.solid_motor import SolidRocketMotor, MotorBurnProfile
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle


@pytest.fixture
def atmosphere():
    return USStandardAtmosphere1976()


@pytest.fixture
def standard_motor():
    return SolidRocketMotor(
        name="Test Motor Mk-I",
        dry_mass_kg=10.0,
        propellant_mass_kg=30.0,
        burn_time_s=5.0,
        isp_sea_level_s=210.0,
        isp_vacuum_s=240.0,
        profile=MotorBurnProfile.NEUTRAL,
    )


@pytest.fixture
def standard_vehicle(standard_motor):
    return PointMassVehicle(
        name="Sounding Rocket Alpha",
        structural_mass_kg=20.0,
        reference_area_m2=0.0314,
        characteristic_length_m=2.0,
        aerodynamics=AeroCoefficients(cd0=0.25, aspect_ratio=6.0),
        motor=standard_motor,
    )


@pytest.fixture
def naca0012():
    return NACA4Airfoil("0012", chord_m=1.0)


@pytest.fixture
def naca2412():
    return NACA4Airfoil("2412", chord_m=1.5)
