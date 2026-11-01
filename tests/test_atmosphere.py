"""
Unit tests for the U.S. Standard Atmosphere (1976) model.
Validated against published NOAA/NASA tables.
"""

import pytest

from aero_flight_dynamics.atmosphere.us_standard_1976 import USStandardAtmosphere1976


def test_sea_level_standard_properties(atmosphere: USStandardAtmosphere1976):
    state = atmosphere.compute(0.0)
    assert state.altitude_m == 0.0
    assert state.geopotential_altitude_m == 0.0
    assert pytest.approx(state.temperature_k, abs=1e-2) == 288.15
    assert pytest.approx(state.pressure_pa, abs=1.0) == 101325.0
    assert pytest.approx(state.density_kg_m3, abs=1e-3) == 1.225
    assert pytest.approx(state.speed_of_sound_m_s, rel=1e-3) == 340.294


def test_tropopause_boundary_11km(atmosphere: USStandardAtmosphere1976):
    """At 11,000 m geopotential, temperature should reach 216.65 K."""
    z_geom = atmosphere.geopotential_to_geometric(11000.0)
    state = atmosphere.compute(z_geom)
    assert pytest.approx(state.temperature_k, abs=1e-2) == 216.65
    assert pytest.approx(state.pressure_pa, rel=1e-3) == 22632.06


def test_isothermal_stratosphere_20km(atmosphere: USStandardAtmosphere1976):
    """Between 11 km and 20 km geopotential, lapse rate is zero (isothermal at 216.65 K)."""
    z_geom = atmosphere.geopotential_to_geometric(20000.0)
    state = atmosphere.compute(z_geom)
    assert pytest.approx(state.temperature_k, abs=1e-2) == 216.65
    assert pytest.approx(state.pressure_pa, rel=1e-3) == 5474.89


def test_stratopause_boundary_47km(atmosphere: USStandardAtmosphere1976):
    """At 47,000 m geopotential, temperature reaches secondary peak around 270.65 K."""
    z_geom = atmosphere.geopotential_to_geometric(47000.0)
    state = atmosphere.compute(z_geom)
    assert pytest.approx(state.temperature_k, abs=1e-1) == 270.65


def test_dynamic_pressure_and_mach_helpers(atmosphere: USStandardAtmosphere1976):
    state = atmosphere.compute(0.0)
    # v = 170.147 m/s -> Mach 0.5 at sea level
    mach = state.mach_number(170.147)
    assert pytest.approx(mach, rel=1e-3) == 0.5

    # Dynamic pressure q = 0.5 * rho * v^2 = 0.5 * 1.225 * 100^2 = 6125 Pa
    q = state.dynamic_pressure(100.0)
    assert pytest.approx(q, rel=1e-3) == 6125.0


def test_negative_altitude_safe_clamping(atmosphere: USStandardAtmosphere1976):
    """Negative altitude should be safely clamped to sea level."""
    state = atmosphere.compute(-500.0)
    assert state.altitude_m == 0.0
    assert pytest.approx(state.pressure_pa, abs=1.0) == 101325.0


def test_high_altitude_vacuum_extension(atmosphere: USStandardAtmosphere1976):
    """Altitudes > 86 km should return valid non-negative near-vacuum states."""
    state = atmosphere.compute(100000.0)  # 100 km Karman line
    assert state.pressure_pa > 0.0
    assert state.pressure_pa < 1.0  # Under 1 Pa
    assert state.density_kg_m3 > 0.0
    assert state.density_kg_m3 < 1e-5
