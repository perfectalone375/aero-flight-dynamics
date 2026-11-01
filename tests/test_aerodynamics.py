"""
Unit tests for aerodynamic geometry generation and coefficient modeling.
"""

import math

import pytest

from aero_flight_dynamics.aerodynamics.airfoil import NACA4Airfoil
from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients


def test_naca0012_symmetry(naca0012: NACA4Airfoil):
    assert naca0012.max_camber_m == 0.0
    assert naca0012.thickness_ratio_t == 0.12
    assert naca0012.zero_lift_angle_rad() == 0.0

    # At alpha = 0, lift is 0
    assert pytest.approx(naca0012.theoretical_lift_coefficient(0.0), abs=1e-6) == 0.0

    # Coordinates generated properly
    geom = naca0012.generate_coordinates(num_points=50)
    assert len(geom.x) == 51
    assert geom.x[0] == 0.0
    assert pytest.approx(geom.x[-1], abs=1e-5) == 1.0


def test_naca2412_camber_properties(naca2412: NACA4Airfoil):
    assert pytest.approx(naca2412.max_camber_m, abs=1e-4) == 0.02
    assert pytest.approx(naca2412.max_camber_pos_p, abs=1e-4) == 0.4
    assert pytest.approx(naca2412.thickness_ratio_t, abs=1e-4) == 0.12

    # Cambered airfoil has negative zero-lift angle
    alpha_0 = naca2412.zero_lift_angle_rad()
    assert alpha_0 < 0.0

    # Positive lift at alpha = 0
    cl_zero_alpha = naca2412.theoretical_lift_coefficient(0.0)
    assert cl_zero_alpha > 0.0


def test_invalid_naca_designation():
    with pytest.raises(ValueError):
        NACA4Airfoil("001")  # Too short
    with pytest.raises(ValueError):
        NACA4Airfoil("abcd")  # Non-digit


def test_aero_coefficients_prandtl_glauert_compressibility():
    aero = AeroCoefficients()
    alpha_rad = math.radians(4.0)

    cl_incompressible = aero.lift_coefficient(alpha_rad, mach=0.0)
    cl_mach_06 = aero.lift_coefficient(alpha_rad, mach=0.6)

    # Compressibility increases lift curve slope by 1 / sqrt(1 - M^2)
    expected_factor = 1.0 / math.sqrt(1.0 - 0.6 ** 2)  # 1 / 0.8 = 1.25
    assert pytest.approx(cl_mach_06 / cl_incompressible, rel=1e-3) == expected_factor


def test_transonic_wave_drag_divergence():
    aero = AeroCoefficients(mach_critical=0.75)
    alpha = math.radians(2.0)

    # Subsonic below M_crit: wave drag = 0
    cd_subsonic = aero.drag_coefficient(alpha, mach=0.6)
    # Transonic above M_crit: sharp drag rise
    cd_transonic = aero.drag_coefficient(alpha, mach=0.9)

    assert cd_transonic > cd_subsonic
    wave_drag_at_09 = aero.wave_drag_coefficient(0.9)
    assert wave_drag_at_09 > 0.0


def test_induced_drag_scaling():
    aero = AeroCoefficients(aspect_ratio=8.0, oswald_efficiency=0.85)
    # CD_i = CL^2 / (pi * AR * e)
    cd_i_at_1 = aero.induced_drag_coefficient(cl=1.0)
    cd_i_at_2 = aero.induced_drag_coefficient(cl=2.0)

    # Induced drag scales quadratically with CL
    assert pytest.approx(cd_i_at_2, rel=1e-3) == 4.0 * cd_i_at_1
