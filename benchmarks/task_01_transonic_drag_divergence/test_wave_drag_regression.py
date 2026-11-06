"""
Regression test case for AI training task: Transonic drag divergence stability.
Demonstrates Fail-to-Pass verification pattern required for SWE-bench evaluation.
"""

from aero_flight_dynamics.aerodynamics.coefficients import AeroCoefficients


def test_high_lift_transonic_drag_stability():
    """
    Verifies that high lift coefficients in transonic corridor (M=0.95, CL=1.5)
    do not cause unphysical numerical blowup in wave drag.
    """
    aero = AeroCoefficients(mach_critical=0.75)
    high_cl = 1.5
    mach_transonic = 0.95

    cd_wave = aero.wave_drag_coefficient(mach=mach_transonic, cl=high_cl)

    # Physical ceiling: Transonic wave drag for streamlined slender body should not exceed 0.45
    assert cd_wave < 0.45, f"Wave drag {cd_wave:.4f} exceeded physical stability ceiling"
    assert cd_wave > 0.0, "Wave drag must be positive in transonic regime"
