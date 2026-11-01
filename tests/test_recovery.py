"""
Unit tests for the parachute recovery subsystem and terminal velocity calculation.
"""

import pytest
from aero_flight_dynamics.recovery.parachute import DualRecoverySystem, Parachute, ParachuteState


def test_parachute_geometry_and_drag_area():
    chute = Parachute(name="Main Chute", diameter_m=2.0, cd=1.5)
    # Area = pi * r^2 = pi * 1.0^2 ~ 3.14159 m^2
    assert pytest.approx(chute.reference_area_m2, rel=1e-3) == 3.14159
    assert pytest.approx(chute.drag_area_cd_s, rel=1e-3) == 1.5 * 3.14159


def test_parachute_inflation_timer():
    chute = Parachute(diameter_m=2.0, cd=1.5, inflation_time_s=1.0)
    chute.state = ParachuteState.FULLY_DEPLOYED

    # Halfway through inflation (0.5s), area scales as fraction^2 = (0.5)^2 = 0.25
    cd_s_half = chute.current_drag_area(0.5)
    assert pytest.approx(cd_s_half / chute.drag_area_cd_s, rel=1e-2) == 0.25

    # Fully inflated
    assert pytest.approx(chute.current_drag_area(1.5), abs=1e-3) == chute.drag_area_cd_s


def test_dual_recovery_touchdown_velocity():
    recovery = DualRecoverySystem()
    # 50 kg rocket landing under standard air density
    v_touchdown = recovery.estimate_touchdown_velocity(vehicle_mass_kg=50.0)

    # Safe touchdown velocity should be between 4 m/s and 7.5 m/s
    assert 4.0 < v_touchdown < 7.5
