"""
Unit tests for solid rocket motor propulsion dynamics.
"""

import pytest

from aero_flight_dynamics.propulsion.solid_motor import MotorBurnProfile, SolidRocketMotor


def test_motor_mass_depletion(standard_motor: SolidRocketMotor):
    assert standard_motor.total_initial_mass_kg == 40.0
    assert standard_motor.current_propellant_mass(0.0) == 30.0

    # Halfway through burn (t = 2.5s for 5.0s burn)
    mid_prop = standard_motor.current_propellant_mass(2.5)
    assert pytest.approx(mid_prop, abs=1e-3) == 15.0
    assert pytest.approx(standard_motor.current_motor_mass(2.5), abs=1e-3) == 25.0

    # After burnout (t = 5.0s and t = 6.0s)
    assert standard_motor.current_propellant_mass(5.0) == 0.0
    assert standard_motor.current_propellant_mass(6.0) == 0.0
    assert standard_motor.current_motor_mass(6.0) == 10.0  # Only dry mass remains


def test_motor_thrust_generation(standard_motor: SolidRocketMotor):
    # Thrust at t = 1.0s should be positive
    thrust_sl = standard_motor.thrust(1.0, ambient_pressure_pa=101325.0)
    assert thrust_sl > 0.0

    # Thrust after burn time should be exactly 0
    thrust_post = standard_motor.thrust(5.1, ambient_pressure_pa=101325.0)
    assert thrust_post == 0.0


def test_altitude_isp_compensation(standard_motor: SolidRocketMotor):
    # Sea level Isp
    isp_sl = standard_motor.effective_isp(101325.0)
    assert pytest.approx(isp_sl, abs=1e-3) == 210.0

    # Vacuum Isp (0 Pa)
    isp_vac = standard_motor.effective_isp(0.0)
    assert pytest.approx(isp_vac, abs=1e-3) == 240.0

    # Mid altitude (e.g. 50 kPa)
    isp_mid = standard_motor.effective_isp(50000.0)
    assert 210.0 < isp_mid < 240.0


def test_boost_sustain_burn_profile():
    motor = SolidRocketMotor(
        dry_mass_kg=5.0,
        propellant_mass_kg=20.0,
        burn_time_s=10.0,
        profile=MotorBurnProfile.BOOST_SUSTAIN,
    )
    # At 30% time (t=3s), 60% of propellant (12 kg) consumed, 8 kg remains
    prop_at_boost_end = motor.current_propellant_mass(3.0)
    assert pytest.approx(prop_at_boost_end, abs=1e-2) == 8.0
