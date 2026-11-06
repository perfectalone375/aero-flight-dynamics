"""
Unit tests for multi-stage rocket trajectory dynamics and stage separation.
"""

import pytest
from aero_flight_dynamics.trajectory.staging import (
    RocketStage,
    StagedRocketSimulator,
    StagedSimulationResult,
)


@pytest.fixture
def two_stage_config():
    stage1 = RocketStage(
        stage_number=1,
        name="Booster",
        dry_mass_kg=25.0,
        propellant_mass_kg=75.0,
        burn_time_s=5.0,
        isp_sea_level_s=230.0,
        isp_vacuum_s=260.0,
        ignition_delay_s=0.0,
        jettison_delay_s=0.5,
    )
    stage2 = RocketStage(
        stage_number=2,
        name="Sustainer",
        dry_mass_kg=10.0,
        propellant_mass_kg=30.0,
        burn_time_s=6.0,
        isp_sea_level_s=250.0,
        isp_vacuum_s=290.0,
        ignition_delay_s=1.0,
        jettison_delay_s=0.0,
    )
    return [stage1, stage2]


def test_staged_simulator_runs_and_separates(two_stage_config):
    sim = StagedRocketSimulator(stages=two_stage_config, payload_mass_kg=15.0)
    res = sim.simulate(launch_elevation_deg=85.0, dt=0.05, max_time_s=150.0)

    assert isinstance(res, StagedSimulationResult)
    assert res.apogee_altitude_m > 5000.0, "2-stage rocket should easily exceed 5 km"
    assert res.max_mach > 1.0, "2-stage rocket should go supersonic"
    assert res.max_q_pa > 0.0

    # Verify staging events occurred in sequence
    event_names = [e["event"] for e in res.staging_events]
    assert any("Ignition of Stage 1" in e for e in event_names)
    assert any("Burnout of Stage 1" in e for e in event_names)
    assert any("Jettison of Stage 1" in e for e in event_names)
    assert any("Ignition of Stage 2" in e for e in event_names)


def test_invalid_stages():
    with pytest.raises(ValueError, match="at least one stage"):
        StagedRocketSimulator(stages=[])

