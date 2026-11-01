"""
Unit tests for telemetry recorder, CSV/JSON exporters, and summary reporting.
"""

import json
import os
import pytest
from aero_flight_dynamics.trajectory.simulation import FlightSimulator
from aero_flight_dynamics.trajectory.vehicle import PointMassVehicle
from aero_flight_dynamics.telemetry.recorder import FlightRecorder


def test_telemetry_exports(standard_vehicle: PointMassVehicle, tmp_path):
    sim = FlightSimulator(vehicle=standard_vehicle)
    result = sim.simulate(launch_elevation_deg=85.0, dt=0.1, max_time_s=60.0)

    recorder = FlightRecorder(result)
    summary = recorder.summary()

    assert summary.apogee_altitude_km > 1.0
    assert summary.max_mach > 0.0

    # Markdown report
    md = recorder.generate_markdown_report()
    assert "# Flight Telemetry Assessment Report" in md
    assert "Apogee Altitude" in md

    # CSV export
    csv_file = str(tmp_path / "telemetry.csv")
    recorder.export_csv(csv_file)
    assert os.path.exists(csv_file)
    with open(csv_file, "r", encoding="utf-8") as f:
        lines = f.readlines()
        assert len(lines) > 10
        assert "time_s,x_m,y_m,z_m" in lines[0]

    # JSON export
    json_file = str(tmp_path / "telemetry.json")
    recorder.export_json(json_file)
    assert os.path.exists(json_file)
    with open(json_file, "r", encoding="utf-8") as f:
        data = json.load(f)
        assert "mission_summary" in data
        assert "trajectory_samples" in data
