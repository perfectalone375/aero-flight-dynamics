"""
Unit tests for atmospheric wind profile, boundary layer shear, and discrete gusts.
"""

import math
import pytest
from aero_flight_dynamics.atmosphere.wind import WindModel


def test_wind_boundary_layer_shear():
    wind = WindModel(surface_wind_speed_m_s=4.0, reference_altitude_m=10.0, shear_exponent=0.14)

    # At reference altitude (10m), mean wind speed should equal surface reference
    v_10m = wind.mean_wind_speed(10.0)
    assert pytest.approx(v_10m, abs=1e-3) == 4.0

    # Wind speed increases with altitude
    v_100m = wind.mean_wind_speed(100.0)
    assert v_100m > v_10m


def test_discrete_gust_activation():
    wind = WindModel(
        gust_amplitude_m_s=6.0,
        gust_altitude_start_m=500.0,
        gust_length_m=200.0
    )

    # Outside gust window: 0 gust speed
    assert wind.discrete_gust_speed(400.0) == 0.0
    assert wind.discrete_gust_speed(800.0) == 0.0

    # At peak of gust (midpoint at z = 600m): (1 - cos(pi)) / 2 = 1.0 -> full amplitude
    v_peak = wind.discrete_gust_speed(600.0)
    assert pytest.approx(v_peak, abs=1e-3) == 6.0


def test_wind_velocity_vector_magnitude():
    wind = WindModel(surface_wind_speed_m_s=5.0, wind_azimuth_deg=90.0)
    wx, wy, wz = wind.wind_velocity_vector(10.0)

    # Magnitude should match speed at 10m
    speed = math.sqrt(wx ** 2 + wy ** 2 + wz ** 2)
    assert pytest.approx(speed, abs=1e-3) == 5.0
