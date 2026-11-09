"""
Unit tests for 6-DoF quaternion rotational kinematics and rigid-body attitude dynamics.
"""

import math
from aero_flight_dynamics.trajectory.quaternions import (
    Quaternion,
    RigidBodyInertia,
    AttitudeDynamicsSimulator,
)
from aero_flight_dynamics.mcp.server import handle_propagate_attitude_quaternions


def test_quaternion_norm_and_normalization():
    q = Quaternion(1.0, 2.0, 3.0, 4.0)
    expected_norm = math.sqrt(1 + 4 + 9 + 16)
    assert abs(q.norm - expected_norm) < 1e-9

    q_unit = q.normalized()
    assert abs(q_unit.norm - 1.0) < 1e-12

    # Zero quaternion fallback
    q_zero = Quaternion(0.0, 0.0, 0.0, 0.0).normalized()
    assert q_zero.q0 == 1.0
    assert q_zero.norm == 1.0


def test_quaternion_hamilton_multiplication():
    # Fundamental quaternion units: i^2 = j^2 = k^2 = -1, ij = k, jk = i, ki = j
    qi = Quaternion(0.0, 1.0, 0.0, 0.0)
    qj = Quaternion(0.0, 0.0, 1.0, 0.0)
    qk = Quaternion(0.0, 0.0, 0.0, 1.0)

    # i * i = -1
    res_ii = qi * qi
    assert abs(res_ii.q0 - (-1.0)) < 1e-12
    assert abs(res_ii.q1) < 1e-12

    # i * j = k
    res_ij = qi * qj
    assert abs(res_ij.q3 - 1.0) < 1e-12

    # j * k = i
    res_jk = qj * qk
    assert abs(res_jk.q1 - 1.0) < 1e-12

    # k * i = j
    res_ki = qk * qi
    assert abs(res_ki.q2 - 1.0) < 1e-12


def test_vector_rotation_pure_axes():
    # 90 degrees around Z axis: vector (1, 0, 0) should become (0, 1, 0)
    qz_90 = Quaternion.from_axis_angle((0.0, 0.0, 1.0), 90.0)
    v_rot = qz_90.rotate_vector((1.0, 0.0, 0.0))

    assert abs(v_rot[0] - 0.0) < 1e-9
    assert abs(v_rot[1] - 1.0) < 1e-9
    assert abs(v_rot[2] - 0.0) < 1e-9

    # 180 degrees around X axis: vector (0, 1, 0) should become (0, -1, 0)
    qx_180 = Quaternion.from_axis_angle((1.0, 0.0, 0.0), 180.0)
    v_rot_x = qx_180.rotate_vector((0.0, 1.0, 0.0))
    assert abs(v_rot_x[0]) < 1e-9
    assert abs(v_rot_x[1] - (-1.0)) < 1e-9
    assert abs(v_rot_x[2]) < 1e-9


def test_euler_angle_roundtrip():
    # Test roll, pitch, yaw round trip
    orig_roll = 15.0
    orig_pitch = -25.0
    orig_yaw = 40.0

    q = Quaternion.from_euler_angles_deg(orig_roll, orig_pitch, orig_yaw)
    r_deg, p_deg, y_deg = q.to_euler_angles_deg()

    assert abs(r_deg - orig_roll) < 1e-4
    assert abs(p_deg - orig_pitch) < 1e-4
    assert abs(y_deg - orig_yaw) < 1e-4


def test_gimbal_lock_singularity_avoidance():
    # Pitch = 90 deg (extreme singularity point)
    q_gimbal = Quaternion.from_euler_angles_deg(0.0, 90.0, 30.0)
    r, p, y = q_gimbal.to_euler_angles_deg()

    assert not math.isnan(r)
    assert not math.isnan(p)
    assert not math.isnan(y)
    assert abs(p - 90.0) < 1e-4


def test_rigid_body_attitude_simulation_energy_conservation():
    # Torque-free symmetric body rotation with zero damping
    inertia = RigidBodyInertia(ixx=2.0, iyy=10.0, izz=10.0)
    sim = AttitudeDynamicsSimulator(inertia=inertia, damping_coefficients=(0.0, 0.0, 0.0))

    q_init = Quaternion(1.0, 0.0, 0.0, 0.0)
    result = sim.simulate(
        initial_quaternion=q_init,
        initial_omega_deg_s=(10.0, 20.0, 0.0),
        duration_s=2.0,
        dt=0.005,
    )

    # In torque-free motion, rotational kinetic energy T must be strictly conserved
    delta_energy = abs(result.final_energy_j - result.initial_energy_j)
    assert delta_energy / result.initial_energy_j < 1e-4
    assert abs(result.final_state.quaternion.norm - 1.0) < 1e-9


def test_attitude_damping_decay():
    # High damping should decrease angular kinetic energy
    inertia = RigidBodyInertia(ixx=1.0, iyy=1.0, izz=1.0)
    sim = AttitudeDynamicsSimulator(inertia=inertia, damping_coefficients=(0.5, 0.5, 0.5))

    q_init = Quaternion(1.0, 0.0, 0.0, 0.0)
    result = sim.simulate(
        initial_quaternion=q_init,
        initial_omega_deg_s=(30.0, 30.0, 30.0),
        duration_s=3.0,
        dt=0.01,
    )

    # Initial speed is sqrt(3 * 30^2) = 51.96 deg/s
    assert result.final_energy_j < result.initial_energy_j * 0.1
    assert result.final_state.total_angular_speed_deg_s < 15.0


def test_mcp_propagate_attitude_quaternions():
    args = {
        "roll_deg": 5.0,
        "pitch_deg": 10.0,
        "yaw_deg": 0.0,
        "p_deg_s": 0.0,
        "q_deg_s": 15.0,
        "r_deg_s": 0.0,
        "duration_s": 1.0,
    }
    resp = handle_propagate_attitude_quaternions(args)

    assert resp["status"] == "success"
    assert "final_euler_deg" in resp
    assert "final_quaternion" in resp
    assert resp["final_energy_j"] <= resp["initial_energy_j"]
    assert resp["simulation_steps"] > 50
