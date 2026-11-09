"""
Quaternion-based 6-DoF rotational kinematics and rigid-body attitude dynamics.
Provides singularity-free orientation tracking, Euler angle conversions,
and Runge-Kutta 4th order (RK4) angular velocity integration for aerospace vehicles.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Tuple


@dataclass
class Quaternion:
    """
    Unit quaternion representation: q = [q0, q1, q2, q3]
    where q0 is the scalar (real) component, and (q1, q2, q3) is the vector component.
    Satisfies q0^2 + q1^2 + q2^2 + q3^2 = 1 for pure rotations.
    """

    q0: float = 1.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0

    @property
    def norm(self) -> float:
        """Euclidean norm of the quaternion."""
        return math.sqrt(self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2)

    def normalized(self) -> Quaternion:
        """Return unit-normalized quaternion to prevent numerical integration drift."""
        n = self.norm
        if n < 1e-15:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.q0 / n, self.q1 / n, self.q2 / n, self.q3 / n)

    def conjugate(self) -> Quaternion:
        """Return quaternion conjugate q* = [q0, -q1, -q2, -q3]."""
        return Quaternion(self.q0, -self.q1, -self.q2, -self.q3)

    def inverse(self) -> Quaternion:
        """Return quaternion inverse q^-1 = q* / ||q||^2."""
        n2 = self.q0**2 + self.q1**2 + self.q2**2 + self.q3**2
        if n2 < 1e-15:
            return Quaternion(1.0, 0.0, 0.0, 0.0)
        return Quaternion(self.q0 / n2, -self.q1 / n2, -self.q2 / n2, -self.q3 / n2)

    def __mul__(self, other: Quaternion) -> Quaternion:
        """Hamiltonian quaternion product q_result = self * other."""
        w1, x1, y1, z1 = self.q0, self.q1, self.q2, self.q3
        w2, x2, y2, z2 = other.q0, other.q1, other.q2, other.q3

        return Quaternion(
            q0=w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            q1=w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            q2=w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            q3=w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def rotate_vector(self, v: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """
        Rotate 3D vector v from Body frame to Inertial frame using v' = q * v * q^-1.
        """
        vq = Quaternion(0.0, v[0], v[1], v[2])
        res = (self * vq) * self.conjugate()
        return (res.q1, res.q2, res.q3)

    def to_rotation_matrix(self) -> List[List[float]]:
        """
        Convert unit quaternion to 3x3 Direction Cosine Matrix (DCM / Rotation Matrix).
        Transforms inertial frame coordinates into body frame coordinates.
        """
        q = self.normalized()
        w, x, y, z = q.q0, q.q1, q.q2, q.q3

        return [
            [
                1.0 - 2.0 * (y**2 + z**2),
                2.0 * (x * y + w * z),
                2.0 * (x * z - w * y),
            ],
            [
                2.0 * (x * y - w * z),
                1.0 - 2.0 * (x**2 + z**2),
                2.0 * (y * z + w * x),
            ],
            [
                2.0 * (x * z + w * y),
                2.0 * (y * z - w * x),
                1.0 - 2.0 * (x**2 + y**2),
            ],
        ]

    def to_euler_angles_deg(self) -> Tuple[float, float, float]:
        """
        Extract 3-2-1 Aerospace Euler Angles: Roll (phi), Pitch (theta), Yaw (psi) in degrees.
        Includes singularity protection at pitch +/- 90 degrees (gimbal lock avoidance).
        """
        q = self.normalized()
        w, x, y, z = q.q0, q.q1, q.q2, q.q3

        # Pitch (theta)
        sin_theta = 2.0 * (w * y - z * x)
        sin_theta = max(-1.0, min(1.0, sin_theta))

        if abs(sin_theta) >= 0.999999:
            # Gimbal lock singularity at +/- 90 deg pitch
            pitch_rad = math.copysign(math.pi / 2.0, sin_theta)
            roll_rad = 0.0
            yaw_rad = math.atan2(-2.0 * (x * y - w * z), 1.0 - 2.0 * (y**2 + z**2))
        else:
            pitch_rad = math.asin(sin_theta)
            roll_rad = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
            yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

        return (
            math.degrees(roll_rad),
            math.degrees(pitch_rad),
            math.degrees(yaw_rad),
        )

    @classmethod
    def from_euler_angles_deg(cls, roll_deg: float, pitch_deg: float, yaw_deg: float) -> Quaternion:
        """Construct unit quaternion from aerospace Euler angles (Roll-Pitch-Yaw, 3-2-1) in degrees."""
        phi = math.radians(roll_deg) / 2.0
        theta = math.radians(pitch_deg) / 2.0
        psi = math.radians(yaw_deg) / 2.0

        c_phi, s_phi = math.cos(phi), math.sin(phi)
        c_th, s_th = math.cos(theta), math.sin(theta)
        c_psi, s_psi = math.cos(psi), math.sin(psi)

        return cls(
            q0=c_phi * c_th * c_psi + s_phi * s_th * s_psi,
            q1=s_phi * c_th * c_psi - c_phi * s_th * s_psi,
            q2=c_phi * s_th * c_psi + s_phi * c_th * s_psi,
            q3=c_phi * c_th * s_psi - s_phi * s_th * c_psi,
        ).normalized()

    @classmethod
    def from_axis_angle(cls, axis: Tuple[float, float, float], angle_deg: float) -> Quaternion:
        """Construct unit quaternion from rotation axis (ax, ay, az) and angle in degrees."""
        ax, ay, az = axis
        norm = math.sqrt(ax**2 + ay**2 + az**2)
        if norm < 1e-15:
            return cls(1.0, 0.0, 0.0, 0.0)

        half_rad = math.radians(angle_deg) / 2.0
        s = math.sin(half_rad) / norm
        return cls(
            q0=math.cos(half_rad),
            q1=ax * s,
            q2=ay * s,
            q3=az * s,
        ).normalized()

    def derivative(self, omega: Tuple[float, float, float]) -> Tuple[float, float, float, float]:
        """
        Compute kinematic quaternion time derivative dq/dt:
        dq/dt = 0.5 * Omega(omega) * q
        where omega = (p, q, r) is body angular velocity in rad/s.
        """
        p, q_rate, r = omega
        w, x, y, z = self.q0, self.q1, self.q2, self.q3

        dq0 = 0.5 * (-p * x - q_rate * y - r * z)
        dq1 = 0.5 * (p * w + r * y - q_rate * z)
        dq2 = 0.5 * (q_rate * w - r * x + p * z)
        dq3 = 0.5 * (r * w + q_rate * x - p * y)

        return (dq0, dq1, dq2, dq3)


@dataclass
class RigidBodyInertia:
    """Principal moments of inertia for a flight vehicle (kg * m^2)."""

    ixx: float = 1.25  # Roll inertia
    iyy: float = 45.0  # Pitch inertia
    izz: float = 45.0  # Yaw inertia
    ixy: float = 0.0
    ixz: float = 0.0
    iyz: float = 0.0

    @property
    def is_symmetric(self) -> bool:
        """Check if vehicle is transversely symmetric (iyy == izz, products zero)."""
        return (
            abs(self.iyy - self.izz) < 1e-6 and abs(self.ixy) < 1e-6 and abs(self.ixz) < 1e-6 and abs(self.iyz) < 1e-6
        )


@dataclass
class AttitudeState:
    """State vector for rigid body attitude dynamics at time t."""

    time_s: float
    quaternion: Quaternion
    omega_rad_s: Tuple[float, float, float]  # (p, q, r) in body axes

    @property
    def roll_pitch_yaw_deg(self) -> Tuple[float, float, float]:
        """Return current Euler angles (Roll, Pitch, Yaw) in degrees."""
        return self.quaternion.to_euler_angles_deg()

    @property
    def total_angular_speed_deg_s(self) -> float:
        """Total scalar angular velocity magnitude in deg/s."""
        mag_rad = math.sqrt(sum(w**2 for w in self.omega_rad_s))
        return math.degrees(mag_rad)

    def rotational_kinetic_energy_j(self, inertia: RigidBodyInertia) -> float:
        """Rotational kinetic energy T = 0.5 * omega^T * I * omega."""
        p, q, r = self.omega_rad_s
        return 0.5 * (inertia.ixx * p**2 + inertia.iyy * q**2 + inertia.izz * r**2)


@dataclass
class AttitudeSimulationResult:
    """Simulation trajectory results for 6-DoF rigid-body attitude dynamics."""

    final_time_s: float
    final_state: AttitudeState
    history: List[AttitudeState] = field(default_factory=list)
    initial_energy_j: float = 0.0
    final_energy_j: float = 0.0
    max_angular_rate_deg_s: float = 0.0


class AttitudeDynamicsSimulator:
    """
    Solves coupled Euler rotational equations of motion and quaternion kinematics:
    I * d(omega)/dt + omega x (I * omega) = M_ext
    dq/dt = 0.5 * q * omega
    using 4th-order Runge-Kutta numerical integration with quaternion normalization.
    """

    def __init__(
        self,
        inertia: Optional[RigidBodyInertia] = None,
        damping_coefficients: Tuple[float, float, float] = (0.05, 0.1, 0.1),
    ) -> None:
        self.inertia = inertia or RigidBodyInertia()
        self.damping_coeffs = damping_coefficients

    def _compute_angular_acceleration(
        self,
        omega: Tuple[float, float, float],
        external_torques: Tuple[float, float, float],
    ) -> Tuple[float, float, float]:
        """
        Evaluate Euler's equations:
        d(omega)/dt = I^-1 * (M_ext - omega x (I * omega))
        """
        p, q, r = omega
        ixx, iyy, izz = self.inertia.ixx, self.inertia.iyy, self.inertia.izz
        mx, my, mz = external_torques

        # Gyroscopic cross-coupling moments
        # For diagonal inertia tensor:
        # Ixx * dp/dt = Mx - (Izz - Iyy) * q * r
        # Iyy * dq/dt = My - (Ixx - Izz) * p * r
        # Izz * dr/dt = Mz - (Iyy - Ixx) * p * q
        dp_dt = (mx - (izz - iyy) * q * r) / ixx
        dq_dt = (my - (ixx - izz) * p * r) / iyy
        dr_dt = (mz - (iyy - ixx) * p * q) / izz

        return (dp_dt, dq_dt, dr_dt)

    def simulate(
        self,
        initial_quaternion: Quaternion,
        initial_omega_deg_s: Tuple[float, float, float] = (0.0, 5.0, 0.0),
        duration_s: float = 10.0,
        dt: float = 0.01,
        torque_func: Optional[Callable[[float, AttitudeState], Tuple[float, float, float]]] = None,
    ) -> AttitudeSimulationResult:
        """
        Integrate attitude state over time using RK4 with quaternion normalization.
        """
        omega_rad = tuple(math.radians(w) for w in initial_omega_deg_s)
        current_q = initial_quaternion.normalized()
        current_state = AttitudeState(time_s=0.0, quaternion=current_q, omega_rad_s=omega_rad)

        history: List[AttitudeState] = [current_state]
        t = 0.0
        max_rate = current_state.total_angular_speed_deg_s
        initial_energy = current_state.rotational_kinetic_energy_j(self.inertia)

        while t < duration_s:
            step_dt = min(dt, duration_s - t)

            # RK4 Stage 1
            torques_1 = (
                torque_func(t, current_state) if torque_func else self._default_damping(current_state.omega_rad_s)
            )
            k1_dw = self._compute_angular_acceleration(current_state.omega_rad_s, torques_1)
            k1_dq = current_state.quaternion.derivative(current_state.omega_rad_s)

            # RK4 Stage 2
            w_mid1 = tuple(w + 0.5 * step_dt * dw for w, dw in zip(current_state.omega_rad_s, k1_dw))
            q_mid1 = Quaternion(
                current_state.quaternion.q0 + 0.5 * step_dt * k1_dq[0],
                current_state.quaternion.q1 + 0.5 * step_dt * k1_dq[1],
                current_state.quaternion.q2 + 0.5 * step_dt * k1_dq[2],
                current_state.quaternion.q3 + 0.5 * step_dt * k1_dq[3],
            ).normalized()
            state_mid1 = AttitudeState(time_s=t + 0.5 * step_dt, quaternion=q_mid1, omega_rad_s=w_mid1)
            torques_2 = torque_func(t + 0.5 * step_dt, state_mid1) if torque_func else self._default_damping(w_mid1)
            k2_dw = self._compute_angular_acceleration(w_mid1, torques_2)
            k2_dq = q_mid1.derivative(w_mid1)

            # RK4 Stage 3
            w_mid2 = tuple(w + 0.5 * step_dt * dw for w, dw in zip(current_state.omega_rad_s, k2_dw))
            q_mid2 = Quaternion(
                current_state.quaternion.q0 + 0.5 * step_dt * k2_dq[0],
                current_state.quaternion.q1 + 0.5 * step_dt * k2_dq[1],
                current_state.quaternion.q2 + 0.5 * step_dt * k2_dq[2],
                current_state.quaternion.q3 + 0.5 * step_dt * k2_dq[3],
            ).normalized()
            state_mid2 = AttitudeState(time_s=t + 0.5 * step_dt, quaternion=q_mid2, omega_rad_s=w_mid2)
            torques_3 = torque_func(t + 0.5 * step_dt, state_mid2) if torque_func else self._default_damping(w_mid2)
            k3_dw = self._compute_angular_acceleration(w_mid2, torques_3)
            k3_dq = q_mid2.derivative(w_mid2)

            # RK4 Stage 4
            w_end = tuple(w + step_dt * dw for w, dw in zip(current_state.omega_rad_s, k3_dw))
            q_end = Quaternion(
                current_state.quaternion.q0 + step_dt * k3_dq[0],
                current_state.quaternion.q1 + step_dt * k3_dq[1],
                current_state.quaternion.q2 + step_dt * k3_dq[2],
                current_state.quaternion.q3 + step_dt * k3_dq[3],
            ).normalized()
            state_end = AttitudeState(time_s=t + step_dt, quaternion=q_end, omega_rad_s=w_end)
            torques_4 = torque_func(t + step_dt, state_end) if torque_func else self._default_damping(w_end)
            k4_dw = self._compute_angular_acceleration(w_end, torques_4)
            k4_dq = q_end.derivative(w_end)

            # Final RK4 update
            w_next = tuple(
                w + (step_dt / 6.0) * (dw1 + 2.0 * dw2 + 2.0 * dw3 + dw4)
                for w, dw1, dw2, dw3, dw4 in zip(current_state.omega_rad_s, k1_dw, k2_dw, k3_dw, k4_dw)
            )
            q_next = Quaternion(
                current_state.quaternion.q0 + (step_dt / 6.0) * (k1_dq[0] + 2.0 * k2_dq[0] + 2.0 * k3_dq[0] + k4_dq[0]),
                current_state.quaternion.q1 + (step_dt / 6.0) * (k1_dq[1] + 2.0 * k2_dq[1] + 2.0 * k3_dq[1] + k4_dq[1]),
                current_state.quaternion.q2 + (step_dt / 6.0) * (k1_dq[2] + 2.0 * k2_dq[2] + 2.0 * k3_dq[2] + k4_dq[2]),
                current_state.quaternion.q3 + (step_dt / 6.0) * (k1_dq[3] + 2.0 * k2_dq[3] + 2.0 * k3_dq[3] + k4_dq[3]),
            ).normalized()

            t += step_dt
            current_state = AttitudeState(time_s=t, quaternion=q_next, omega_rad_s=w_next)
            history.append(current_state)

            if current_state.total_angular_speed_deg_s > max_rate:
                max_rate = current_state.total_angular_speed_deg_s

        final_energy = current_state.rotational_kinetic_energy_j(self.inertia)

        return AttitudeSimulationResult(
            final_time_s=t,
            final_state=current_state,
            history=history,
            initial_energy_j=initial_energy,
            final_energy_j=final_energy,
            max_angular_rate_deg_s=max_rate,
        )

    def _default_damping(self, omega: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Passive aerodynamic/viscous damping torque M = -c * omega."""
        return (
            -self.damping_coeffs[0] * omega[0],
            -self.damping_coeffs[1] * omega[1],
            -self.damping_coeffs[2] * omega[2],
        )
