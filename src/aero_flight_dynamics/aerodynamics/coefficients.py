"""
Aerodynamic force and moment coefficient evaluation across subsonic,
transonic, and supersonic Mach regimes.
"""

import math
from dataclasses import dataclass


@dataclass
class AeroCoefficients:
    """
    Computes total aerodynamic coefficients (CL, CD, Cm) as functions of
    angle of attack (alpha), Mach number, and vehicle geometry.
    """
    cd0: float = 0.025               # Zero-lift parasitic drag coefficient
    aspect_ratio: float = 8.0         # Wing/body aspect ratio (b^2 / S)
    oswald_efficiency: float = 0.85   # Oswald efficiency factor e
    cl_alpha_0: float = 5.73          # Lift-curve slope per radian at M=0 (2*pi ~ 6.28 for 2D, ~5.7 for finite wing)
    alpha_stall_deg: float = 14.0     # Stall angle of attack in degrees
    mach_critical: float = 0.75       # Drag divergence critical Mach number
    wave_drag_factor: float = 20.0    # Transonic wave drag sharpness coefficient

    def lift_coefficient(self, alpha_rad: float, mach: float = 0.0) -> float:
        """
        Calculates lift coefficient CL accounting for angle of attack, stall,
        and compressibility corrections.
        """
        alpha_deg = math.degrees(alpha_rad)
        stall_rad = math.radians(self.alpha_stall_deg)

        # Baseline linear lift slope before stall
        if abs(alpha_rad) <= stall_rad:
            cl_subsonic = self.cl_alpha_0 * alpha_rad
        else:
            # Post-stall decay (simplified Viterna/flat-plate transition)
            sign = 1.0 if alpha_rad >= 0 else -1.0
            cl_max = self.cl_alpha_0 * stall_rad
            excess_alpha = abs(alpha_rad) - stall_rad
            cl_subsonic = sign * (cl_max * math.exp(-2.5 * excess_alpha) + 1.1 * math.sin(2.0 * alpha_rad))

        # Mach compressibility adjustments
        if mach < 0.0:
            mach = 0.0

        if mach < 0.8:
            # Prandtl-Glauert subsonic compressibility correction
            beta = math.sqrt(max(0.01, 1.0 - mach ** 2))
            return cl_subsonic / beta
        elif mach < 1.2:
            # Transonic interpolation zone
            cl_at_08 = cl_subsonic / math.sqrt(1.0 - 0.8 ** 2)
            # Supersonic Ackeret target at M=1.2
            cl_at_12 = (4.0 * alpha_rad) / math.sqrt(1.2 ** 2 - 1.0)
            tau = (mach - 0.8) / (1.2 - 0.8)
            return (1.0 - tau) * cl_at_08 + tau * cl_at_12
        else:
            # Supersonic linearized Ackeret theory
            beta = math.sqrt(mach ** 2 - 1.0)
            return (4.0 * alpha_rad) / beta

    def wave_drag_coefficient(self, mach: float, cl: float = 0.0) -> float:
        """
        Calculates transonic wave drag rise CD_wave above the critical Mach number.
        Uses Lock's empirical model: CD_wave = 20 * (M - M_div)^4 for M > M_div.
        """
        if mach <= 0:
            return 0.0

        # Effective critical Mach decreases slightly with lift (Korn equation effect)
        # Saturated to prevent unphysical numerical divergence in high-alpha maneuvers
        lift_shift = min(0.12, 0.1 * abs(cl))
        m_eff_crit = max(0.55, self.mach_critical - lift_shift)

        if mach <= m_eff_crit:
            return 0.0
        elif mach <= 1.05:
            # Transonic drag rise
            delta_m = mach - m_eff_crit
            raw_cd = self.wave_drag_factor * (delta_m ** 4)
            return min(0.40, raw_cd)
        elif mach <= 1.4:
            # Peak transonic wave drag
            peak_cd = self.wave_drag_factor * ((1.05 - m_eff_crit) ** 4)
            # Tapers slowly towards supersonic wave drag
            return peak_cd * (1.0 - 0.3 * (mach - 1.05))
        else:
            # Supersonic wave drag decay ~ 1 / sqrt(M^2 - 1)
            return 0.04 / math.sqrt(mach ** 2 - 1.0)

    def induced_drag_coefficient(self, cl: float) -> float:
        """
        Calculates induced drag coefficient: CD_i = CL^2 / (pi * AR * e).
        """
        if self.aspect_ratio <= 0 or self.oswald_efficiency <= 0:
            return 0.0
        return (cl ** 2) / (math.pi * self.aspect_ratio * self.oswald_efficiency)

    def drag_coefficient(self, alpha_rad: float, mach: float = 0.0) -> float:
        """
        Total drag coefficient: CD = CD0 + CD_induced + CD_wave.
        """
        cl = self.lift_coefficient(alpha_rad, mach)
        cd_induced = self.induced_drag_coefficient(cl)
        cd_wave = self.wave_drag_coefficient(mach, cl)
        return self.cd0 + cd_induced + cd_wave

    def pitching_moment_coefficient(self, alpha_rad: float, cm0: float = 0.0, cm_alpha: float = -0.5) -> float:
        """
        Pitching moment coefficient Cm = Cm0 + Cm_alpha * alpha.
        Negative Cm_alpha implies longitudinal static stability.
        """
        return cm0 + cm_alpha * alpha_rad
