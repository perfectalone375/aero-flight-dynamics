"""
Example: NACA 4-Digit Airfoil Geometry and Aerodynamic Polar Generation.
"""

import math
from aero_flight_dynamics import NACA4Airfoil, AeroCoefficients


def analyze_airfoils():
    print("=== NACA Airfoil Comparative Aerodynamic Analysis ===")

    airfoils = [
        ("NACA 0012 (Symmetric)", NACA4Airfoil("0012")),
        ("NACA 2412 (Moderate Camber)", NACA4Airfoil("2412")),
        ("NACA 4415 (High Lift / Thick)", NACA4Airfoil("4415")),
    ]

    aero = AeroCoefficients()

    print(f"{'Airfoil':<30} | {'Max Camber':<12} | {'Thickness':<10} | {'Alpha_0 (deg)':<14} | {'CL (alpha=5deg)':<15}")
    print("-" * 90)

    for name, foil in airfoils:
        alpha0_deg = math.degrees(foil.zero_lift_angle_rad())
        cl_5deg = foil.theoretical_lift_coefficient(math.radians(5.0))
        print(
            f"{name:<30} | {foil.max_camber_m * 100:>10.1f}% | {foil.thickness_ratio_t * 100:>8.1f}% | "
            f"{alpha0_deg:>12.2f} | {cl_5deg:>15.4f}"
        )


if __name__ == "__main__":
    analyze_airfoils()
