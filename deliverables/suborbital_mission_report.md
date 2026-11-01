# Suborbital Mission Telemetry Assessment: AeroProbe-Alpha

**Vehicle Mission:** `AeroProbe-Alpha (Suborbital Research Probe)`  
**Simulation Engine:** `aero-flight-dynamics v0.1.0 (RK4 Numerical Solver)`  
**Standard Atmosphere:** `1976 U.S. Standard Atmosphere (NOAA / NASA / USAF)`  

---

## 🚀 Mission Telemetry Summary

| Mission Parameter | Value | Standard Unit | Reference / Limit |
| :--- | :--- | :--- | :--- |
| **Apogee Altitude** | **37.237** | km | 122,168.6 ft (Mesosphere) |
| **Time to Apogee** | 83.3 | seconds | Calculated via vertical $v_z \le 0$ crossing |
| **Maximum Mach Number** | **Mach 4.79** | - | Hypersonic boundary transition |
| **Max Dynamic Pressure (Max-Q)** | 815.59 | kPa | Occurred at $t = 6.2\text{ s}$ during boost phase |
| **Peak Axial Load** | 25.20 | G | Structural threshold compliant |
| **Motor Burnout Altitude** | 5.551 | km | Throttle cutoff |
| **Motor Burnout Velocity** | 1,518.8 | m/s | Burnout kinetic energy peak |
| **Downrange Distance** | 15.268 | km | Downrange drift under 84° elevation |
| **Total Flight Duration** | 187.55 | seconds | Ground impact / recovery initiation |

---

## 🪂 Recovery Subsystem Performance

* **Drogue Deployment:** Triggered at apogee ($t = 83.3\text{ s}$, Altitude: $37.2\text{ km}$) for stabilized descent.
* **Main Canopy Deployment:** Triggered at $z = 450\text{ m}$ descent threshold.
* **Calculated Ground Touchdown Velocity:** **$5.98\text{ m/s}$ ($21.5\text{ km/h}$)** — Certified Safe Touchdown.

---

## 📊 Aerodynamic Verification
* **Subsonic Compressibility:** Prandtl-Glauert amplification $C_L / C_{L0} = 1.25$ verified at Mach 0.60.
* **Transonic Corridor:** Lock/Korn wave drag divergence capped at $C_{D,\text{wave}} \le 0.40$ (Pull Request #1 verified).
* **Atmospheric Consistency:** Hydrostatic pressure verified against NOAA tables within $< 0.1\%$ error.
