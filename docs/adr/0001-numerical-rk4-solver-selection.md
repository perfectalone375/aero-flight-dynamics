# ADR 0001: Selection of 4th-Order Runge-Kutta (RK4) for Flight Dynamics Simulation

## Status
Accepted

## Context
Suborbital sounding rocket flight dynamics require integrating continuous non-linear ordinary differential equations (ODEs) across atmospheric layers from sea level up to 100+ km altitude:
- Variable mass depletion $\dot{m}(t)$
- Dynamic pressure $q = \frac{1}{2} \rho v^2$ spanning subsonic, transonic, and supersonic regimes
- Altitude-varying gravitational acceleration $g(z) = g_0 \left(\frac{R_E}{R_E + z}\right)^2$
- Discrete staging events and parachute drag deployment

We considered three integration strategies:
1. **Explicit Forward Euler (1st order):** $O(\Delta t)$ global error. Prone to numerical divergence during rapid transonic acceleration ($M \approx 0.8 - 1.2$) unless $\Delta t < 0.001\text{s}$, incurring unacceptable computational cost.
2. **Runge-Kutta 4th-Order (RK4):** $O(\Delta t^4)$ global error. Provides high accuracy at moderate time-steps ($\Delta t = 0.05\text{s}$), deterministic memory allocation, and robust stability across non-linear aerodynamic discontinuities.
3. **Adaptive-Step Dormand-Prince (RK45 / SciPy `solve_ivp`):** Variable step size improves efficiency in ballistic coast, but creates non-uniform time-series for Model Context Protocol (MCP) telemetry streaming and fixed-rate visualizer gauges.

## Decision
We implemented a self-contained, fixed-step **Runge-Kutta 4th-Order (RK4)** numerical integrator as the core trajectory solver.

$$\begin{aligned}
k_1 &= f(t_n, y_n) \\
k_2 &= f\left(t_n + \frac{\Delta t}{2}, y_n + \frac{\Delta t}{2} k_1\right) \\
k_3 &= f\left(t_n + \frac{\Delta t}{2}, y_n + \frac{\Delta t}{2} k_2\right) \\
k_4 &= f(t_n + \Delta t, y_n + \Delta t k_3) \\
y_{n+1} &= y_n + \frac{\Delta t}{6} (k_1 + 2k_2 + 2k_3 + k_4)
\end{aligned}$$

## Consequences
- **Positive:** Deterministic execution time, zero external runtime solver dependencies (pure NumPy/Python), and exact phase space energy conservation during high-speed atmospheric flight.
- **Positive:** Uniform sampling cadence ideal for real-time telemetry streaming and downstream MCP tool serialization.
- **Negative:** During long vacuum orbital coast periods, fixed time-stepping executes redundant evaluations compared to adaptive integrators.
