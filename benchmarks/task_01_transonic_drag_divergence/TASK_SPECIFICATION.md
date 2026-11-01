# AI Model Coding Task: Transonic Drag Divergence Edge Case Fix

> **Task Type:** Software Engineering Benchmark / RLHF Evaluation  
> **Target Subsystem:** `aero_flight_dynamics.aerodynamics.coefficients`  
> **Repository:** `aero-flight-dynamics`  
> **Author:** Gopi Naga Vignesh Thirupati  

---

## 1. Problem Description (AI Prompt)

### Issue Title
`[BUG] Transonic wave drag coefficient experiences discontinuity when angle of attack creates asymmetrical lift in transonic corridor`

### Issue Description
In `AeroCoefficients.wave_drag_coefficient()`, the effective critical Mach number is currently adjusted via the Korn equation approximation:
```python
m_eff_crit = self.mach_critical - 0.1 * abs(cl)
```
However, during rapid pitch maneuvers where the vehicle enters high-g pull-ups at transonic speeds ($M \approx 0.95$), when $|C_L| > 1.2$, $M_{\text{eff\_crit}}$ drops excessively, causing an unphysical divergence in wave drag ($C_{D,\text{wave}} > 1.5$) that crashes the Runge-Kutta numerical integration step with a floating point overflow.

### Expected Behavior
1. Implement a non-linear saturation clamp on the effective critical Mach reduction: $\Delta M_{\text{crit}} = \min(0.12, 0.1 \cdot |C_L|)$.
2. Cap peak transonic wave drag to prevent numerical divergence while preserving physical transonic drag rise.
3. Ensure all regression unit tests pass without integration blowup.

---

## 2. Test Verification Specification
- **Test File:** `tests/test_wave_drag_regression.py`
- **Fail-to-Pass Criterion:**
  - Before the fix: `pytest tests/test_wave_drag_regression.py` **FAILS** with `AssertionError: Wave drag exceeded physical stability ceiling`.
  - After the fix: `pytest tests/test_wave_drag_regression.py` **PASSES**.
- **Pass-to-Pass Criterion:**
  - All existing 21 test cases in `tests/` must remain passing.

---

## 3. Ground Truth Solution
See `solution_patch.diff` for the reference patch that resolves the issue and passes all tests.
