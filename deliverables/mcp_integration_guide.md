# Model Context Protocol (MCP) Integration Guide

`aero-flight-dynamics` includes a native **Model Context Protocol (MCP)** server that allows AI coding assistants—including **Claude Desktop**, **Cursor**, **Windsurf**, and **Antigravity**—to invoke aerospace flight dynamics tools directly in real time.

---

## 🛠️ Tools Exposed via MCP

1. **`simulate_flight`**
   * Executes a 3-DoF Runge-Kutta 4th order numerical trajectory simulation.
   * Parameters: `mission_name`, `launch_elevation_deg`, `dry_mass_kg`, `propellant_mass_kg`, `burn_time_s`, `isp_sea_level_s`, `cd0`.
   * Returns: Apogee altitude, top Mach number, Max-Q dynamic pressure, axial G-load, burnout velocity, and flight duration.

2. **`query_atmosphere`**
   * Evaluates the 1976 U.S. Standard Atmosphere model from sea level to 86 km.
   * Parameters: `altitude_m`.
   * Returns: Thermodynamic temperature (K and °C), hydrostatic pressure (Pa), air density ($\text{kg/m}^3$), speed of sound (m/s), and Sutherland dynamic viscosity.

3. **`evaluate_airfoil`**
   * Solves thin-airfoil theory and geometry for any NACA 4-digit airfoil profile (e.g. 0012, 2412, 4415).
   * Parameters: `designation`, `alpha_deg`, `chord_m`.
   * Returns: Max camber, max thickness, zero-lift angle ($\alpha_0$), and theoretical lift coefficient ($C_L$).

4. **`evaluate_parachute_recovery`**
   * Calculates dual-deployment recovery aerodynamics and ground touchdown terminal velocity.
   * Parameters: `vehicle_mass_kg`, `main_diameter_m`, `drogue_diameter_m`.
   * Returns: Touchdown speed (m/s and km/h), safety validation status, and combined canopy drag area.

---

## 💻 Claude Desktop Configuration

Add the following block to your `claude_desktop_config.json`:

### macOS / Linux
```json
{
  "mcpServers": {
    "aero-flight-dynamics": {
      "command": "python",
      "args": ["-m", "aero_flight_dynamics.mcp.server"]
    }
  }
}
```

### Windows
```json
{
  "mcpServers": {
    "aero-flight-dynamics": {
      "command": "python.exe",
      "args": ["-m", "aero_flight_dynamics.mcp.server"],
      "env": {
        "PYTHONPATH": "F:/askable/src"
      }
    }
  }
}
```

---

## 🧪 Testing MCP Server Directly via CLI

You can test the MCP server over standard input/output using JSON-RPC:

```bash
# Query available tools
echo '{"jsonrpc": "2.0", "id": 1, "method": "tools/list"}' | python -m aero_flight_dynamics.mcp.server

# Query atmosphere at 10,000 meters
echo '{"jsonrpc": "2.0", "id": 2, "method": "tools/call", "params": {"name": "query_atmosphere", "arguments": {"altitude_m": 10000}}}' | python -m aero_flight_dynamics.mcp.server
```
