"""
Unit tests for the Model Context Protocol (MCP) Server.
Verifies JSON-RPC 2.0 tool discovery and execution.
"""

import json
from aero_flight_dynamics.mcp.server import process_message


def test_mcp_initialize():
    msg = {"jsonrpc": "2.0", "id": 1, "method": "initialize"}
    resp = process_message(msg)
    assert resp["id"] == 1
    assert "protocolVersion" in resp["result"]
    assert resp["result"]["serverInfo"]["name"] == "aero-flight-dynamics-mcp"


def test_mcp_tools_list():
    msg = {"jsonrpc": "2.0", "id": 2, "method": "tools/list"}
    resp = process_message(msg)
    tool_names = [t["name"] for t in resp["result"]["tools"]]
    assert "simulate_flight" in tool_names
    assert "query_atmosphere" in tool_names
    assert "evaluate_airfoil" in tool_names
    assert "evaluate_parachute_recovery" in tool_names


def test_mcp_tool_call_query_atmosphere():
    msg = {
        "jsonrpc": "2.0",
        "id": 3,
        "method": "tools/call",
        "params": {"name": "query_atmosphere", "arguments": {"altitude_m": 10000.0}},
    }
    resp = process_message(msg)
    assert "result" in resp
    content = json.loads(resp["result"]["content"][0]["text"])
    assert content["altitude_m"] == 10000.0
    assert content["temperature_k"] < 288.15


def test_mcp_tool_call_evaluate_airfoil():
    msg = {
        "jsonrpc": "2.0",
        "id": 4,
        "method": "tools/call",
        "params": {"name": "evaluate_airfoil", "arguments": {"designation": "0012", "alpha_deg": 0.0}},
    }
    resp = process_message(msg)
    content = json.loads(resp["result"]["content"][0]["text"])
    assert content["zero_lift_angle_deg"] == 0.0
    assert content["cl_at_angle"] == 0.0


def test_mcp_tool_call_evaluate_parachute():
    msg = {
        "jsonrpc": "2.0",
        "id": 5,
        "method": "tools/call",
        "params": {"name": "evaluate_parachute_recovery", "arguments": {"vehicle_mass_kg": 40.0}},
    }
    resp = process_message(msg)
    content = json.loads(resp["result"]["content"][0]["text"])
    assert content["is_safe_landing"] is True
    assert content["touchdown_velocity_m_s"] < 7.5
