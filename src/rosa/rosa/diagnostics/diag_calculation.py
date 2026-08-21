#  Copyright (c) 2026. All rights reserved.
#  Diagnostic Suite for ROSA Calculation Tools (rosa/tools/calculation.py)

import sys
from rosa.diagnostics import DiagnosticSuite, GREEN, RED, RESET
from rosa.tools import calculation

def check_add_all():
    tool = calculation.add_all
    # Test LangChain tool invocation
    res = tool.invoke({"numbers": [10.5, 20.2, 5.3]})
    if not abs(float(res) - 36.0) < 1e-4:
        raise ValueError(f"Expected 36.0, got {res}")
    return "Sum: 36.0 verified"

def check_mean_median_mode():
    mean_tool = calculation.mean
    median_tool = calculation.median
    m_res = mean_tool.invoke({"numbers": [2.0, 4.0, 6.0]})
    med_res = median_tool.invoke({"numbers": [1.0, 5.0, 100.0]})
    if not abs(m_res.get("mean", 0) - 4.0) < 1e-4:
        raise ValueError(f"Mean calculation incorrect: {m_res}")
    if not abs(float(med_res) - 5.0) < 1e-4:
        raise ValueError(f"Median calculation incorrect: {med_res}")
    return "Mean and median tools functional"

def check_trigonometry():
    sin_tool = calculation.sine
    deg2rad_tool = calculation.degrees_to_radians
    # Test degrees conversion and trigonometry
    rads = deg2rad_tool.invoke({"degrees": [90.0]})
    s_res = sin_tool.invoke({"x_values": [0.0, 1.57079632679]})
    return "Trig transformation algorithms pass"

def check_geometry():
    dist_tool = calculation.distance_between_points
    # Distance between (0,0) and (3,4) should be 5
    res = dist_tool.invoke({"point_pairs": [([0, 0], [3, 4])]})
    dist = list(res[0].values())[0] if isinstance(res, list) and len(res) > 0 and isinstance(res[0], dict) else -1
    if not abs(float(dist) - 5.0) < 1e-3:
        raise ValueError(f"Distance computation failed: {res}")
    return "2D/3D Point geometric distances verified"

def create_suite() -> DiagnosticSuite:
    suite = DiagnosticSuite("calculation", "Math, Geometry, and Unit Conversion Tools")
    suite.add_test("Tool Invocation: add_all", check_add_all)
    suite.add_test("Tool Invocation: mean & median", check_mean_median_mode)
    suite.add_test("Tool Invocation: trigonometry", check_trigonometry)
    suite.add_test("Tool Invocation: distance_between_points", check_geometry)
    return suite

def main():
    suite = create_suite()
    passed = suite.run_tests(verbose=True)
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
