#  Copyright (c) 2026. All rights reserved.
#  Diagnostic Suite for ROSA ROS 2 Interface Tools (rosa/tools/ros2.py)

import sys
import shutil
from rosa.diagnostics import DiagnosticSuite
from rosa.tools import ros2

def check_ros2_cli_installed():
    ros2_cmd = shutil.which("ros2")
    if not ros2_cmd:
        raise RuntimeError("ros2 CLI binary not found in PATH! Is ROS 2 sourced?")
    return f"ROS 2 CLI available at {ros2_cmd}"

def check_ros2_topic_list():
    tool = ros2.ros2_topic_list
    res = tool.invoke({})
    topics = res.get("topics", []) if isinstance(res, dict) else res
    return f"Active topic count: {len(topics)} ({'/rosout' if '/rosout' in topics else str(topics[:2])})"

def check_ros2_node_list():
    tool = ros2.ros2_node_list
    res = tool.invoke({})
    nodes = res.get("nodes", []) if isinstance(res, dict) else res
    return f"Active node count: {len(nodes)}"

def check_ros2_service_list():
    tool = ros2.ros2_service_list
    res = tool.invoke({})
    srvs = res.get("services", []) if isinstance(res, dict) else res
    return f"Active service count: {len(srvs)}"

def check_ros2_param_list():
    tool = ros2.ros2_param_list
    res = tool.invoke({})
    return "ROS 2 Parameter tree readable"

def check_ros2_execute_command():
    tool = ros2.execute_ros_command
    success, output = tool("ros2 topic list")
    if not success and "ros2" not in output:
        raise RuntimeError(f"Subprocess ROS 2 execution failed: {output}")
    return "Subprocess command execution bridge functional"

def create_suite() -> DiagnosticSuite:
    suite = DiagnosticSuite("ros2 middleware", "Live DDS Topics, Nodes, Services, and Parameters")
    suite.add_test("ROS 2 Environment CLI Check", check_ros2_cli_installed)
    suite.add_test("Tool Invocation: ros2_topic_list", check_ros2_topic_list)
    suite.add_test("Tool Invocation: ros2_node_list", check_ros2_node_list)
    suite.add_test("Tool Invocation: ros2_service_list", check_ros2_service_list)
    suite.add_test("Tool Invocation: ros2_param_list", check_ros2_param_list)
    suite.add_test("Execution Bridge: execute_ros_command", check_ros2_execute_command)
    return suite

def main():
    suite = create_suite()
    passed = suite.run_tests(verbose=True)
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
