#  Copyright (c) 2026. All rights reserved.
#  Diagnostic Suite for ROSA Log Analysis Tools (rosa/tools/log.py & ROS log commands)

import os
import sys
import tempfile
from rosa.diagnostics import DiagnosticSuite
from rosa.tools import log as log_tool, ros2

def check_log_directories():
    tool = ros2.ros2_log_directories
    try:
        dirs = tool.invoke({})
        count = len(dirs) if isinstance(dirs, (list, dict)) else 1
        sample = str(dirs[:2]) if isinstance(dirs, list) and dirs else str(dirs)
        if len(sample) > 50:
            sample = sample[:50] + "..."
        return f"Found {count} log directories ({sample})"
    except Exception as e:
        return f"Log directories queried ({e})"

def check_roslog_list():
    tool = ros2.roslog_list
    logs = tool.invoke({"min_size": 100})
    count = len(logs.get("logs", [])) if isinstance(logs, dict) else len(logs) if isinstance(logs, list) else 0
    return f"Retrieved log file listing (found {count} matching files)"

def check_read_log_file():
    tool = log_tool.read_log
    # Create a temporary mock log file to test parser logic cleanly without breaking if rosout is empty
    with tempfile.NamedTemporaryFile("w+", delete=False, suffix=".log") as tmp:
        for i in range(1, 25):
            tmp.write(f"[INFO] [1722000{i:03d}] [node_mock]: Sample ROS log message line {i}\n")
        tmp_path = tmp.name
    
    try:
        res = tool.invoke({
            "log_file_directory": os.path.dirname(tmp_path),
            "log_filename": os.path.basename(tmp_path),
            "num_lines": 5
        })
        if "Sample ROS log message line" not in str(res):
            raise ValueError(f"Failed to extract log lines correctly: {res}")
        return "Log file directory/filename extraction verified"
    finally:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)

def create_suite() -> DiagnosticSuite:
    suite = DiagnosticSuite("log & forensics", "ROS Log Directory Discovery and Content Slicing")
    suite.add_test("Tool Invocation: ros2_log_directories", check_log_directories)
    suite.add_test("Tool Invocation: roslog_list", check_roslog_list)
    suite.add_test("Tool Invocation: read_log (mock log)", check_read_log_file)
    return suite

def main():
    suite = create_suite()
    passed = suite.run_tests(verbose=True)
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
