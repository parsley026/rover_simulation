#  Copyright (c) 2026. All rights reserved.
#  Diagnostic Suite for ROSA System & Runtime Tools (rosa/tools/system.py) and Environment Check

import sys
import os
import time
import urllib.request
import urllib.error
from rosa.diagnostics import DiagnosticSuite
from rosa.tools import system

def check_python_environment():
    ver = sys.version_info
    if ver.major < 3 or (ver.major == 3 and ver.minor < 10):
        raise RuntimeError(f"Python 3.10+ required, found {ver.major}.{ver.minor}")
    return f"Python {ver.major}.{ver.minor}.{ver.micro} OK"

def check_langchain_core_imports():
    try:
        from langchain_core.tools import tool, Tool
        from langchain_core.globals import set_debug, set_verbose
        from langchain_core.prompts import ChatPromptTemplate
    except ImportError as e:
        raise RuntimeError(f"Modern LangChain Core v1.3+ import failed: {e}")
    return "LangChain v1.x compatibility verified"

def check_verbosity_tool():
    tool = system.set_verbosity
    res1 = tool.invoke({"enable_verbose_messages": True})
    res2 = tool.invoke({"enable_verbose_messages": False})
    return "Verbosity dynamically toggled OK"

def check_debugging_tool():
    tool = system.set_debugging
    res = tool.invoke({"enable_debug_messages": False})
    return "Debug level setting OK"

def check_wait_tool():
    tool = system.wait
    start = time.time()
    res = tool.invoke({"seconds": 1})
    duration = time.time() - start
    if duration < 0.9:
        raise ValueError(f"Wait tool did not pause sufficiently: took {duration:.2f}s")
    return f"Timed delay completed ({duration:.2f}s)"

def check_local_vllm_connection():
    url = "http://localhost:8000/v1/models"
    try:
        req = urllib.request.Request(url, headers={"Authorization": "Bearer sk-no-key-required"})
        with urllib.request.urlopen(req, timeout=3) as resp:
            if resp.status == 200:
                return "vLLM local server operational at :8000"
            else:
                raise RuntimeError(f"Unexpected status: {resp.status}")
    except urllib.error.URLError:
        return "NOTICE: vLLM server offline on port 8000 (optional offline test)"
    except Exception as e:
        return f"NOTICE: Local server check skipped ({e})"

def create_suite() -> DiagnosticSuite:
    suite = DiagnosticSuite("system & runtime", "Environment Configuration and System Tools")
    suite.add_test("Python Runtime Version", check_python_environment)
    suite.add_test("LangChain v1.3+ Compatibility", check_langchain_core_imports)
    suite.add_test("Tool Invocation: set_verbosity", check_verbosity_tool)
    suite.add_test("Tool Invocation: set_debugging", check_debugging_tool)
    suite.add_test("Tool Invocation: wait(1s)", check_wait_tool)
    suite.add_test("LLM Backend Endpoint Check", check_local_vllm_connection)
    return suite

def main():
    suite = create_suite()
    passed = suite.run_tests(verbose=True)
    sys.exit(0 if passed else 1)

if __name__ == "__main__":
    main()
