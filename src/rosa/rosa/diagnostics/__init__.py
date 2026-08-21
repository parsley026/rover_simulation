#  Copyright (c) 2026. All rights reserved.
#  ROSA Tool Diagnostics Suite — Shared Utilities and Runners.

import sys
import time
from typing import Callable, List, Dict, Any, Tuple

# Terminal coloring constants
RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
RED = "\033[31m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
BLUE = "\033[34m"

class DiagnosticResult:
    def __init__(self, name: str, passed: bool, message: str, elapsed_ms: float):
        self.name = name
        self.passed = passed
        self.message = message
        self.elapsed_ms = elapsed_ms

class DiagnosticSuite:
    def __init__(self, module_name: str, description: str):
        self.module_name = module_name
        self.description = description
        self.tests: List[Tuple[str, Callable[[], Any]]] = []
        self.results: List[DiagnosticResult] = []

    def add_test(self, test_name: str, test_func: Callable[[], Any]):
        """Register a diagnostic check to the suite."""
        self.tests.append((test_name, test_func))

    def run_tests(self, verbose: bool = True) -> bool:
        """Run all registered diagnostic tests in this suite."""
        if verbose:
            print(f"\n{BOLD}{CYAN}=== [{self.module_name}] Diagnostics: {self.description} ==={RESET}")
        
        suite_passed = True
        self.results.clear()

        for name, func in self.tests:
            start_time = time.time()
            try:
                result_data = func()
                elapsed = (time.time() - start_time) * 1000
                msg = str(result_data) if result_data is not None else "OK"
                res = DiagnosticResult(name, True, msg, elapsed)
                if verbose:
                    print(f"  {GREEN}[PASS]{RESET} {name.ljust(35)} ({elapsed:.1f}ms) -> {msg}")
            except Exception as e:
                elapsed = (time.time() - start_time) * 1000
                res = DiagnosticResult(name, False, str(e), elapsed)
                suite_passed = False
                if verbose:
                    print(f"  {RED}[FAIL]{RESET} {name.ljust(35)} ({elapsed:.1f}ms) -> {RED}{e}{RESET}")
            self.results.append(res)
            
        return suite_passed

    def summary(self) -> Tuple[int, int]:
        """Return (passed_count, total_count)."""
        passed = sum(1 for r in self.results if r.passed)
        return passed, len(self.results)
