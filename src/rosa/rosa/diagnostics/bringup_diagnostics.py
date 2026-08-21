#!/usr/bin/env python3
#  Copyright (c) 2026. All rights reserved.
#  Master Bringup & Verification Runner for ROSA Tool Diagnostics.
#  Systematically executes all modular tool diagnostic scripts and formats an executive health scorecard.

import sys
import time
import argparse
from typing import List, Dict

from rosa.diagnostics import DiagnosticSuite, BOLD, RESET, GREEN, RED, YELLOW, CYAN, BLUE
from rosa.diagnostics import diag_system, diag_calculation, diag_ros2, diag_log

def get_all_suites() -> Dict[str, DiagnosticSuite]:
    return {
        "system": diag_system.create_suite(),
        "calculation": diag_calculation.create_suite(),
        "ros2": diag_ros2.create_suite(),
        "log": diag_log.create_suite(),
    }

def print_banner():
    print(f"\n{BOLD}{CYAN}========================================================================={RESET}")
    print(f"{BOLD}{CYAN}      ROSA (Robot Operating System Agent) — BRINGUP TOOL DIAGNOSTICS     {RESET}")
    print(f"{BOLD}{CYAN}========================================================================={RESET}")
    print("Verifying environment, LLM connection readiness, and LangChain v1.x tools...")

def print_scorecard(suites: List[DiagnosticSuite], total_elapsed: float):
    print(f"\n{BOLD}{BLUE}--- EXECUTIVE DIAGNOSTIC SCORECARD ---{RESET}")
    print(f"{'MODULE NAME'.ljust(18)} | {'TESTS PASSED'.ljust(15)} | {'STATUS'.ljust(12)} | DESCRIPTION")
    print("-" * 75)
    
    total_passed = 0
    total_tests = 0
    all_ok = True

    for suite in suites:
        passed, count = suite.summary()
        total_passed += passed
        total_tests += count
        status_str = f"{GREEN}HEALTHY{RESET}" if passed == count else f"{RED}ATTENTION{RESET}"
        if passed != count:
            all_ok = False
        
        ratio = f"{passed}/{count}"
        print(f"{suite.module_name.ljust(18)} | {ratio.ljust(15)} | {status_str.ljust(21)} | {suite.description}")
    
    print("-" * 75)
    print(f"Total Execution Time: {total_elapsed:.2f} seconds | Cumulative Status: ", end="")
    if all_ok and total_tests > 0:
        print(f"{BOLD}{GREEN}[ ALL SYSTEM TOOLS 100% READY FOR DEPLOYMENT ]{RESET}\n")
    else:
        print(f"{BOLD}{RED}[ ISSUES DETECTED - CHECK LOGS ABOVE ]{RESET}\n")
    return all_ok

def main():
    parser = argparse.ArgumentParser(description="ROSA Master Bringup Diagnostics Runner")
    parser.add_argument("--only", choices=["system", "calculation", "ros2", "log"], help="Run tests only for specified tool module")
    parser.add_argument("--quiet", "-q", action="store_true", help="Suppress individual test logs and only display scorecard")
    args, unknown = parser.parse_known_args()

    print_banner()
    start_time = time.time()
    
    suites_map = get_all_suites()
    target_suites = [suites_map[args.only]] if args.only else list(suites_map.values())
    
    all_passed = True
    for suite in target_suites:
        if not suite.run_tests(verbose=not args.quiet):
            all_passed = False
            
    total_elapsed = time.time() - start_time
    scorecard_ok = print_scorecard(target_suites, total_elapsed)
    
    sys.exit(0 if scorecard_ok else 1)

if __name__ == "__main__":
    main()
