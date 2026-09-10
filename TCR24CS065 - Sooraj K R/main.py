# main.py
# Runs all 6 test cases and prints the results with evaluation metrics.
# Also demonstrates dynamic replanning for test cases 4, 5, and 6.

from structures import Transition
from planner import SafePlanner, replan_with_unavailable_transition, replan_with_new_goal, replan_with_new_transition
from test_cases import (
    create_test_case_1,
    create_test_case_2,
    create_test_case_3,
    create_test_case_4,
    create_test_case_5,
    create_test_case_6,
)


def print_result(result):
    """Print the planning result in a nice format."""
    print(f"  Success:           {result.success}")
    print(f"  State Path:        {result.statePath}")
    print(f"  Transition Path:   {result.transitionPath}")
    print(f"  Total Cost:        {result.totalCost}")
    print(f"  Safety Score:      {result.safetyScore}  (min distance to bad states)")
    print(f"  Reliability:       {result.reliabilityScore}")
    print(f"  Objective Score:   {result.score}")
    print(f"  Explored States:   {result.exploredStates}")
    print(f"  Planning Time:     {result.planningTime} seconds")
    print()


def print_separator(title):
    """Print a nice section header."""
    print("=" * 60)
    print(f"  {title}")
    print("=" * 60)


# Create our planner (same instance for all tests)
planner = SafePlanner()


# ============================================================
# Test Case 1: Basic Reachability
# ============================================================
print_separator("TEST CASE 1: Basic Reachability")
print("Graph: S -> A -> B -> G")
print("Expected: Path [0, 1, 2, 3], Cost = 3.0")
print()
problem1 = create_test_case_1()
result1 = planner.plan(problem1)
print_result(result1)


# ============================================================
# Test Case 2: Bad State Avoidance
# ============================================================
print_separator("TEST CASE 2: Bad State Avoidance")
print("Path 1: S -> A -> X -> G  (X is BAD)")
print("Path 2: S -> C -> D -> G  (safe)")
print("Expected: Planner picks Path 2")
print()
problem2 = create_test_case_2()
result2 = planner.plan(problem2)
print_result(result2)
# Check that no bad states were visited
bad_visited = [s for s in result2.statePath if s in problem2.badStates]
print(f"  Bad states visited: {len(bad_visited)} (should be 0)")
print()


# ============================================================
# Test Case 3: Safety Margin
# ============================================================
print_separator("TEST CASE 3: Safety Margin (Cost vs Safety)")
print("Path 1: S -> A -> B -> G  (cheap but CLOSE to bad state)")
print("Path 2: S -> C -> D -> G  (expensive but FAR from bad state)")
print("Note: A* picks the cheapest path. The safety score shows")
print("      how close that path gets to bad states.")
print()
problem3 = create_test_case_3()
result3 = planner.plan(problem3)
print_result(result3)
print("  Analysis: Path 1 is cheaper (cost=3.0) but safety distance")
print("  is small because it passes near the bad state at [1.0, 1.0].")
print("  Path 2 costs more (cost=6.0) but stays much farther away.")
print("  The objective score accounts for both cost and safety distance.")
print()


# ============================================================
# Test Case 4: Dynamic Transition (Replanning)
# ============================================================
print_separator("TEST CASE 4: Dynamic Transition")
print("--- Initial Plan: S -> A -> G (direct) ---")
print()
problem4 = create_test_case_4()
result4_before = planner.plan(problem4)
print_result(result4_before)

print("--- Transition A -> G becomes UNAVAILABLE! Replanning... ---")
print()
result4_after = replan_with_unavailable_transition(planner, problem4, from_state=1, to_state=4)
print_result(result4_after)
print(f"  Replanning time: {result4_after.planningTime} seconds")
print()


# ============================================================
# Test Case 5: Goal Update (Replanning)
# ============================================================
print_separator("TEST CASE 5: Goal Update")
print("--- Initial Plan: Goal is G1 (state 2) ---")
print()
problem5 = create_test_case_5()
result5_before = planner.plan(problem5)
print_result(result5_before)

print("--- Goal changes to G2 (state 4)! Replanning... ---")
print()
result5_after = replan_with_new_goal(planner, problem5, new_goal=4)
print_result(result5_after)
print(f"  Replanning time: {result5_after.planningTime} seconds")
print()


# ============================================================
# Test Case 6: Transition Addition (Replanning)
# ============================================================
print_separator("TEST CASE 6: Transition Addition")
print("--- Initial Plan: S -> A -> B -> G (long path) ---")
print()
problem6 = create_test_case_6()
result6_before = planner.plan(problem6)
print_result(result6_before)

print("--- New shortcut S -> G added! Replanning... ---")
print()
shortcut = Transition(
    id=63,
    from_state=0,
    to_state=3,
    cost=1.0,          # much cheaper than the long path
    safety=1.0,
    reliability=1.0,
    available=True
)
result6_after = replan_with_new_transition(planner, problem6, shortcut)
print_result(result6_after)
print(f"  Replanning time: {result6_after.planningTime} seconds")
print()


# ============================================================
# Summary
# ============================================================
print_separator("SUMMARY")
print(f"  Test 1 (Basic Reachability):   {'PASS' if result1.success else 'FAIL'}")
print(f"  Test 2 (Bad State Avoidance):  {'PASS' if result2.success and len(bad_visited) == 0 else 'FAIL'}")
print(f"  Test 3 (Safety Margin):        {'PASS' if result3.success else 'FAIL'}")
print(f"  Test 4 (Dynamic Transition):   {'PASS' if result4_after.success else 'FAIL'}")
print(f"  Test 5 (Goal Update):          {'PASS' if result5_after.success else 'FAIL'}")
print(f"  Test 6 (Transition Addition):  {'PASS' if result6_after.success and result6_after.totalCost < result6_before.totalCost else 'FAIL'}")
print()