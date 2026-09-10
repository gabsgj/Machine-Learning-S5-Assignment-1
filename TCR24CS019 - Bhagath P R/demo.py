#!/usr/bin/env python3
"""
Interactive & Visual Demonstration for Safe Semantic Planner (PCCST503 Assignment 1).
Demonstrates all 6 Illustrative Test Cases and dynamic incremental replanning with LPA*, D* Lite, A*, and Parallel Search.
"""

import sys
import os

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "src"))

from semantic_planner.models import State, Transition, PlanningProblem, ObjectiveWeights
from semantic_planner.lpa_star import LPAStarPlanner
from semantic_planner.d_star_lite import DStarLitePlanner
from semantic_planner.a_star import AStarPlanner
from semantic_planner.parallel_planner import ParallelBidirectionalPlanner
from tabulate import tabulate


def print_banner(title: str):
    print("\n" + "=" * 78)
    print(f"  {title.upper()}")
    print("=" * 78)


def print_result_table(results: dict):
    headers = ["Planner", "Success", "Path Length", "Total Cost", "Min Bad Dist", "Explored", "Time (ms)", "Memory (KB)"]
    rows = []
    for name, r in results.items():
        rows.append([
            name,
            "YES" if r.success else "NO",
            len(r.state_path),
            f"{r.total_cost:.3f}",
            f"{r.min_safety_distance:.3f}",
            r.explored_states,
            f"{r.planning_time_ms:.3f}",
            f"{r.memory_kb:.2f}"
        ])
    print(tabulate(rows, headers=headers, tablefmt="grid"))


def demo_test_case_1():
    print_banner("Test Case 1: Basic Reachability (S -> A -> B -> G)")
    states = [
        State(id=1, embedding=[0.0, 0.0], name="S"),
        State(id=2, embedding=[1.0, 0.0], name="A"),
        State(id=3, embedding=[2.0, 0.0], name="B"),
        State(id=4, embedding=[3.0, 0.0], name="G"),
    ]
    transitions = [
        Transition(id=101, from_state=1, to_state=2, cost=1.0),
        Transition(id=102, from_state=2, to_state=3, cost=1.0),
        Transition(id=103, from_state=3, to_state=4, cost=1.0),
    ]
    problem = PlanningProblem(1, 4, [], states, transitions)

    results = {}
    for name, cls in [("LPA*", LPAStarPlanner), ("D* Lite", DStarLitePlanner), ("A* Baseline", AStarPlanner), ("Parallel Bidirectional", ParallelBidirectionalPlanner)]:
        planner = cls()
        results[name] = planner.plan(problem)

    print_result_table(results)
    print(f"Selected Path: {' -> '.join(map(str, results['LPA*'].state_path))}")


def demo_test_case_2():
    print_banner("Test Case 2: Bad State Avoidance (S -> A -> X -> G vs S -> C -> D -> G)")
    states = [
        State(id=1, embedding=[0.0, 0.0], name="S"),
        State(id=2, embedding=[1.0, 1.0], name="A"),
        State(id=3, embedding=[2.0, 1.0], name="X (BAD)"),
        State(id=4, embedding=[1.0, -1.0], name="C"),
        State(id=5, embedding=[2.0, -1.0], name="D"),
        State(id=6, embedding=[3.0, 0.0], name="G"),
    ]
    transitions = [
        Transition(id=201, from_state=1, to_state=2, cost=1.0),
        Transition(id=202, from_state=2, to_state=3, cost=1.0),
        Transition(id=203, from_state=3, to_state=6, cost=1.0),
        Transition(id=204, from_state=1, to_state=4, cost=1.5),
        Transition(id=205, from_state=4, to_state=5, cost=1.5),
        Transition(id=206, from_state=5, to_state=6, cost=1.5),
    ]
    problem = PlanningProblem(1, 6, [3], states, transitions)

    results = {}
    for name, cls in [("LPA*", LPAStarPlanner), ("D* Lite", DStarLitePlanner), ("A* Baseline", AStarPlanner), ("Parallel Bidirectional", ParallelBidirectionalPlanner)]:
        planner = cls()
        results[name] = planner.plan(problem)

    print_result_table(results)
    print(f"Safe Path Selected: {' -> '.join(map(str, results['LPA*'].state_path))} (Avoided Bad State 3)")


def demo_test_case_3():
    print_banner("Test Case 3: Safety Margin Pareto Balancing")
    states = [
        State(id=1, embedding=[0.0, 2.0], name="S"),
        State(id=2, embedding=[1.0, 0.2], name="M1 (Near Obstacle)"),
        State(id=3, embedding=[1.0, 3.5], name="M2 (High Clearance)"),
        State(id=4, embedding=[2.0, 2.0], name="G"),
        State(id=99, embedding=[1.0, 0.0], name="Obstacle"),
    ]
    transitions = [
        Transition(id=301, from_state=1, to_state=2, cost=1.0),
        Transition(id=302, from_state=2, to_state=4, cost=1.0),
        Transition(id=303, from_state=1, to_state=3, cost=2.0),
        Transition(id=304, from_state=3, to_state=4, cost=2.0),
    ]

    # Mode 1: High Safety
    w_safe = ObjectiveWeights(beta=1.0, gamma=5.0, safety_margin=1.5, safety_penalty_coeff=20.0)
    p_safe = LPAStarPlanner().plan(PlanningProblem(1, 4, [99], states, transitions, w_safe))

    # Mode 2: Cost-Only
    w_cost = ObjectiveWeights(beta=1.0, gamma=0.0, safety_margin=0.0, safety_penalty_coeff=0.0)
    p_cost = LPAStarPlanner().plan(PlanningProblem(1, 4, [99], states, transitions, w_cost))

    results = {
        "Safety-Conscious (gamma=5.0, margin=1.5)": p_safe,
        "Cost-Only (gamma=0.0, margin=0.0)": p_cost,
    }
    print_result_table(results)
    print(f"Safety Mode Path: {' -> '.join(map(str, p_safe.state_path))} | Min Clearance: {p_safe.min_safety_distance:.2f}")
    print(f"Cost Mode Path:   {' -> '.join(map(str, p_cost.state_path))} | Min Clearance: {p_cost.min_safety_distance:.2f}")


def demo_test_case_4():
    print_banner("Test Case 4: Dynamic Transition Failure & Incremental Replanning")
    states = [
        State(id=1, embedding=[0.0, 0.0], name="S"),
        State(id=2, embedding=[1.0, 0.0], name="A"),
        State(id=3, embedding=[0.0, 1.0], name="B"),
        State(id=4, embedding=[1.0, 1.0], name="C"),
        State(id=5, embedding=[2.0, 0.0], name="G"),
    ]
    transitions = [
        Transition(id=401, from_state=1, to_state=2, cost=1.0),
        Transition(id=402, from_state=2, to_state=5, cost=1.0),
        Transition(id=403, from_state=1, to_state=3, cost=1.5),
        Transition(id=404, from_state=3, to_state=4, cost=1.5),
        Transition(id=405, from_state=4, to_state=5, cost=1.5),
    ]
    problem = PlanningProblem(1, 5, [], states, transitions)

    planner = LPAStarPlanner()
    r_initial = planner.plan(problem)
    print("1. Initial Plan (before failure):")
    print(f"   Path: {' -> '.join(map(str, r_initial.state_path))} | Cost: {r_initial.total_cost:.2f} | Time: {r_initial.planning_time_ms:.3f} ms")

    print("\n   [!] EVENT: Edge (2, 5) failed and became unavailable!")
    planner.update_edge(2, 5, available=False)
    r_replan = planner.replan()
    print("2. Incremental Replan (LPA*):")
    print(f"   Path: {' -> '.join(map(str, r_replan.state_path))} | Cost: {r_replan.total_cost:.2f} | Replan Time: {r_replan.planning_time_ms:.3f} ms | Explored in Replan: {r_replan.explored_states}")


def demo_test_case_5():
    print_banner("Test Case 5: Goal Migration (Goal Update during Execution)")
    states = [
        State(id=1, embedding=[0.0, 0.0], name="S"),
        State(id=2, embedding=[1.0, 0.0], name="A"),
        State(id=3, embedding=[2.0, 1.0], name="G1"),
        State(id=4, embedding=[2.0, -1.0], name="G2"),
    ]
    transitions = [
        Transition(id=501, from_state=1, to_state=2, cost=1.0),
        Transition(id=502, from_state=2, to_state=3, cost=1.0),
        Transition(id=503, from_state=2, to_state=4, cost=1.0),
    ]
    problem = PlanningProblem(1, 3, [], states, transitions)

    planner = LPAStarPlanner()
    r1 = planner.plan(problem)
    print(f"1. Initial Goal = G1 (3): Path = {' -> '.join(map(str, r1.state_path))}")

    print("\n   [!] EVENT: Goal updated to G2 (4)!")
    planner.update_goal(4)
    r2 = planner.replan()
    print(f"2. Incremental Replan: Path = {' -> '.join(map(str, r2.state_path))} | Time: {r2.planning_time_ms:.3f} ms")


def demo_test_case_6():
    print_banner("Test Case 6: Dynamic Shortcut Insertion (Transition Addition)")
    states = [
        State(id=1, embedding=[0.0, 0.0], name="S"),
        State(id=2, embedding=[1.0, 0.0], name="A"),
        State(id=3, embedding=[2.0, 0.0], name="B"),
        State(id=4, embedding=[3.0, 0.0], name="G"),
    ]
    transitions = [
        Transition(id=601, from_state=1, to_state=2, cost=2.0),
        Transition(id=602, from_state=2, to_state=3, cost=2.0),
        Transition(id=603, from_state=3, to_state=4, cost=2.0),
    ]
    problem = PlanningProblem(1, 4, [], states, transitions)

    planner = LPAStarPlanner()
    r1 = planner.plan(problem)
    print(f"1. Initial Path: {' -> '.join(map(str, r1.state_path))} | Cost = {r1.total_cost:.2f}")

    print("\n   [!] EVENT: Added new direct shortcut transition (1 -> 4) with cost 2.5!")
    shortcut = Transition(id=699, from_state=1, to_state=4, cost=2.5)
    planner.graph.add_transition(shortcut)
    planner.update_edge(1, 4, new_cost=2.5, available=True)

    r2 = planner.replan()
    print(f"2. Incremental Replan: Path = {' -> '.join(map(str, r2.state_path))} | Cost = {r2.total_cost:.2f} | Time: {r2.planning_time_ms:.3f} ms")


def main():
    print("\n" + "#" * 78)
    print("#  SAFE SEMANTIC PLANNER IN FINITE CARTESIAN STATE SPACE")
    print("#  PCCST503 Machine Learning - Assignment 1 Demonstration")
    print("#" * 78)

    demo_test_case_1()
    demo_test_case_2()
    demo_test_case_3()
    demo_test_case_4()
    demo_test_case_5()
    demo_test_case_6()

    print("\n" + "=" * 78)
    print("  DEMONSTRATION COMPLETED SUCCESSFULLY")
    print("=" * 78 + "\n")


if __name__ == "__main__":
    main()
