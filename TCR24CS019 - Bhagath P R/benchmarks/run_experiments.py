#!/usr/bin/env python3
"""
Comprehensive Benchmark & Experimental Evaluation Suite.
Measures performance metrics across grid sizes, obstacle densities, embedding dimensions,
and dynamic replanning speedups (LPA*, D* Lite, A*, Parallel Search).
Outputs Markdown tables and quantitative evaluation data for the Design Report & README.md.
"""

import sys
import os
import random
import time
import json
import numpy as np

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from semantic_planner.models import State, Transition, PlanningProblem, ObjectiveWeights
from semantic_planner.graph import CartesianGraph
from semantic_planner.lpa_star import LPAStarPlanner
from semantic_planner.d_star_lite import DStarLitePlanner
from semantic_planner.a_star import AStarPlanner
from semantic_planner.parallel_planner import ParallelBidirectionalPlanner
from tabulate import tabulate


def generate_grid_problem(
    rows: int,
    cols: int,
    obstacle_ratio: float = 0.12,
    dim: int = 2,
    seed: int = 42
) -> PlanningProblem:
    random.seed(seed)
    np.random.seed(seed)

    states = []
    state_map = {}
    sid = 1

    for r in range(rows):
        for c in range(cols):
            coords = [float(r), float(c)]
            if dim > 2:
                extra = list(np.random.normal(0.0, 0.05, dim - 2))
                coords.extend(extra)
            s = State(id=sid, embedding=coords, name=f"({r},{c})")
            states.append(s)
            state_map[(r, c)] = sid
            sid += 1

    start_id = state_map[(0, 0)]
    goal_id = state_map[(rows - 1, cols - 1)]

    # Keep a guaranteed diagonal corridor open
    corridor = set()
    for i in range(min(rows, cols)):
        corridor.add((i, i))
        if i + 1 < rows:
            corridor.add((i + 1, i))
        if i + 1 < cols:
            corridor.add((i, i + 1))

    all_coords = [rc for rc in state_map.keys() if rc not in corridor]
    num_obstacles = int(len(all_coords) * obstacle_ratio)
    bad_coords = set(random.sample(all_coords, num_obstacles))
    bad_states = [state_map[rc] for rc in bad_coords]

    # Create 4-connected grid transitions
    transitions = []
    tid = 1
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    for (r, c), u_id in state_map.items():
        for dr, dc in directions:
            nr, nc = r + dr, c + dc
            if (nr, nc) in state_map:
                v_id = state_map[(nr, nc)]
                dist = np.linalg.norm(states[u_id - 1].vector - states[v_id - 1].vector)
                transitions.append(
                    Transition(
                        id=tid,
                        from_state=u_id,
                        to_state=v_id,
                        cost=max(1.0, float(dist)),
                        safety=1.0,
                        reliability=round(random.uniform(0.95, 1.0), 3),
                        available=True
                    )
                )
                tid += 1

    return PlanningProblem(
        initial_state=start_id,
        goal_state=goal_id,
        bad_states=bad_states,
        states=states,
        transitions=transitions,
        weights=ObjectiveWeights(alpha=1000.0, beta=1.0, gamma=2.0, delta=1.0, safety_margin=1.5)
    )


def experiment_1_scalability():
    print("\n" + "=" * 80)
    print("EXPERIMENT 1: SCALABILITY ACROSS GRID STATE SPACE SIZES (12% Obstacle Density)")
    print("=" * 80)

    grid_configs = [
        (10, 10, "10x10 (100 states)"),
        (20, 20, "20x20 (400 states)"),
        (30, 30, "30x30 (900 states)"),
        (40, 40, "40x40 (1600 states)"),
        (50, 50, "50x50 (2500 states)"),
    ]

    planners = [
        ("LPA*", LPAStarPlanner),
        ("D* Lite", DStarLitePlanner),
        ("A* (Scratch)", AStarPlanner),
        ("Parallel Search", ParallelBidirectionalPlanner),
    ]

    headers = ["Grid Size", "Planner", "Success", "Cost", "Clearance", "Explored", "Time (ms)", "Memory (KB)"]
    table_rows = []
    data_records = []

    for rows, cols, label in grid_configs:
        prob = generate_grid_problem(rows, cols, obstacle_ratio=0.12, seed=123)
        for name, cls in planners:
            planner = cls()
            res = planner.plan(prob)
            table_rows.append([
                label,
                name,
                "100%" if res.success else "0%",
                f"{res.total_cost:.2f}" if res.success else "N/A",
                f"{res.min_safety_distance:.2f}" if res.success else "0.0",
                res.explored_states,
                f"{res.planning_time_ms:.2f}",
                f"{res.memory_kb:.1f}",
            ])
            data_records.append({
                "size": label,
                "planner": name,
                "success": res.success,
                "cost": res.total_cost,
                "clearance": res.min_safety_distance,
                "explored": res.explored_states,
                "time_ms": res.planning_time_ms,
                "memory_kb": res.memory_kb
            })

    print(tabulate(table_rows, headers=headers, tablefmt="github"))
    return data_records


def experiment_2_dynamic_replanning():
    print("\n" + "=" * 80)
    print("EXPERIMENT 2: INCREMENTAL REPLANNING VS FROM-SCRATCH PLANNING (30x30 Grid, 900 States)")
    print("=" * 80)

    prob = generate_grid_problem(30, 30, obstacle_ratio=0.10, seed=777)

    scenarios = [
        ("1 Edge Disabled (On Path)", 1),
        ("3 Edges Disabled", 3),
        ("5 Edges Disabled", 5),
        ("Dynamic Bad State Inserted", "bad_state"),
        ("Dynamic Shortcut Inserted", "shortcut"),
    ]

    headers = [
        "Dynamic Event",
        "A* Replan (ms)",
        "LPA* Replan (ms)",
        "D* Lite Replan (ms)",
        "LPA* Speedup",
        "D* Lite Speedup",
        "LPA* Nodes",
        "A* Nodes"
    ]
    table_rows = []
    records = []

    for name, event in scenarios:
        prob_step = generate_grid_problem(30, 30, obstacle_ratio=0.10, seed=777)
        lpa_step = LPAStarPlanner()
        dstar_step = DStarLitePlanner()
        astar_step = AStarPlanner()

        lpa_step.plan(prob_step)
        dstar_step.plan(prob_step)
        astar_step.plan(prob_step)

        if isinstance(event, int):
            path = lpa_step.plan(prob_step).state_path
            for i in range(min(event, len(path) - 1)):
                u, v = path[i], path[i + 1]
                lpa_step.update_edge(u, v, available=False)
                dstar_step.update_edge(u, v, available=False)
                astar_step.update_edge(u, v, available=False)
        elif event == "bad_state":
            path = lpa_step.plan(prob_step).state_path
            mid_node = path[len(path) // 2]
            lpa_step.update_bad_states(added=[mid_node])
            dstar_step.update_bad_states(added=[mid_node])
            astar_step.update_bad_states(added=[mid_node])
        elif event == "shortcut":
            path = lpa_step.plan(prob_step).state_path
            n1 = path[len(path) // 4]
            n2 = path[3 * len(path) // 4]
            shortcut = Transition(id=99999, from_state=n1, to_state=n2, cost=1.0, safety=1.0, reliability=1.0)
            for p in [lpa_step, dstar_step, astar_step]:
                p.graph.add_transition(shortcut)
                p.update_edge(n1, n2, new_cost=1.0, available=True)

        res_lpa = lpa_step.replan()
        res_dstar = dstar_step.replan()
        res_astar = astar_step.replan()

        t_astar = max(0.001, res_astar.planning_time_ms)
        t_lpa = max(0.001, res_lpa.planning_time_ms)
        t_dstar = max(0.001, res_dstar.planning_time_ms)

        speedup_lpa = t_astar / t_lpa
        speedup_dstar = t_astar / t_dstar

        table_rows.append([
            name,
            f"{t_astar:.3f}",
            f"{t_lpa:.3f}",
            f"{t_dstar:.3f}",
            f"{speedup_lpa:.1f}x",
            f"{speedup_dstar:.1f}x",
            res_lpa.explored_states,
            res_astar.explored_states
        ])
        records.append({
            "event": name,
            "t_astar": t_astar,
            "t_lpa": t_lpa,
            "t_dstar": t_dstar,
            "speedup_lpa": speedup_lpa,
            "speedup_dstar": speedup_dstar,
            "nodes_lpa": res_lpa.explored_states,
            "nodes_astar": res_astar.explored_states
        })

    print(tabulate(table_rows, headers=headers, tablefmt="github"))
    return records


def experiment_3_high_dimension():
    print("\n" + "=" * 80)
    print("EXPERIMENT 3: EMBEDDING DIMENSIONALITY SCALING (R^d for d in [2..128])")
    print("=" * 80)

    dims = [2, 4, 8, 16, 32, 64, 128]
    headers = ["Dimension d", "Planner", "Success", "Path Cost", "Explored Nodes", "Time (ms)", "Memory (KB)"]
    table_rows = []
    records = []

    for d in dims:
        prob = generate_grid_problem(20, 20, obstacle_ratio=0.10, dim=d, seed=42)
        planner = LPAStarPlanner()
        res = planner.plan(prob)
        table_rows.append([
            f"R^{d}",
            "LPA*",
            "100%" if res.success else "0%",
            f"{res.total_cost:.2f}",
            res.explored_states,
            f"{res.planning_time_ms:.2f}",
            f"{res.memory_kb:.1f}"
        ])
        records.append({
            "dim": d,
            "cost": res.total_cost,
            "explored": res.explored_states,
            "time_ms": res.planning_time_ms,
            "memory_kb": res.memory_kb
        })

    print(tabulate(table_rows, headers=headers, tablefmt="github"))
    return records


def experiment_4_safety_pareto():
    print("\n" + "=" * 80)
    print("EXPERIMENT 4: SAFETY MARGIN THRESHOLD VS PATH COST (PARETO FRONTIER)")
    print("=" * 80)

    margins = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0]
    headers = ["Safety Margin (d_safe)", "Success", "Raw Cost (C)", "Min Clearance (D)", "Composite Score", "Time (ms)"]
    table_rows = []
    records = []

    for m in margins:
        prob = generate_grid_problem(20, 20, obstacle_ratio=0.10, seed=99)
        prob.weights = ObjectiveWeights(alpha=1000.0, beta=1.0, gamma=5.0, delta=1.0, safety_margin=m, safety_penalty_coeff=15.0)
        planner = LPAStarPlanner()
        res = planner.plan(prob)

        table_rows.append([
            f"{m:.1f}",
            "YES" if res.success else "NO",
            f"{res.total_cost:.2f}",
            f"{res.min_safety_distance:.2f}",
            f"{res.composite_score:.2f}",
            f"{res.planning_time_ms:.2f}"
        ])
        records.append({
            "margin": m,
            "cost": res.total_cost,
            "clearance": res.min_safety_distance,
            "score": res.composite_score,
            "time_ms": res.planning_time_ms
        })

    print(tabulate(table_rows, headers=headers, tablefmt="github"))
    return records


def main():
    print("\n" + "#" * 80)
    print("#  AUTOMATED BENCHMARK EVALUATION FOR SAFE SEMANTIC CARTESIAN PLANNER")
    print("#" * 80)

    exp1 = experiment_1_scalability()
    exp2 = experiment_2_dynamic_replanning()
    exp3 = experiment_3_high_dimension()
    exp4 = experiment_4_safety_pareto()

    dump_path = os.path.join(os.path.dirname(__file__), "benchmark_results.json")
    with open(dump_path, "w") as f:
        json.dump({
            "exp1_scalability": exp1,
            "exp2_replanning": exp2,
            "exp3_dimension": exp3,
            "exp4_pareto": exp4
        }, f, indent=2)

    print(f"\n[+] Raw results saved to {dump_path}")
    print("\n" + "=" * 80)
    print("ALL EXPERIMENTS COMPLETED SUCCESSFULLY")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    main()
