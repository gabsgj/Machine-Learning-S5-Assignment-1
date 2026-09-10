"""
experiments.py
Generates random planning problems of increasing size and measures the metrics
the assignment asks for:
  - goal success rate
  - number of bad states visited (expected zero)
  - total path cost
  - minimum distance to bad states
  - number of explored states
  - planning time
  - memory usage
  - replanning time (incremental vs. full re-plan, for comparison)

Results are printed to the console AND saved to results.csv so they can be
pasted into the report / turned into charts.
"""

import copy
import csv
import random
import time
import tracemalloc

from models import State, Transition, PlanningProblem
from dstar_lite import DStarLite


# ---------------------------------------------------------------------------
# Random problem generator
# ---------------------------------------------------------------------------
def generate_random_problem(num_states=50, num_bad=5, k_neighbors=4, seed=0):
    """
    Scatter `num_states` points randomly in a 100x100 square, connect each
    point to its k nearest neighbors (a common way to build a realistic-ish
    graph from a point cloud), and mark a few states as bad.
    """
    rng = random.Random(seed)
    states = [State(i, [rng.uniform(0, 100), rng.uniform(0, 100)]) for i in range(num_states)]

    bad_states = rng.sample(range(num_states), num_bad)

    transitions = []
    tid = 0
    for s in states:
        neighbors = sorted((o for o in states if o.id != s.id), key=lambda o: s.distance_to(o))
        for o in neighbors[:k_neighbors]:
            cost = s.distance_to(o)
            safety = rng.uniform(0.4, 1.0)
            reliability = rng.uniform(0.8, 1.0)
            transitions.append(Transition(tid, s.id, o.id, cost=cost,
                                           safety=safety, reliability=reliability))
            tid += 1

    initial_state = 0
    goal_state = num_states - 1
    # make sure start/goal aren't accidentally bad states
    bad_states = [b for b in bad_states if b not in (initial_state, goal_state)]

    return PlanningProblem(initial_state, goal_state, bad_states, states, transitions)


# ---------------------------------------------------------------------------
# Single-run measurement
# ---------------------------------------------------------------------------
def measure_single_run(num_states, num_bad, k_neighbors, seed):
    problem = generate_random_problem(num_states, num_bad, k_neighbors, seed)
    planner = DStarLite()

    tracemalloc.start()
    result = planner.plan(problem)
    _, peak_mem = tracemalloc.get_traced_memory()
    tracemalloc.stop()

    bad_states_visited = sum(1 for sid in result.state_path if sid in problem.bad_states)

    row = {
        "num_states": num_states,
        "success": result.success,
        "bad_states_visited": bad_states_visited,
        "total_cost": round(result.total_cost, 3) if result.success else None,
        "min_dist_to_bad": round(result.safety_score, 3) if result.success else None,
        "explored_states": result.explored_states,
        "planning_time_s": round(result.planning_time, 6),
        "peak_memory_kb": round(peak_mem / 1024, 2),
        "incremental_replan_time_s": None,
        "full_replan_time_s": None,
    }

    # ---- replanning comparison: break the first edge of the found path ----
    if result.success and len(result.state_path) > 1:
        u, v = result.state_path[0], result.state_path[1]

        t0 = time.perf_counter()
        planner.update_transition(u, v, available=False)
        row["incremental_replan_time_s"] = round(time.perf_counter() - t0, 6)

        # For a fair comparison: build an identical problem with that edge
        # disabled from the start, and plan it completely from scratch.
        problem_broken = copy.deepcopy(problem)
        for t in problem_broken.transitions:
            if t.from_id == u and t.to_id == v:
                t.available = False

        fresh_planner = DStarLite()
        t0 = time.perf_counter()
        fresh_planner.plan(problem_broken)
        row["full_replan_time_s"] = round(time.perf_counter() - t0, 6)

    return row


# ---------------------------------------------------------------------------
# Main experiment sweep
# ---------------------------------------------------------------------------
def run_experiments():
    sizes = [20, 50, 100, 200]
    num_trials = 5
    rows = []

    for size in sizes:
        for trial in range(num_trials):
            row = measure_single_run(
                num_states=size,
                num_bad=max(1, size // 10),
                k_neighbors=4,
                seed=trial,
            )
            row["trial"] = trial
            rows.append(row)
            print(
                f"n={size:4d} trial={trial}  success={row['success']!s:5}  "
                f"cost={row['total_cost']}  explored={row['explored_states']:4d}  "
                f"plan_t={row['planning_time_s']:.5f}s  mem={row['peak_memory_kb']}KB  "
                f"incr_replan={row['incremental_replan_time_s']}s  "
                f"full_replan={row['full_replan_time_s']}s"
            )

    # ---- summary per size ----
    print("\n" + "=" * 80)
    print("SUMMARY (averaged over trials)")
    print("=" * 80)
    for size in sizes:
        size_rows = [r for r in rows if r["num_states"] == size]
        success_rate = sum(r["success"] for r in size_rows) / len(size_rows)
        avg_explored = sum(r["explored_states"] for r in size_rows) / len(size_rows)
        avg_time = sum(r["planning_time_s"] for r in size_rows) / len(size_rows)
        avg_mem = sum(r["peak_memory_kb"] for r in size_rows) / len(size_rows)

        incr_times = [r["incremental_replan_time_s"] for r in size_rows if r["incremental_replan_time_s"] is not None]
        full_times = [r["full_replan_time_s"] for r in size_rows if r["full_replan_time_s"] is not None]
        avg_incr = sum(incr_times) / len(incr_times) if incr_times else None
        avg_full = sum(full_times) / len(full_times) if full_times else None

        print(f"n={size:4d}  success_rate={success_rate:.0%}  avg_explored={avg_explored:.1f}  "
              f"avg_plan_time={avg_time:.5f}s  avg_mem={avg_mem:.1f}KB  "
              f"avg_incremental_replan={avg_incr}  avg_full_replan={avg_full}")

    # ---- save raw data to CSV for the report ----
    with open("results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    print("\nRaw results written to results.csv")


if __name__ == "__main__":
    run_experiments()