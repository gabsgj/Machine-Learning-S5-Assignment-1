import csv
import time
from planner import DStarLitePlanner, build_demo_problem


def run_case(name, planner):
    start = time.perf_counter()
    result = planner.plan()
    elapsed = time.perf_counter() - start
    return {
        "test_case": name,
        "success": result.success,
        "state_path": "->".join(map(str, result.state_path)),
        "transition_path": "->".join(map(str, result.transition_path)),
        "total_path_cost": round(result.total_cost, 4),
        "minimum_safety_distance": round(result.safety_score, 4),
        "average_reliability": round(result.reliability, 4),
        "explored_states": result.explored_states,
        "planning_time_seconds": round(elapsed, 8),
    }


def main():
    rows = []

    # Test 1-3
    states, transitions = build_demo_problem()
    planner = DStarLitePlanner(states, transitions, 0, 3, {6})
    rows.append(run_case("1 - Basic Reachability", planner))
    rows.append(run_case("2 - Bad State Avoidance", planner))
    rows.append(run_case("3 - Safety Margin", planner))

    # Test 4 - dynamic transition
    states, transitions = build_demo_problem()
    planner = DStarLitePlanner(states, transitions, 0, 3, {6})
    planner.plan()
    planner.update_transition(13, available=False)
    rows.append(run_case("4 - Dynamic Transition", planner))

    # Test 5 - goal update
    states, transitions = build_demo_problem()
    planner = DStarLitePlanner(states, transitions, 0, 3, {6})
    planner.set_goal(11)
    rows.append(run_case("5 - Goal Update", planner))

    # Test 6 - transition addition
    states, transitions = build_demo_problem()
    planner = DStarLitePlanner(states, transitions, 0, 3, {6})
    planner.update_transition(13, available=False)
    planner.plan()
    from planner import Transition
    planner.add_transition(Transition(99, 0, 3, 0.5, 0.99, 0.99, True))
    rows.append(run_case("6 - Transition Addition", planner))

    with open("experimental_results.csv", "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print("Experimental results written to experimental_results.csv")
    for row in rows:
        print(row)


if __name__ == "__main__":
    main()
