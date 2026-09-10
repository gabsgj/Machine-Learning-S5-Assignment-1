import time

from planner.dstar_lite import DStarLite

from experiments.test_cases import (
    test_case_1,
    test_case_2,
    test_case_3,
    test_case_4,
    test_case_5,
    test_case_6,
)


TEST_CASES = {
    "Test 1 - Basic Reachability": test_case_1,
    "Test 2 - Bad State Avoidance": test_case_2,
    "Test 3 - Safety Margin": test_case_3,
    "Test 4 - Dynamic Transition": test_case_4,
    "Test 5 - Goal Update": test_case_5,
    "Test 6 - Transition Addition": test_case_6,
}


def run_benchmark():

    results = []

    print("=" * 70)
    print("SAFE SEMANTIC PLANNER - EXPERIMENTAL EVALUATION")
    print("=" * 70)

    for name, test_function in TEST_CASES.items():

        problem = test_function()

        start_time = time.perf_counter()

        planner = DStarLite(
            problem=problem,
            cost_weight=1.0,
            safety_weight=0.0,
            reliability_weight=0.0,
        )

        result = planner.plan()

        elapsed_ms = (
            time.perf_counter()
            - start_time
        ) * 1000

        bad_states_visited = sum(
            1
            for state_id in result.state_path
            if state_id in problem.bad_states
        )

        print()
        print("-" * 70)
        print(name)
        print("-" * 70)

        print(
            f"Success:              {result.success}"
        )

        print(
            f"Path:                 {result.state_path}"
        )

        print(
            f"Total Cost:           {result.total_cost:.3f}"
        )

        print(
            f"Minimum Safety:       "
            f"{result.safety_score}"
        )

        print(
            f"Reliability:          "
            f"{result.reliability_score:.3f}"
        )

        print(
            f"Bad States Visited:   "
            f"{bad_states_visited}"
        )

        print(
            f"Explored States:      "
            f"{result.explored_states}"
        )

        print(
            f"Planning Time:        "
            f"{elapsed_ms:.3f} ms"
        )

        results.append(
            {
                "Test Case": name,
                "Success": result.success,
                "Path": result.state_path,
                "Cost": result.total_cost,
                "Safety": result.safety_score,
                "Reliability": result.reliability_score,
                "Bad States": bad_states_visited,
                "Explored States": result.explored_states,
                "Planning Time (ms)": elapsed_ms,
            }
        )

    return results


if __name__ == "__main__":

    results = run_benchmark()

    print()
    print()
    print("=" * 70)
    print("SUMMARY")
    print("=" * 70)

    for row in results:

        print(
            f"{row['Test Case']:<35}"
            f"Success={str(row['Success']):<7}"
            f"Cost={row['Cost']:<8.2f}"
            f"Explored={row['Explored States']:<5}"
            f"Time={row['Planning Time (ms)']:.3f} ms"
        )