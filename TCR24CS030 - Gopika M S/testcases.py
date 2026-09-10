
"""
PCCST503 - Machine Learning
Assignment 1
Test Cases for Safe Semantic Planner
"""

from planner import run_tests


def display_result(name, result):

    print("\n" + "=" * 70)
    print(name)
    print("=" * 70)

    print("Success:", result.success)
    print("State Path:", result.state_path)
    print("Transition Path:", result.transition_path)

    print(
        "Total Cost:",
        round(result.total_cost, 3)
    )

    print(
        "Minimum Safety Distance:",
        round(
            result.minimum_safety_distance,
            3
        )
    )

    print(
        "Cumulative Reliability:",
        round(
            result.cumulative_reliability,
            4
        )
    )

    print(
        "Explored States:",
        result.explored_states
    )

    print(
        "Planning Time:",
        round(
            result.planning_time_ms,
            3
        ),
        "ms"
    )


def main():

    print("\nPCCST503 - SAFE SEMANTIC PLANNER")
    print("D* Lite Test Case Evaluation")

    results = run_tests()

    for name, result in results:

        display_result(
            name,
            result
        )

    successful = sum(
        1
        for _, result in results
        if result.success
    )

    print("\n" + "=" * 70)
    print("SUMMARY")
    print("=" * 70)

    print(
        "Successful test runs:",
        successful,
        "/",
        len(results)
    )

    if successful == len(results):

        print(
            "Overall result: ALL TESTS PASSED"
        )

    else:

        print(
            "Overall result: SOME TESTS FAILED"
        )


if __name__ == "__main__":
    main()
