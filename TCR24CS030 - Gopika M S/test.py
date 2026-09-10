"""
PCCST503 - Machine Learning
Assignment 1
Automatic test execution and experimental result generation.
"""

from planner import run_tests
import csv


def main():

    results = run_tests()

    print("=" * 70)
    print("PCCST503 - SAFE SEMANTIC PLANNER")
    print("EXPERIMENTAL TEST RESULTS")
    print("=" * 70)

    with open(
        "experimental_results.csv",
        "w",
        newline=""
    ) as file:

        writer = csv.writer(file)

        writer.writerow([
            "Test",
            "Success",
            "State Path",
            "Transition Path",
            "Total Cost",
            "Minimum Safety Distance",
            "Cumulative Reliability",
            "Explored States",
            "Planning Time (ms)"
        ])

        for name, result in results:

            status = "PASS" if result.success else "FAIL"

            writer.writerow([
                name,
                result.success,
                "->".join(
                    map(str, result.state_path)
                ),
                "->".join(
                    map(str, result.transition_path)
                ),
                round(result.total_cost, 4),
                round(
                    result.minimum_safety_distance,
                    4
                ),
                round(
                    result.cumulative_reliability,
                    4
                ),
                result.explored_states,
                round(
                    result.planning_time_ms,
                    4
                )
            ])

            print("\n" + name)
            print("-" * 70)
            print("Status:", status)
            print("State Path:", result.state_path)
            print(
                "Transition Path:",
                result.transition_path
            )
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
                "Reliability:",
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

    print("\n" + "=" * 70)
    print("Results saved to experimental_results.csv")
    print("=" * 70)


if __name__ == "__main__":
    main()
