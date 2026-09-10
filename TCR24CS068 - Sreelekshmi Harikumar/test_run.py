from planner.dstar_lite import DStarLite

from experiments.test_cases import (
    test_case_1,
    test_case_2,
    test_case_3,
    test_case_4,
    test_case_5,
    test_case_6
)


tests = [
    ("Test 1", test_case_1()),
    ("Test 2", test_case_2()),
    ("Test 3", test_case_3()),
    ("Test 4", test_case_4()),
    ("Test 5", test_case_5()),
    ("Test 6", test_case_6()),
]


for name, problem in tests:

    planner = DStarLite(problem)

    result = planner.plan()

    print("=" * 50)

    print(name)

    print("Success:", result.success)

    print("State path:", result.state_path)

    print("Total cost:", result.total_cost)

    print("Safety:", result.safety_score)

    print("Reliability:", result.reliability_score)

    print("Explored states:", result.explored_states)

    print("Planning time:", result.planning_time, "ms")

    print("Message:", result.message)