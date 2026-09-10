from planner import Planner
from test_cases import build_test_graph
from transition import Transition
from utils import draw_graph


def print_path(path):
    if path:
        print(" -> ".join(path))
    else:
        print("No Path Found")


def main():

    # ==========================
    # Test Case 1
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("G")

    print("\n===== Test Case 1 =====")
    print("Basic Reachability")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_1.png")

    # ==========================
    # Test Case 2
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("G")

    planner.add_bad_state("B")

    print("\n===== Test Case 2 =====")
    print("Bad State Avoidance")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_2.png")

    # ==========================
    # Test Case 3
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("G")

    print("\n===== Test Case 3 =====")
    print("Safety Preference")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_3.png")

    # ==========================
    # Test Case 4
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("G")

    # Block transition A -> B
    for edge in planner.graph["A"]:
        if edge.to_state == "B":
            edge.available = False

    print("\n===== Test Case 4 =====")
    print("Dynamic Transition")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_4.png")

    # ==========================
    # Test Case 5
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("D")

    print("\n===== Test Case 5 =====")
    print("Goal Update")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_5.png")

    # ==========================
    # Test Case 6
    # ==========================
    planner = Planner()
    build_test_graph(planner)

    planner.set_initial_state("S")
    planner.set_goal_state("G")

    planner.add_transition(
        Transition(
            "S",
            "G",
            1,
            0.99,
            0.99
        )
    )

    print("\n===== Test Case 6 =====")
    print("Transition Addition")

    path = planner.plan()
    print_path(path)
    draw_graph(planner, path, "test_case_6.png")

    print("\nGraphs have been saved in the 'output' folder.")


if __name__ == "__main__":
    main()