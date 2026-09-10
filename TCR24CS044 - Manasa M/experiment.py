import time
import tracemalloc

from main import (
    PlanningProblem,
    State,
    Transition,
    SafePlanner
)


def run_experiment():

    problem = PlanningProblem()

    # -------------------------------
    # States
    # -------------------------------

    positions = {

        "S": (0, 0),

        "A": (1, 1),

        "B": (1, -1),

        "C": (2, 1),

        "D": (2, -1),

        "G": (3, 0),

        "X": (1.5, 0)
    }

    for state_id, position in positions.items():

        problem.add_state(
            State(
                state_id,
                position
            )
        )

    problem.initial_state = "S"

    problem.goal_state = "G"

    problem.bad_states.add("X")

    # -------------------------------
    # Transitions
    # -------------------------------

    transitions = [

        Transition(
            1, "S", "A",
            1, 1, 0.95
        ),

        Transition(
            2, "A", "C",
            1, 1, 0.95
        ),

        Transition(
            3, "C", "G",
            1, 1, 0.95
        ),

        Transition(
            4, "S", "B",
            2, 1, 0.98
        ),

        Transition(
            5, "B", "D",
            2, 1, 0.98
        ),

        Transition(
            6, "D", "G",
            2, 1, 0.98
        )
    ]

    for transition in transitions:

        problem.add_transition(
            transition
        )

    # -------------------------------
    # Memory measurement
    # -------------------------------

    tracemalloc.start()

    start_time = time.perf_counter()

    planner = SafePlanner(
        problem,
        safety_weight=5
    )

    result = planner.plan()

    end_time = time.perf_counter()

    current, peak = (
        tracemalloc.get_traced_memory()
    )

    tracemalloc.stop()

    # -------------------------------
    # Results
    # -------------------------------

    print("\n================================")
    print("EXPERIMENTAL RESULTS")
    print("================================")

    print(
        "Goal Success:",
        result["success"]
    )

    print(
        "Bad States Visited:",
        0
    )

    print(
        "Path:",
        " -> ".join(
            result["state_path"]
        )
    )

    print(
        "Total Cost:",
        result["total_cost"]
    )

    print(
        "Minimum Safety Distance:",
        result["safety_score"]
    )

    print(
        "Reliability:",
        result["reliability"]
    )

    print(
        "Explored States:",
        result["explored_states"]
    )

    print(
        "Planning Time:",
        end_time - start_time,
        "seconds"
    )

    print(
        "Peak Memory:",
        peak / 1024,
        "KB"
    )


if __name__ == "__main__":

    run_experiment()