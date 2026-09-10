from planner.models import (
    State,
    Transition,
    PlanningProblem
)


def create_states():

    return [
        State(0, "S", [0.0, 0.0]),
        State(1, "A", [1.0, 0.0]),
        State(2, "B", [2.0, 0.0]),
        State(3, "G", [3.0, 0.0]),

        State(4, "C", [1.0, 1.0]),
        State(5, "D", [2.0, 1.0]),

        State(6, "X", [2.0, -1.0]),
    ]


def create_problem(
    edges,
    bad_states=None,
    goal_state=3
):

    transitions = []

    for i, edge in enumerate(edges):

        if len(edge) == 2:

            source, target = edge
            cost = 1.0

        else:

            source, target, cost = edge

        transitions.append(
            Transition(
                id=i,
                from_state=source,
                to_state=target,
                cost=cost,
                safety=1.0,
                reliability=1.0,
                available=True
            )
        )

    return PlanningProblem(
        initial_state=0,
        goal_state=goal_state,
        bad_states=set(bad_states or []),
        states=create_states(),
        transitions=transitions
    )


# ======================================================
# TEST CASE 1
# Basic Reachability
# ======================================================

def test_case_1():

    edges = [
        (0, 1),
        (1, 2),
        (2, 3)
    ]

    return create_problem(edges)


# ======================================================
# TEST CASE 2
# Bad State Avoidance
# ======================================================

def test_case_2():

    edges = [
        # Dangerous route
        (0, 1),
        (1, 6),
        (6, 3),

        # Safe route
        (0, 4),
        (4, 5),
        (5, 3)
    ]

    return create_problem(
        edges,
        bad_states={6}
    )


# ======================================================
# TEST CASE 3
# Safety Margin
# ======================================================

def test_case_3():

    edges = [
        # Cheap route
        (0, 1, 1),
        (1, 6, 1),
        (6, 3, 1),

        # Safer route
        (0, 4, 2),
        (4, 5, 2),
        (5, 3, 2)
    ]

    return create_problem(
        edges,
        bad_states={6}
    )


# ======================================================
# TEST CASE 4
# Dynamic Transition
# ======================================================

def test_case_4():

    edges = [
        # Initially preferred route
        (0, 1, 1),
        (1, 3, 1),

        # Alternative route
        (1, 2, 2),
        (2, 3, 2)
    ]

    return create_problem(edges)


# ======================================================
# TEST CASE 5
# Goal Update
# ======================================================

def test_case_5():

    edges = [
        (0, 1),
        (1, 3),

        (0, 4),
        (4, 5),
        (5, 3)
    ]

    return create_problem(
        edges,
        goal_state=3
    )


# ======================================================
# TEST CASE 6
# Transition Addition
# ======================================================

def test_case_6():

    edges = [
        # Existing longer route
        (0, 1, 5),
        (1, 3, 5),

        # Alternative route
        (0, 4, 3),
        (4, 5, 3),
        (5, 3, 3)
    ]

    return create_problem(edges)