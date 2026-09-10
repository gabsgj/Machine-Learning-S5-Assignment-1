# test_cases.py
# All 6 test cases from the assignment PDF.
# Each function creates and returns a PlanningProblem.

from structures import State, Transition, PlanningProblem


# ============================================================
# Test Case 1: Basic Reachability
# Simple linear path: S -> A -> B -> G
# No bad states. Should find the only valid path.
# ============================================================
def create_test_case_1():
    states = [
        State(id=0, embedding=[0.0, 0.0]),   # S (start)
        State(id=1, embedding=[1.0, 0.0]),   # A
        State(id=2, embedding=[2.0, 0.0]),   # B
        State(id=3, embedding=[3.0, 0.0]),   # G (goal)
    ]

    transitions = [
        Transition(id=10, from_state=0, to_state=1, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=11, from_state=1, to_state=2, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=12, from_state=2, to_state=3, cost=1.0, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=3,
        badStates=[],
        states=states,
        transitions=transitions
    )


# ============================================================
# Test Case 2: Bad State Avoidance
# Path 1: S -> A -> X -> G  (X is a bad state!)
# Path 2: S -> C -> D -> G  (safe path)
# Planner should pick Path 2.
# ============================================================
def create_test_case_2():
    states = [
        State(id=0, embedding=[0.0, 0.0]),    # S (start)
        State(id=1, embedding=[1.0, 1.0]),    # A
        State(id=2, embedding=[2.0, 1.0]),    # X (BAD STATE)
        State(id=3, embedding=[1.0, -1.0]),   # C
        State(id=4, embedding=[3.0, 0.0]),    # G (goal)
        State(id=5, embedding=[2.0, -1.0]),   # D
    ]

    transitions = [
        # Path 1 (through bad state X)
        Transition(id=20, from_state=0, to_state=1, cost=1.0, safety=0.5, reliability=1.0, available=True),
        Transition(id=21, from_state=1, to_state=2, cost=1.0, safety=0.0, reliability=1.0, available=True),
        Transition(id=22, from_state=2, to_state=4, cost=1.0, safety=0.0, reliability=1.0, available=True),
        # Path 2 (safe detour)
        Transition(id=23, from_state=0, to_state=3, cost=1.5, safety=1.0, reliability=1.0, available=True),
        Transition(id=24, from_state=3, to_state=5, cost=1.5, safety=1.0, reliability=1.0, available=True),
        Transition(id=25, from_state=5, to_state=4, cost=1.5, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=4,
        badStates=[2],       # X is bad
        states=states,
        transitions=transitions
    )


# ============================================================
# Test Case 3: Safety Margin
# Two valid paths, neither goes through a bad state, but:
# - Path 1: cheaper cost, but passes CLOSE to bad states
# - Path 2: higher cost, but stays FAR from bad states
# This tests how the planner balances cost vs safety distance.
# ============================================================
def create_test_case_3():
    states = [
        State(id=0, embedding=[0.0, 0.0]),    # S (start)
        State(id=1, embedding=[1.0, 0.5]),    # A (close to bad state B1)
        State(id=2, embedding=[2.0, 0.5]),    # B (close to bad state B1)
        State(id=3, embedding=[1.0, -3.0]),   # C (far from bad states)
        State(id=4, embedding=[2.0, -3.0]),   # D (far from bad states)
        State(id=5, embedding=[3.0, 0.0]),    # G (goal)
        State(id=6, embedding=[1.0, 1.0]),    # B1 (bad state, near path 1)
    ]

    transitions = [
        # Path 1: S -> A -> B -> G  (cheap but close to bad state)
        Transition(id=30, from_state=0, to_state=1, cost=1.0, safety=0.5, reliability=0.9, available=True),
        Transition(id=31, from_state=1, to_state=2, cost=1.0, safety=0.5, reliability=0.9, available=True),
        Transition(id=32, from_state=2, to_state=5, cost=1.0, safety=0.5, reliability=0.9, available=True),
        # Path 2: S -> C -> D -> G  (expensive but far from bad states)
        Transition(id=33, from_state=0, to_state=3, cost=2.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=34, from_state=3, to_state=4, cost=2.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=35, from_state=4, to_state=5, cost=2.0, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=5,
        badStates=[6],       # B1 is the bad state
        states=states,
        transitions=transitions
    )


# ============================================================
# Test Case 4: Dynamic Transition
# Initially: S -> A -> G (direct path)
# Also:      S -> B -> C -> G (backup path)
# After planning, transition A -> G becomes unavailable.
# Planner should find the backup path.
# ============================================================
def create_test_case_4():
    states = [
        State(id=0, embedding=[0.0, 0.0]),    # S (start)
        State(id=1, embedding=[1.0, 0.0]),    # A
        State(id=2, embedding=[1.0, -1.0]),   # B
        State(id=3, embedding=[2.0, -1.0]),   # C
        State(id=4, embedding=[3.0, 0.0]),    # G (goal)
    ]

    transitions = [
        # Direct path: S -> A -> G
        Transition(id=40, from_state=0, to_state=1, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=41, from_state=1, to_state=4, cost=1.0, safety=1.0, reliability=1.0, available=True),
        # Backup path: S -> B -> C -> G
        Transition(id=42, from_state=0, to_state=2, cost=1.5, safety=1.0, reliability=1.0, available=True),
        Transition(id=43, from_state=2, to_state=3, cost=1.5, safety=1.0, reliability=1.0, available=True),
        Transition(id=44, from_state=3, to_state=4, cost=1.5, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=4,
        badStates=[],
        states=states,
        transitions=transitions
    )


# ============================================================
# Test Case 5: Goal Update
# S -> A -> G1 (original goal)
# S -> A -> B -> G2 (new goal after update)
# The goal changes from G1 to G2, planner should find new path.
# ============================================================
def create_test_case_5():
    states = [
        State(id=0, embedding=[0.0, 0.0]),    # S (start)
        State(id=1, embedding=[1.0, 0.0]),    # A
        State(id=2, embedding=[2.0, 0.0]),    # G1 (original goal)
        State(id=3, embedding=[2.0, 1.0]),    # B
        State(id=4, embedding=[3.0, 1.0]),    # G2 (new goal)
    ]

    transitions = [
        Transition(id=50, from_state=0, to_state=1, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=51, from_state=1, to_state=2, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=52, from_state=1, to_state=3, cost=1.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=53, from_state=3, to_state=4, cost=1.0, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=2,         # original goal is G1
        badStates=[],
        states=states,
        transitions=transitions
    )


# ============================================================
# Test Case 6: Transition Addition
# Initially: S -> A -> B -> G (long path)
# Later: a new shortcut S -> G is added
# Planner should discover the improved solution.
# ============================================================
def create_test_case_6():
    states = [
        State(id=0, embedding=[0.0, 0.0]),    # S (start)
        State(id=1, embedding=[1.0, 0.0]),    # A
        State(id=2, embedding=[2.0, 0.0]),    # B
        State(id=3, embedding=[3.0, 0.0]),    # G (goal)
    ]

    transitions = [
        # Original long path: S -> A -> B -> G
        Transition(id=60, from_state=0, to_state=1, cost=2.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=61, from_state=1, to_state=2, cost=2.0, safety=1.0, reliability=1.0, available=True),
        Transition(id=62, from_state=2, to_state=3, cost=2.0, safety=1.0, reliability=1.0, available=True),
    ]

    return PlanningProblem(
        initialState=0,
        goalState=3,
        badStates=[],
        states=states,
        transitions=transitions
    )
