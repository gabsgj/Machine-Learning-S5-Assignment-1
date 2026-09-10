from planner.dstar_lite import DStarLite
from planner.models import (
    State,
    Transition,
    PlanningProblem,
)


# ==========================================================
# CREATE TEST GRAPH
# ==========================================================

states = [
    State(0, "S", [0.0, 0.0]),
    State(1, "A", [1.0, 0.0]),
    State(2, "B", [2.0, 1.0]),
    State(3, "G", [3.0, 0.0]),
]


transitions = [
    Transition(
        id=0,
        from_state=0,
        to_state=1,
        cost=1.0,
    ),

    Transition(
        id=1,
        from_state=1,
        to_state=3,
        cost=1.0,
    ),

    Transition(
        id=2,
        from_state=1,
        to_state=2,
        cost=2.0,
    ),

    Transition(
        id=3,
        from_state=2,
        to_state=3,
        cost=2.0,
    ),
]


problem = PlanningProblem(
    initial_state=0,
    goal_state=3,
    bad_states=set(),
    states=states,
    transitions=transitions,
)


# ==========================================================
# CREATE PLANNER
# ==========================================================

planner = DStarLite(problem)


# ==========================================================
# INITIAL PLAN
# ==========================================================

print("=" * 60)
print("INITIAL PLAN")
print("=" * 60)

result1 = planner.plan()

print("Success:", result1.success)
print("Path:", result1.state_path)
print("Cost:", result1.total_cost)
print("Explored:", result1.explored_states)
print("Time:", result1.planning_time, "ms")


# ==========================================================
# DISABLE A -> G
# ==========================================================

print()
print("=" * 60)
print("DYNAMIC UPDATE")
print("=" * 60)

print("Disabling transition A -> G")

planner.update_transition(
    transition_id=1,
    available=False,
)


# ==========================================================
# INCREMENTAL REPLAN
# ==========================================================

print()
print("=" * 60)
print("INCREMENTAL REPLAN")
print("=" * 60)

result2 = planner.replan()

print("Success:", result2.success)
print("Path:", result2.state_path)
print("Cost:", result2.total_cost)
print("Explored:", result2.explored_states)
print("Time:", result2.planning_time, "ms")


# ==========================================================
# EXPECTED RESULT
# ==========================================================

print()
print("=" * 60)
print("EXPECTED")
print("=" * 60)

print("Initial path:")
print("[0, 1, 3]")

print()
print("Updated path:")
print("[0, 1, 2, 3]")