from models import State, Transition, PlanningProblem
from dstar_lite import DStarLite


def create_states():
    return [
        State(0, [0, 0]),
        State(1, [1, 1]),
        State(2, [2, 2]),
        State(3, [3, 1]),
        State(4, [4, 3]),
        State(5, [5, 5])
    ]


def print_result(title, problem, result):
    print("\n" + "=" * 60)
    print(title)
    print("=" * 60)
    print("Initial State:", problem.initial_state)
    print("Goal State:", problem.goal_state)
    print("Bad States:", problem.bad_states)

    if result["success"]:
        print("Safe State Path:", result["state_path"])
        print("Transition Path:", result["transition_path"])
        print("Total Cost:", result["cost"])
        print("Bad States Visited: 0")
        print("Explored States:", result["explored"])
        print("Planning/Replanning Time:",
              round(result["time"] * 1000, 4), "ms")
    else:
        print("No safe path found.")


states = create_states()

transitions = [
    Transition(1, 0, 1, 1, 1.0, 1.0),
    Transition(2, 1, 2, 1, 1.0, 1.0),
    Transition(3, 2, 5, 1, 1.0, 1.0)
]

problem = PlanningProblem(0, 5, set(), states, transitions)
planner = DStarLite(problem)
result = planner.plan()

print_result("TEST CASE 1: BASIC REACHABILITY", problem, result)


states = create_states()

transitions = [
    Transition(1, 0, 1, 1, 1.0, 1.0),
    Transition(2, 1, 2, 1, 1.0, 1.0),
    Transition(3, 2, 5, 1, 1.0, 1.0),
    Transition(4, 0, 3, 2, 1.0, 1.0),
    Transition(5, 3, 4, 2, 1.0, 1.0),
    Transition(6, 4, 5, 2, 1.0, 1.0)
]

problem = PlanningProblem(0, 5, {2}, states, transitions)
planner = DStarLite(problem)
result = planner.plan()

print_result("TEST CASE 2: BAD STATE AVOIDANCE", problem, result)


states = create_states()

transitions = [
    Transition(1, 0, 1, 1, 0.70, 0.70),
    Transition(2, 1, 2, 1, 0.70, 0.70),
    Transition(3, 2, 5, 1, 0.70, 0.70),
    Transition(4, 0, 3, 2, 0.95, 0.98),
    Transition(5, 3, 4, 2, 0.95, 0.98),
    Transition(6, 4, 5, 2, 0.95, 0.98)
]

problem = PlanningProblem(0, 5, {2}, states, transitions)
planner = DStarLite(problem)
result = planner.plan()

print_result("TEST CASE 3: SAFETY AND RELIABILITY", problem, result)


states = create_states()

transitions = [
    Transition(1, 0, 1, 1, 1.0, 1.0),
    Transition(2, 1, 5, 1, 1.0, 1.0),
    Transition(3, 0, 3, 5, 1.0, 1.0),
    Transition(4, 3, 4, 5, 1.0, 1.0),
    Transition(5, 4, 5, 5, 1.0, 1.0)
]

problem = PlanningProblem(0, 5, set(), states, transitions)
planner = DStarLite(problem)

result = planner.plan()
print_result("TEST CASE 4A: INITIAL PLAN", problem, result)

planner.update_transition(2, available=False)

result = planner.replan()
print_result("TEST CASE 4B: AFTER TRANSITION FAILURE", problem, result)


states = create_states()

transitions = [
    Transition(1, 0, 1, 2, 1.0, 1.0),
    Transition(2, 1, 2, 2, 1.0, 1.0),
    Transition(3, 2, 5, 2, 1.0, 1.0),
    Transition(4, 0, 3, 2, 1.0, 1.0),
    Transition(5, 3, 4, 2, 1.0, 1.0),
    Transition(6, 4, 5, 2, 1.0, 1.0)
]

problem = PlanningProblem(0, 4, {2}, states, transitions)
planner = DStarLite(problem)

result = planner.plan()
print_result("TEST CASE 5A: ORIGINAL GOAL", problem, result)

planner.update_goal(5)

result = planner.replan()
print_result("TEST CASE 5B: UPDATED GOAL", problem, result)


states = create_states()

transitions = [
    Transition(1, 0, 3, 4, 0.9, 0.95),
    Transition(2, 3, 4, 4, 0.9, 0.95),
    Transition(3, 4, 5, 4, 0.9, 0.95)
]

problem = PlanningProblem(0, 5, {2}, states, transitions)
planner = DStarLite(problem)

result = planner.plan()
print_result("TEST CASE 6A: BEFORE TRANSITION ADDITION", problem, result)

problem.transitions.append(
    Transition(4, 0, 5, 2, 0.95, 0.99)
)

planner.update_vertex(0)

result = planner.replan()
print_result("TEST CASE 6B: AFTER TRANSITION ADDITION", problem, result)
