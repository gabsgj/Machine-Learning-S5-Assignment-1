"""
test_cases.py
The 6 illustrative test cases from the assignment, written as runnable functions.
Each function builds its own small PlanningProblem, runs the DStarLite planner,
checks the result against what the assignment says SHOULD happen, and prints a
pass/fail line.
"""

from models import State, Transition, PlanningProblem
from dstar_lite import DStarLite


def _check(label: str, condition: bool):
    status = "PASS" if condition else "FAIL"
    print(f"  [{status}] {label}")
    return condition


# ---------------------------------------------------------------------------
# Test Case 1: Basic Reachability   S -> A -> B -> G
# ---------------------------------------------------------------------------
def test_case_1():
    print("\nTest Case 1: Basic Reachability (S -> A -> B -> G)")

    states = [State(0, [0, 0]), State(1, [1, 0]), State(2, [2, 0]), State(3, [3, 0])]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=1.0, reliability=1.0),
        Transition(1, 1, 2, cost=1, safety=1.0, reliability=1.0),
        Transition(2, 2, 3, cost=1, safety=1.0, reliability=1.0),
    ]
    problem = PlanningProblem(initial_state=0, goal_state=3, bad_states=[],
                               states=states, transitions=transitions)

    planner = DStarLite()
    result = planner.plan(problem)

    print(f"  path: {result.state_path}, cost: {result.total_cost}")
    ok = _check("planner succeeds", result.success)
    ok &= _check("path is the unique route [0,1,2,3]", result.state_path == [0, 1, 2, 3])
    return ok


# ---------------------------------------------------------------------------
# Test Case 2: Bad State Avoidance
#   S -> A -> X(bad) -> G   (shorter, forbidden)
#   S -> C -> D -> G        (longer, must be chosen)
# ---------------------------------------------------------------------------
def test_case_2():
    print("\nTest Case 2: Bad State Avoidance")

    states = [
        State(0, [0, 0]),   # S
        State(1, [1, 1]),   # A
        State(2, [2, 1]),   # X (bad)
        State(3, [1, -1]),  # C
        State(4, [2, -1]),  # D
        State(5, [3, 0]),   # G
    ]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=1.0, reliability=1.0),  # S->A
        Transition(1, 1, 2, cost=1, safety=1.0, reliability=1.0),  # A->X
        Transition(2, 2, 5, cost=1, safety=1.0, reliability=1.0),  # X->G
        Transition(3, 0, 3, cost=1, safety=1.0, reliability=1.0),  # S->C
        Transition(4, 3, 4, cost=1, safety=1.0, reliability=1.0),  # C->D
        Transition(5, 4, 5, cost=1, safety=1.0, reliability=1.0),  # D->G
    ]
    problem = PlanningProblem(initial_state=0, goal_state=5, bad_states=[2],
                               states=states, transitions=transitions)

    planner = DStarLite()
    result = planner.plan(problem)

    print(f"  path: {result.state_path}, cost: {result.total_cost}")
    ok = _check("planner succeeds", result.success)
    ok &= _check("bad state (2) never visited", 2 not in result.state_path)
    ok &= _check("path is the safe route [0,3,4,5]", result.state_path == [0, 3, 4, 5])
    return ok


# ---------------------------------------------------------------------------
# Test Case 3: Safety Margin
#   Path 1 (via A): cheap (raw cost 2), but its edges have LOW safety scores
#                   because it passes right next to a bad state.
#   Path 2 (via C): pricier (raw cost 4), but its edges have HIGH safety scores
#                   because it stays far from the bad state.
#   We show that safety_weight controls which one the planner actually picks.
# ---------------------------------------------------------------------------
def test_case_3():
    print("\nTest Case 3: Safety Margin (cost vs. danger trade-off)")

    states = [
        State(0, [0, 0]),      # S
        State(1, [1, 0.05]),   # A -- sits right next to the bad state
        State(2, [2, 0]),      # G
        State(3, [1, 5]),      # C -- far from the bad state
        State(4, [1, 0]),      # BAD state
    ]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=0.1, reliability=1.0),  # S->A  (cheap, risky)
        Transition(1, 1, 2, cost=1, safety=0.1, reliability=1.0),  # A->G  (cheap, risky)
        Transition(2, 0, 3, cost=2, safety=0.9, reliability=1.0),  # S->C  (pricier, safe)
        Transition(3, 3, 2, cost=2, safety=0.9, reliability=1.0),  # C->G  (pricier, safe)
    ]
    problem = PlanningProblem(initial_state=0, goal_state=2, bad_states=[4],
                               states=states, transitions=transitions)

    # Pure cost minimization: safety_weight = 0 -> should take the cheap, risky route
    cheap_planner = DStarLite(safety_weight=0.0, reliability_weight=0.0)
    cheap_result = cheap_planner.plan(problem)
    print(f"  safety_weight=0.0 -> path={cheap_result.state_path}, "
          f"cost={cheap_result.total_cost}, min-dist-to-bad={cheap_result.safety_score:.2f}")

    # Safety-sensitive: safety_weight high -> should switch to the safer, pricier route
    safe_planner = DStarLite(safety_weight=0.5, reliability_weight=0.0)
    safe_result = safe_planner.plan(problem)
    print(f"  safety_weight=0.5 -> path={safe_result.state_path}, "
          f"cost={safe_result.total_cost}, min-dist-to-bad={safe_result.safety_score:.2f}")

    ok = _check("both planners succeed", cheap_result.success and safe_result.success)
    ok &= _check("safety_weight=0 picks the cheap route via A",
                 cheap_result.state_path == [0, 1, 2])
    ok &= _check("safety_weight=0.5 switches to the safer route via C",
                 safe_result.state_path == [0, 3, 2])
    ok &= _check("the safer route does keep a larger minimum distance from the bad state",
                 safe_result.safety_score > cheap_result.safety_score)
    return ok


# ---------------------------------------------------------------------------
# Test Case 4: Dynamic Transition
#   Initially S -> A -> G is the plan. Then (A, G) becomes unavailable.
#   Planner must find the alternative route WITHOUT a full graph rebuild.
# ---------------------------------------------------------------------------
def test_case_4():
    print("\nTest Case 4: Dynamic Transition (edge disappears mid-plan)")

    states = [State(0, [0, 0]), State(1, [1, 0]), State(2, [2, 0]),
              State(3, [1, -1]), State(4, [2, -1])]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=1.0, reliability=1.0),  # S->A
        Transition(1, 1, 2, cost=1, safety=1.0, reliability=1.0),  # A->G  (will break)
        Transition(2, 0, 3, cost=1, safety=1.0, reliability=1.0),  # S->C
        Transition(3, 3, 4, cost=1, safety=1.0, reliability=1.0),  # C->D
        Transition(4, 4, 2, cost=1, safety=1.0, reliability=1.0),  # D->G
    ]
    problem = PlanningProblem(initial_state=0, goal_state=2, bad_states=[],
                               states=states, transitions=transitions)

    planner = DStarLite()
    result1 = planner.plan(problem)
    print(f"  before break: path={result1.state_path}, cost={result1.total_cost}")

    result2 = planner.update_transition(1, 2, available=False)
    print(f"  after  break: path={result2.state_path}, cost={result2.total_cost}, "
          f"states touched this replan={result2.explored_states - result1.explored_states}")

    ok = _check("initial plan succeeds via A", result1.success and result1.state_path == [0, 1, 2])
    ok &= _check("replan succeeds via alternative route", result2.success and 1 not in result2.state_path[1:])
    ok &= _check("replan touches far fewer states than a full re-search would",
                 (result2.explored_states - result1.explored_states) < len(states))
    return ok


# ---------------------------------------------------------------------------
# Test Case 5: Goal Update
# ---------------------------------------------------------------------------
def test_case_5():
    print("\nTest Case 5: Goal Update")

    states = [State(0, [0, 0]), State(1, [1, 0]), State(2, [2, 0]), State(3, [3, 0])]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=1.0, reliability=1.0),
        Transition(1, 1, 2, cost=1, safety=1.0, reliability=1.0),
        Transition(2, 2, 3, cost=1, safety=1.0, reliability=1.0),
    ]
    problem = PlanningProblem(initial_state=0, goal_state=2, bad_states=[],
                               states=states, transitions=transitions)

    planner = DStarLite()
    result1 = planner.plan(problem)
    print(f"  goal=2: path={result1.state_path}")

    result2 = planner.set_goal(3, problem)
    print(f"  goal=3: path={result2.state_path}")

    ok = _check("first goal reached correctly", result1.state_path == [0, 1, 2])
    ok &= _check("revised goal reached correctly", result2.state_path == [0, 1, 2, 3])
    return ok


# ---------------------------------------------------------------------------
# Test Case 6: Transition Addition (a new shortcut appears)
# ---------------------------------------------------------------------------
def test_case_6():
    print("\nTest Case 6: Transition Addition (shortcut appears)")

    states = [State(0, [0, 0]), State(1, [1, 0]), State(2, [2, 0]), State(3, [3, 0])]
    transitions = [
        Transition(0, 0, 1, cost=1, safety=1.0, reliability=1.0),
        Transition(1, 1, 2, cost=1, safety=1.0, reliability=1.0),
        Transition(2, 2, 3, cost=1, safety=1.0, reliability=1.0),
    ]
    problem = PlanningProblem(initial_state=0, goal_state=3, bad_states=[],
                               states=states, transitions=transitions)

    planner = DStarLite()
    result1 = planner.plan(problem)
    print(f"  before shortcut: path={result1.state_path}, cost={result1.total_cost}")

    shortcut = Transition(99, 0, 3, cost=1, safety=1.0, reliability=1.0)  # S -> G directly
    result2 = planner.add_transition(shortcut)
    print(f"  after  shortcut: path={result2.state_path}, cost={result2.total_cost}")

    ok = _check("original plan takes 3 hops", len(result1.state_path) == 4)
    ok &= _check("planner discovers and uses the new shortcut", result2.state_path == [0, 3])
    ok &= _check("shortcut reduces total cost", result2.total_cost < result1.total_cost)
    return ok


if __name__ == "__main__":
    tests = [test_case_1, test_case_2, test_case_3, test_case_4, test_case_5, test_case_6]
    results = [t() for t in tests]

    print("\n" + "=" * 40)
    print(f"SUMMARY: {sum(results)}/{len(results)} test cases passed")
    print("=" * 40)