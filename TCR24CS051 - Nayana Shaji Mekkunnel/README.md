# Safe Semantic Planner (Python)

This is a Python implementation of a safe dynamic planner for the PCCST503
assignment. It uses D* Lite over a directed finite Cartesian state space.

## What it does

- Never visits a supplied bad state.
- Finds a lowest adjusted-cost route using incremental D* Lite.
- Supports changes to an edge's availability, cost, safety, or reliability.
- Supports a moving start, changing bad-state set, and a changing goal.
- Reports raw path cost, minimum Euclidean clearance from bad states,
  reliability, explored states, and planning time.

The default configuration minimizes raw cost. To prefer clearance, construct
the planner with `safety_weight > 0`; it adds `safety_weight / clearance` to
the effective cost of entering each state. Safety and reliability fields on a
transition can likewise be enabled with their corresponding weights.

## Run

From this folder:

```powershell
python -m unittest -v
```

No third-party package is required. `test_planner.py` implements the six
illustrative test cases in the brief.

## Minimal use

```python
from safe_planner import State, Transition, PlanningProblem, SafeDStarLitePlanner

problem = PlanningProblem(
    initial_state=0, goal_state=2,
    states=[State(0, (0, 0)), State(1, (1, 0)), State(2, (2, 0))],
    transitions=[Transition(0, 0, 1, 1), Transition(1, 1, 2, 1)],
    bad_states=set(),
)
planner = SafeDStarLitePlanner(problem)
print(planner.plan())

# Incremental local replan after an edge outage:
planner.update_transition(1, available=False)
print(planner.plan())

# Add a newly discovered shortcut:
planner.add_transition(Transition(2, 0, 2, 0.5))
```

## Design notes

D* Lite maintains `g` and one-step lookahead `rhs` values in a priority queue.
An edge change updates the affected source vertex and propagates only as far as
needed on the next call to `plan()`. Updating the bad-state set can alter every
clearance value, so the implementation refreshes all vertices but preserves
the loaded graph. Goal changes reset goal-dependent D* Lite values; that is
the safe, standard behavior for this algorithm.

For `|V|` states and `|E|` transitions, initialization is `O(|E| log |V|)` in
the worst case and uses `O(|V| + |E|)` memory. Local incremental edge changes
usually revisit only the affected region, although worst-case replanning is
still `O(|E| log |V|)`.