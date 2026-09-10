# Safe Semantic Planner in a Finite Cartesian State Space

**Course:** PCCST503 – Machine Learning  
**Assignment:** Assignment 1  
**Topic:** Design of a Safe Semantic Planner in a Finite Cartesian State Space  
**Implementation:** Python 3  
**Algorithm:** D* Lite (incremental heuristic graph search)

## 1. Objective

This project implements a generic planner for a finite Cartesian state space. It receives an initial state, goal state, bad states and directed transitions. Each transition contains cost, safety, reliability and availability.

The planner:
- reaches the goal when a valid route exists,
- never enters a bad state,
- minimizes transition cost while accounting for safety and reliability,
- calculates minimum Euclidean distance to bad states,
- supports dynamic transition updates,
- supports goal changes,
- supports transition insertion and removal.

## 2. State Representation

Each state is represented by:
- `id`: unique integer identifier
- `embedding`: Cartesian coordinate tuple such as `(x, y)`

The Euclidean distance between two states is:

`d(s1,s2) = sqrt(sum((s1[i]-s2[i])^2))`

The safety distance of a state is its minimum Euclidean distance to any bad state.

## 3. Transition Representation

Each transition contains:
- `id`
- `source`
- `target`
- `cost`
- `safety`
- `reliability`
- `available`

Unavailable transitions and transitions entering bad states are ignored.

## 4. Heuristic and Objective

D* Lite uses the Euclidean distance between the current state and goal as its admissible geometric heuristic.

For route selection, the implementation uses an effective transition cost:

`effective_cost = cost + safety_penalty + reliability_penalty`

where:

` safety_penalty = SAFETY_WEIGHT / safety_distance `

and

` reliability_penalty = RELIABILITY_WEIGHT * (1 - reliability) `

This makes the planner prefer routes that are inexpensive, sufficiently far from bad states and reliable.

## 5. Safety

A bad state is never inserted into a valid path.

For every visited state, the planner calculates the distance to the nearest bad state. The reported safety score is the minimum of those distances over the complete path.

Therefore, a larger minimum safety distance means a larger safety margin.

## 6. Why D* Lite?

D* Lite is an incremental replanning algorithm. Instead of performing a completely new search after every local graph change, it maintains `g` and `rhs` values and repairs the affected portion of the search graph.

The implementation exposes:
- `update_transition()` for availability/cost/safety/reliability changes,
- `add_transition()` for new edges,
- `remove_transition()` for deleted edges,
- `set_goal()` for goal updates.

## 7. Time and Space Complexity

For a graph with `V` states and `E` transitions, D* Lite has logarithmic priority-queue operations and is efficient for repeated changes. A conservative bound for a planning/replanning episode is commonly expressed around `O((V + E) log V)` depending on the number of affected vertices and graph updates.

Space usage is `O(V + E)` for state values, adjacency lists, transitions and the priority queue.

## 8. Test Cases

The implementation covers all six cases from the assignment:

1. **Basic Reachability** – finds a path from S to G.
2. **Bad State Avoidance** – avoids X even when a short route passes through X.
3. **Safety Margin** – includes safety distance in route evaluation.
4. **Dynamic Transition** – replans after a transition becomes unavailable.
5. **Goal Update** – changes the goal and computes a revised route.
6. **Transition Addition** – discovers a newly inserted shortcut.

## 9. How to Run

No external packages are required.

```bash
python planner.py
```

Run the unit tests:

```bash
python -m unittest test_planner.py -v
```

Generate experimental results:

```bash
python experiment.py
```

This creates:

`experimental_results.csv`

## 10. Files

- `planner.py` – main D* Lite planner and data structures
- `test_planner.py` – six assignment test cases
- `experiment.py` – experimental evaluation
- `experimental_results.csv` – recorded evaluation results
- `report.md` – design report
- `requirements.txt` – dependency information

## 11. Expected Demonstration

Run `planner.py` and show that a valid state path is produced.

Then run:

```bash
python -m unittest test_planner.py -v
```

All six tests should pass.

## 12. Conclusion

The project demonstrates safe graph planning in a finite Cartesian state space. The planner combines graph-search efficiency with safety and reliability considerations and supports dynamic replanning when the environment changes.
