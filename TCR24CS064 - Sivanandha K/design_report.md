# Design Report
## PCCST503 – Machine Learning, Assignment 1
### Design of a Safe Semantic Planner in a Finite Cartesian State Space

## 1. Objective

The assignment requires a generic planner that computes a safe path through a finite Cartesian state space while avoiding bad states, minimizing transition cost, maintaining a large safety margin, and operating efficiently in a changing environment.

D* Lite was selected because it is designed for incremental replanning when the graph or environment changes.

## 2. State Representation

Each state is represented as:

```cpp
struct State {
    uint64_t id;
    std::vector<double> embedding;
};
```

`id` uniquely identifies a state. `embedding` stores the Cartesian coordinates in an arbitrary dimension `d`, so the implementation is not limited to two-dimensional examples.

The planning problem contains:
- initial state;
- goal state;
- bad-state IDs;
- all states;
- directed transitions.

## 3. Transition Representation

Each directed transition stores:

```cpp
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};
```

This directly represents the interface requested in the assignment. The graph maintains both outgoing and incoming transition lists. Incoming lists are important for D* Lite because predecessor vertices must be updated after a value changes.

## 4. Data Structures

The implementation uses:

| Structure | Purpose |
|---|---|
| `unordered_map<uint64_t, State>` | State lookup by ID |
| `unordered_map<uint64_t, Transition>` | Transition lookup |
| `unordered_map<uint64_t, vector<uint64_t>> outgoing` | Successors of a state |
| `unordered_map<uint64_t, vector<uint64_t>> incoming` | Predecessors of a state |
| `unordered_set<uint64_t>` | Bad-state membership |
| `unordered_map<uint64_t,double> g` | D* Lite cost-to-go estimate |
| `unordered_map<uint64_t,double> rhs` | One-step lookahead value |
| Priority queue | D* Lite open list |
| Safety-distance map | Distance from each state to the nearest bad state |

## 5. Safety Computation

For every state `s`, the planner calculates its Euclidean distance to every bad state and stores the minimum:

`D(s) = min ||embedding(s) - embedding(b)||_2`

for all bad states `b`.

A bad state itself is never allowed in a solution path. This is deliberately a hard constraint because the assignment requires zero bad-state visits.

When no bad states are defined, the minimum safety distance is reported as **N/A**, because there is no nearest bad state from which a meaningful finite distance can be calculated.

## 6. Objective and Edge Weight

The assignment gives the example objective:

`Score(P) = αG − βC + γD + δR`

The implementation treats goal completion and bad-state avoidance as hard constraints and converts the remaining soft objectives into a minimization edge weight.

For a transition `t` ending at state `v`:

`effective_cost(t) = cost(t) + λ(1 − safety(t))/(D(v)+1) + ρ(1 − reliability(t))`

where:
- `λ` controls the influence of transition safety;
- `ρ` controls the influence of reliability;
- `D(v)` is the distance of the destination state from the nearest bad state.

This means a route that is slightly more expensive can be preferred when it provides a significantly better safety margin or reliability. The hard exclusion of bad states guarantees that a bad state is never selected merely because it has low cost.

## 7. Heuristic Function

D* Lite uses:

`h(s1,s2) = k × EuclideanDistance(s1,s2)`

The factor `k` is computed from the minimum effective transition-cost / geometric-distance ratio over currently usable transitions. This prevents the heuristic from exceeding the implemented effective path cost under the graph model.

When a safe geometric lower bound cannot be established, `k = 0`, reducing the method to a D* Lite search with an admissible zero heuristic.

## 8. D* Lite Procedure

The main procedure is:

1. Load states and transitions.
2. Compute distance to the nearest bad state.
3. Compute the safe heuristic scale.
4. Initialize `rhs(goal)=0` and insert the goal into the priority queue.
5. Repeatedly process the smallest D* Lite key.
6. Update `g` and `rhs` values and propagate changes to predecessor states.
7. Stop when the start state is locally consistent and no better queue key exists.
8. Reconstruct the path by repeatedly choosing the available successor minimizing `effective_cost + g(successor)`.
9. Evaluate path metrics.

## 9. Dynamic Environment and Replanning

The environment can change through:
- goal updates;
- transition availability changes;
- transition insertion.

For an availability change, the affected predecessor is updated and D* Lite propagates the effect through the existing graph instead of performing a fresh graph construction.

For a goal change, the graph, state embeddings, transition records, safety distances, and adjacency structures are reused. Only goal-dependent D* Lite search values are reinitialized.

For a newly inserted transition, its outgoing/incoming references are added and the affected source is updated before continuing the search.

This satisfies the assignment's requirement to explain how the planner can replan efficiently after updates.

## 10. Complexity

Let:
- `V` = number of states;
- `E` = number of directed transitions;
- `B` = number of bad states;
- `d` = embedding dimension.

### Safety preprocessing
The straightforward nearest-bad-state calculation takes:

`O(V × B × d)`

### D* Lite planning
With a binary heap priority queue, the graph-search portion has the standard logarithmic priority-queue behavior and is commonly expressed as approximately:

`O((V + E) log V)`

for a full search, with incremental updates potentially touching substantially fewer vertices than a complete re-search.

### Space
The graph, D* Lite values, and queue require:

`O(V + E)`

space, excluding the coordinate storage factor `O(Vd)` and the safety-distance preprocessing storage.

## 11. Test Cases

### Test Case 1 – Basic Reachability
Graph:

`S → A → B → G`

The planner returns the unique valid path.

### Test Case 2 – Bad State Avoidance
The cheaper route passes through a bad state `X`. The planner rejects that route and selects the alternative safe route.

### Test Case 3 – Safety Margin
Two safe routes are available. The planner is configured with a stronger safety weight, so the route with a larger minimum distance from the bad state is selected despite its larger raw transition cost.

### Test Case 4 – Dynamic Transition
Initially `S → A → G` is selected. When `A → G` becomes unavailable, the planner incrementally replans and selects the alternative route.

### Test Case 5 – Goal Update
The goal changes during execution. The graph data are retained and the planner computes a new route to the new goal.

### Test Case 6 – Transition Addition
A new shortcut is inserted. The planner discovers the shortcut and improves the solution.

## 12. Experimental Results

The included program was compiled with:

`g++ -std=c++17 -O2 -Wall -Wextra -pedantic`

Representative results from the supplied test run are summarized below. Planning times are machine-dependent and should not be treated as universal benchmarks.

| Case | Result | State path | Cost | Min safety distance | Bad states | Explored | Replan ms |
|---|---|---|---:|---:|---:|---:|---:|
| 1 | Success | 1→2→3→4 | 3.0 | N/A | 0 | 4 | — |
| 2 | Success | 1→3→5 | 2.4 | 1.000 | 0 | 3 | — |
| 3 | Success | 1→3→4→6 | 5.0 | 1.803 | 0 | 4 | — |
| 4A | Success | 1→2→4 | 2.0 | N/A | 0 | 3 | — |
| 4B | Success | 1→3→4 | 3.0 | N/A | 0 | 7 | ~0.001 |
| 5A | Success | 1→2→3→4 | 3.0 | N/A | 0 | 4 | — |
| 5B | Success | 1→5 | 1.2 | N/A | 0 | 2 | ~0.001 |
| 6A | Success | 1→2→3→4 | 6.0 | N/A | 0 | 4 | — |
| 6B | Success | 1→4 | 0.8 | N/A | 0 | 5 | ~0.001 |

The raw execution output is stored in `results/experimental_results.txt`.

## 13. Discussion

The experiments demonstrate the required behavior:

- The goal is reached in every solvable case.
- Zero bad states are visited.
- The bad-state route is rejected even when it would otherwise be attractive.
- Increasing the safety weight can trade path cost for a larger safety margin.
- A failed transition triggers an alternative route without reconstructing the graph.
- A changed goal produces a new route while retaining the graph representation.
- A newly added shortcut is incorporated and produces a lower-cost solution.

The reported planning and replanning times are very small because the illustrative graphs contain only a few states. Larger benchmarks are needed for a meaningful performance comparison.

## 14. Limitations

1. Safety-distance preprocessing is a straightforward `V × B` computation and could be optimized for very large state spaces.
2. Goal changes reinitialize goal-dependent D* Lite values; the graph itself is reused, but this is not as incremental as an edge-cost update.
3. The implementation uses scalar weights for safety and reliability. Different applications may require calibrated weights.
4. The supplied illustrative tests do not constitute a statistically significant runtime benchmark.
5. The implementation is a standalone C++ demonstration rather than a GUI or ROS planner.

## 15. Conclusion

A generic D* Lite-based safe semantic planner has been implemented for a finite Cartesian state space. It represents states and transitions using the requested fields, excludes bad states, incorporates cost, safety and reliability, and supports dynamic transition and goal updates. The six required illustrative scenarios are included and produce safe solutions with zero bad-state visits.
