# Design Report — Safe Semantic Planner

## 1. Introduction

This project implements a generic planner for a finite Cartesian state space. The assignment requires the planner to reach a goal, avoid bad states, minimize transition cost, maximize the minimum distance from bad states, and remain practical under dynamic updates.

The implementation chooses **D* Lite**. D* Lite is suitable because the environment can change after an initial plan and the planner should repair an existing solution instead of rebuilding the entire search state whenever possible.

## 2. State representation

Each state has:

```cpp
struct State {
    uint64_t id;
    std::vector<double> embedding;
};
```

`id` uniquely identifies the state. `embedding` stores its Cartesian coordinates in R^d.

The Euclidean distance is:

```text
d(a,b) = sqrt(sum_i (a_i-b_i)^2)
```

The nearest-bad-state distance for state `s` is:

```text
d(s,B) = min_b in B d(s,b)
```

## 3. Transition representation

Each directed transition stores:

- transition ID;
- source state;
- destination state;
- base cost;
- safety field;
- reliability;
- availability flag.

The planner treats unavailable transitions as unusable.

## 4. Data structures

The implementation uses:

- `unordered_map` for state and transition lookup;
- adjacency lists for outgoing and incoming transitions;
- `unordered_set` for bad states;
- `unordered_map` for D* Lite `g` and `rhs` values;
- a priority queue for the D* Lite open list.

Incoming adjacency is important because D* Lite propagates changes backward from affected vertices to their predecessors.

## 5. D* Lite formulation

The planner maintains:

```text
g(s)   = current estimate of cost-to-go
rhs(s) = one-step lookahead value
```

The goal is initialized with:

```text
rhs(goal) = 0
g(goal) = infinity
```

For every other state:

```text
rhs(u) = min [ c(u,v) + g(v) ]
```

over available, valid, non-bad outgoing transitions.

The priority key is:

```text
K(u) = (
    min(g(u), rhs(u)) + h(start,u) + km,
    min(g(u), rhs(u))
)
```

where `h` is Euclidean distance and `km` supports movement of the current start.

When a vertex becomes inconsistent (`g != rhs`), it is inserted into the priority queue. `computeShortestPath()` repeatedly repairs the most urgent inconsistent vertices.

## 6. Safety computation

Bad states are hard constraints. The planner does not use an edge entering a bad state.

For every other state, the distance to the closest bad state is computed from the Cartesian embeddings.

The final reported safety score is:

```text
min distance-to-bad over all states in the returned path
```

This directly matches the assignment's minimum-safety-distance metric.

### Safety-aware search cost

Because the minimum-distance objective is not naturally additive, the implementation uses a scalarized edge cost:

```text
C_eff =
    C_transition
    + alpha / max(d(to,B), epsilon)
    + beta * (1 - reliability)
```

where `alpha` is `safetyWeight` and `beta` is `reliabilityWeight`.

This makes states close to bad states more expensive while preserving the original transition cost. The final minimum distance is reported separately so experiments can compare cost against actual safety.

## 7. Reliability

Reliability is incorporated as a penalty:

```text
reliability_penalty = beta * (1 - reliability)
```

A reliability of 1 adds no penalty; a reliability of 0 adds the full configured penalty.

The result also reports cumulative path reliability as the product of transition reliabilities.

## 8. Dynamic updates and replanning

The planner supports:

- changing the goal;
- changing bad states;
- enabling/disabling a transition;
- adding a transition;
- removing a transition.

For local transition changes, affected vertices are updated and their predecessors are allowed to propagate inconsistency through the D* Lite queue.

Changing the goal or bad-state set resets the D* Lite consistency structure because these operations can change many edge costs at once. The graph itself is retained, so no external data structure rebuild is required.

This distinction is important: incremental repair is strongest for local transition changes, while global changes can require broader propagation.

## 9. Correctness properties

### Goal reachability

If a finite-cost path exists from start to goal under the current constraints, D* Lite's shortest-path repair converges to a consistent shortest-cost solution under the effective edge weights.

### Bad-state avoidance

Edges whose source or destination is a bad state receive infinite weight and are never selected.

### Dynamic transition changes

When a transition changes availability, its source is updated and the change propagates through predecessor relationships.

## 10. Complexity

Let:

- `V` = number of states;
- `E` = number of directed transitions.

For a conventional D* Lite search, the main graph-search work is bounded by the same asymptotic family as repeated shortest-path repair, commonly expressed as approximately:

```text
O(E log V)
```

for a major repair under standard priority-queue assumptions.

The current implementation additionally computes nearest-bad-state distance by scanning all bad states when evaluating an edge. If `B` is the number of bad states and `d` is embedding dimension, one such distance calculation costs:

```text
O(B*d)
```

Therefore, the practical implementation can have an additional safety-cost factor. A production optimization would cache nearest-bad-state distances and invalidate only affected states when the bad-state set changes.

Space complexity is:

```text
O(V + E)
```

for graph, D* Lite values, adjacency lists, and queue entries, excluding repeated stale queue entries.

## 11. Experimental evaluation

The demonstration contains the six scenarios specified by the assignment:

1. Basic reachability;
2. bad-state avoidance;
3. safety margin;
4. dynamic transition;
5. goal update;
6. transition addition.

The program reports:

- success;
- state path;
- total transition cost;
- minimum safety distance;
- cumulative reliability;
- explored states;
- planning time;
- replanning time.

Run:

```bash
cmake -S . -B build
cmake --build build -j
./build/safe_planner
ctest --test-dir build --output-on-failure
```

The actual timing numbers depend on the student's machine, compiler, operating system, and build configuration. They should be copied from the executable rather than invented in the report.

## 12. Conclusion

The project implements a safe, safety-aware, reliability-aware D* Lite planner with dynamic transition updates. The design directly addresses the required graph search, heuristic, safety, optimization, and replanning objectives.
