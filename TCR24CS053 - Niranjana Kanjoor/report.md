# Design Report
## Safe Semantic Planner in a Finite Cartesian State Space

### 1. Introduction

The objective is to design a planner that computes a safe path in a finite Cartesian state space. The input consists of an initial state, a goal state, bad states and directed transitions. Every transition has a cost, reliability, safety score and availability flag.

The implementation uses Python and the D* Lite incremental search approach.

### 2. State Representation

A state is represented by a unique integer ID and a Cartesian embedding.

Example:

```text
State(0, (0, 0))
State(1, (1, 1))
```

The embedding allows Euclidean distance to be used as the geometric heuristic and for safety-distance calculation.

### 3. Data Structures

The implementation uses:
- dictionaries for states and transitions,
- adjacency lists for outgoing and incoming transitions,
- a binary heap priority queue for the D* Lite open list,
- `g` values for estimated cost-to-go,
- `rhs` values for one-step lookahead cost,
- a set for bad states.

### 4. Transition Model

A transition contains source and target state IDs and the following attributes:

| Attribute | Meaning |
|---|---|
| cost | Basic traversal cost |
| safety | Transition-provided safety value |
| reliability | Reliability in the range 0 to 1 |
| available | Whether the transition can currently be used |

Unavailable transitions are ignored.

### 5. Heuristic Function

The heuristic is Euclidean distance between the current state and goal:

`h(s,g) = sqrt(sum((s_i-g_i)^2))`

Because the state embeddings are Cartesian coordinates, this provides a natural geometric estimate.

### 6. Safety Computation

For a state `s`, safety distance is:

`D(s) = min distance(s,b)` for all bad states `b`.

If a state itself is bad, its safety distance is zero and the state is prohibited.

For a complete path, the minimum safety distance among all visited states is reported as the path safety score.

### 7. Route Cost

The planner uses the basic transition cost together with safety and reliability penalties:

`C_eff = C + alpha/D + beta(1-R)`

where:
- `C` = transition cost,
- `D` = distance to nearest bad state,
- `R` = transition reliability,
- `alpha` = safety weight,
- `beta` = reliability weight.

The implementation uses `alpha = 0.8` and `beta = 0.5`.

This balances low cost with safer and more reliable transitions.

### 8. D* Lite Algorithm

D* Lite maintains two values for every state:
- `g(s)` – current best known cost-to-go,
- `rhs(s)` – one-step lookahead value.

The goal is initialized with `rhs(goal)=0`. States whose values are inconsistent are maintained in a priority queue.

When a graph change occurs, affected vertices and their predecessors are identified and updated; the lightweight search-value tables are then reset to guarantee correctness after arbitrary graph edits. This is useful in dynamic environments where transitions can become unavailable or new transitions can appear.

### 9. Dynamic Replanning

The implementation supports:

**Transition availability update**

```python
planner.update_transition(13, available=False)
```

**Goal update**

```python
planner.set_goal(new_goal)
```

**New transition**

```python
planner.add_transition(new_transition)
```

**Transition removal**

```python
planner.remove_transition(transition_id)
```

These operations allow the planner to adapt when the environment changes.

### 10. Test Cases

#### Test Case 1 – Basic Reachability

A valid route exists from the start to the goal. The planner returns a successful path.

#### Test Case 2 – Bad State Avoidance

State X is marked as bad. Although a short route passes through X, the planner rejects that route and chooses a valid alternative.

#### Test Case 3 – Safety Margin

Multiple valid routes can exist with different distances from bad states. The effective cost includes a safety penalty, encouraging a larger safety margin.

#### Test Case 4 – Dynamic Transition

A transition is made unavailable after an initial plan. The planner updates the affected vertices and computes another valid route.

#### Test Case 5 – Goal Update

The goal is changed during execution. The planner initializes the new goal condition and computes a revised route.

#### Test Case 6 – Transition Addition

A new shortcut transition is inserted. The planner incorporates the new edge and can discover the improved solution.

### 11. Experimental Metrics

The implementation records:
- goal success,
- state path,
- transition path,
- total path cost,
- minimum safety distance,
- average reliability,
- number of explored states,
- planning time.

These correspond to the evaluation requirements in the assignment.

### 12. Complexity

For `V` states and `E` transitions, the maintained graph structures require `O(V+E)` space. D* Lite uses a priority queue, so individual queue operations are logarithmic. Overall planning/replanning work depends on the number of affected vertices; a conservative graph-search bound is approximately `O((V+E) log V)` for a planning episode.

### 13. Conclusion

The implemented Safe Semantic Planner satisfies the main planning requirements: goal reachability, strict avoidance of bad states, cost optimization, safety-distance evaluation and dynamic replanning. The use of D* Lite makes the implementation suitable for environments in which transition availability and goals can change over time.
