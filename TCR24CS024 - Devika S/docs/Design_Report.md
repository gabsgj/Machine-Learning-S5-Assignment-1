# Design Report
## Safe Semantic Planner in a Finite Cartesian State Space

### 1. Introduction

The objective is to design a planner that computes a safe path between an initial state and a goal state in a finite Cartesian state space.

Each state has a vector representation. Directed transitions connect states and contain cost, safety, reliability and an availability flag.

The planner must reach the goal without visiting bad states while balancing path cost and safety.

### 2. State Representation

A state is represented using:

```cpp
class State {
public:
    uint64_t id;
    vector<double> embedding;
};
```

The `id` uniquely identifies the state.

The `embedding` stores its Cartesian coordinates.

For example:

```text
State 0 = (0, 0)
State 1 = (1, 0)
State 2 = (2, 0)
```

### 3. Transition Representation

A transition contains:

- transition ID
- source state
- destination state
- cost
- safety score
- reliability
- availability

A transition is ignored if it is unavailable or connects to a bad state.

### 4. Planning Problem

The planning problem contains:

- initial state
- goal state
- set of bad states
- list of states
- list of transitions

### 5. Algorithm

A simplified D* Lite-style graph-search method is used.

The main steps are:

1. Store all states and transitions.
2. Mark bad states.
3. Build incoming and outgoing adjacency lists.
4. Initialize the goal with zero cost.
5. Perform reverse priority-queue search from the goal.
6. Propagate path values toward the initial state.
7. Starting from the initial state, select the best available successor.
8. Stop when the goal is reached.
9. Calculate the minimum Euclidean distance from the path to all bad states.

### 6. Heuristic Function

Euclidean distance is used:

```text
h(s1,s2) = sqrt(
    (x1-x2)^2 +
    (y1-y2)^2 +
    ...
)
```

For a d-dimensional Cartesian embedding:

```text
h(s1,s2) = sqrt( sum((s1[i]-s2[i])^2) )
```

This provides a geometric estimate between states.

### 7. Combined Transition Cost

The implementation combines the original transition cost with safety and reliability penalties:

```text
combinedCost =
    cost
    + 2 × (1 / (safety + 0.01))
    + (1 - reliability)
```

Therefore:

- lower transition cost is preferred
- higher safety reduces the safety penalty
- higher reliability reduces the reliability penalty

This allows the planner to consider more than only physical path length.

### 8. Safety Computation

The safety score is defined as the minimum Euclidean distance between any visited state and any bad state.

```text
D = min distance(visited state, bad state)
```

A larger value means the path stays farther from dangerous states.

Bad states themselves are never included in the generated path.

### 9. Dynamic Environment

The environment can change because:

- the goal can change
- a transition can become unavailable
- a new transition can be added
- bad states can be changed

The program supports these changes by updating the planning problem and running the planner again.

For example:

```cpp
problem.transitions[1].available = false;
```

makes a transition unavailable and causes an alternative route to be selected.

Similarly:

```cpp
problem.goalState = 4;
```

changes the goal.

A new transition can be added using:

```cpp
problem.transitions.push_back(
    Transition(4, 0, 4, 2, 10, 0.9)
);
```

### 10. Time Complexity

For a graph containing `V` states and `E` transitions, the priority-queue search is approximately:

```text
O((V + E) log V)
```

### 11. Space Complexity

The adjacency lists and search structures require approximately:

```text
O(V + E)
```

memory.

### 12. Test Cases

#### Test Case 1: Basic Reachability

Graph:

```text
S → A → B → G
```

Expected result: the unique valid path is selected.

#### Test Case 2: Bad State Avoidance

```text
S → A → X → G
```

where `X` is bad.

Alternative:

```text
S → C → D → G
```

The planner selects the safe route.

#### Test Case 3: Safety Margin

Two valid paths are available.

The planner considers safety as part of the transition value and also reports the minimum distance to bad states.

#### Test Case 4: Dynamic Transition

Initially:

```text
S → A → G
```

After `(A,G)` becomes unavailable, an alternative route is selected.

#### Test Case 5: Goal Update

The goal is changed while the program is running. The planner computes a new route to the updated goal.

#### Test Case 6: Transition Addition

A new shortcut is added. The planner considers it when calculating the new route.

### 13. Evaluation Metrics

The following metrics are reported:

- goal success
- bad-state visits
- total path cost
- minimum safety distance
- explored states
- planning time

### 14. Conclusion

The implemented planner provides a safe graph-search solution for a finite Cartesian state space. It avoids bad states, considers cost, safety and reliability, and supports environmental changes such as transition removal, transition addition and goal updates.

The project demonstrates how graph search and heuristic information can be combined to solve safe path-planning problems efficiently.
