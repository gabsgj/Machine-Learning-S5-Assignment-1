# PCCST503 – Machine Learning

## Assignment 1: Design of a Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**

---

## 1. Introduction

This project focuses on the design and implementation of a safe semantic planner for a finite Cartesian state space.

The planning environment is represented as a directed graph consisting of states and transitions. Each state has a Cartesian embedding, while each transition contains cost, safety, reliability, and availability information.

The planner receives an initial state, a goal state, and a set of bad states. It searches for a path from the initial state to the goal while avoiding bad states.

The implementation uses a D* Lite based graph-search approach. It also demonstrates replanning when the planning environment changes.

---

## 2. Objective

The main objectives of the planner are:

1. Reach the specified goal state.
2. Avoid all bad states.
3. Minimize the total transition cost.
4. Prefer paths that maintain a larger distance from bad states.
5. Consider transition reliability.
6. Consider transition availability.
7. Support changes in the planning environment.
8. Measure planning and replanning performance.

---

## 3. Problem Definition

Let the finite set of states be:

$$
S=\{s_1,s_2,\ldots,s_n\}
$$

Each state is embedded in a Cartesian space:

$$
s_i=(x_1,x_2,\ldots,x_d)
$$

The planner receives the following inputs:

* Initial state \(s_I\)
* Goal state \(s_G\)
* Set of bad states \(B\)
* Set of directed transitions \(T\)

Each transition contains:

* Transition ID
* Source state
* Destination state
* Cost
* Safety score
* Reliability
* Availability flag

The objective is to find a path:

$$
P=(s_I,s_1,s_2,\ldots,s_G)
$$

such that no visited state belongs to the bad-state set.

---

## 4. State Representation

A state is represented using an ID and a Cartesian embedding.

```cpp
class State {
public:
    uint64_t id;
    vector<double> embedding;
};
```

The state ID uniquely identifies the state.

The embedding stores the coordinates of the state in the Cartesian space.

For example:

```text
State 0 = (0,0)
State 1 = (1,0)
State 2 = (2,0)
State 3 = (3,0)
```

The Euclidean distance between states \(s_i\) and \(s_j\) is:

$$
d(s_i,s_j)=
\sqrt{
\sum_{k=1}^{d}
(x_{ik}-x_{jk})^2
}
$$

This distance is used for heuristic estimation and safety calculations.

---

## 5. Transition Representation

A transition represents a directed connection between two states.

```cpp
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};
```

The transition fields have the following meanings:

| Field       | Description                                            |
| ----------- | ------------------------------------------------------ |
| ID          | Unique transition identifier                           |
| From        | Source state                                           |
| To          | Destination state                                      |
| Cost        | Cost of using the transition                           |
| Safety      | Safety score associated with the transition            |
| Reliability | Reliability of the transition                          |
| Available   | Indicates whether the transition can currently be used |

---

## 6. Planning Problem

The complete planning problem is represented as:

```cpp
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};
```

It contains all information required by the planner.

---

## 7. Planning Result

The planner produces a planning result containing:

```cpp
class PlanningResult {
public:
    bool success;
    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
    double minimumSafetyDistance;
    double cumulativeReliability;
    size_t statesExplored;
    double planningTimeMs;
};
```

The result contains:

* Whether planning was successful.
* The sequence of states.
* The sequence of transitions.
* Total path cost.
* Minimum safety distance.
* Reliability.
* Number of explored states.
* Planning time.

---

## 8. Algorithm Design

The planner uses a D* Lite based graph-search approach.

D* Lite is designed for path planning in environments where the graph can change.

The algorithm maintains two values for each state:

$$
g(s)
$$

and

$$
rhs(s)
$$

The \(g\) value represents the current best path-cost estimate.

The \(rhs\) value represents the one-step look-ahead value.

For a state \(u\):

$$
rhs(u)=
\min_{(u,v)\in E}
\left(c(u,v)+g(v)\right)
$$

The priority key is calculated as:

$$
Key(u)=
\left[
\min(g(u),rhs(u))+h(u),
\min(g(u),rhs(u))
\right]
$$

where \(h(u)\) is the heuristic estimate.

---

## 9. Data Structures

The implementation uses several data structures.

### 9.1 State Map

A hash map stores states:

```cpp
unordered_map<uint64_t, State> stateMap;
```

It provides fast state lookup.

### 9.2 Outgoing Edges

Outgoing transitions are stored using:

```cpp
unordered_map<uint64_t, vector<Transition>> outgoing;
```

This allows the planner to find transitions leaving a state.

### 9.3 Incoming Edges

Incoming transitions are stored using:

```cpp
unordered_map<uint64_t, vector<Transition>> incoming;
```

This is useful when updating predecessor states.

### 9.4 Priority Queue

A priority queue stores states that need processing.

It is used by the D* Lite search procedure.

### 9.5 Bad-State Set

Bad states are stored in a hash set:

```cpp
unordered_set<uint64_t> badSet;
```

This allows efficient bad-state checking.

---

## 10. Heuristic Function

The heuristic estimates the distance between the current state and the goal.

The Euclidean distance is:

$$
h(s)=
\sqrt{
\sum_{i=1}^{d}
(s_i-G_i)^2
}
$$

where \(G\) represents the goal coordinates.

The Euclidean heuristic is suitable because the states are embedded in Cartesian space.

---

## 11. Safety Computation

The planner calculates the distance from every state to its nearest bad state.

For a state \(s\):

$$
D(s)=
\min_{b\in B}
d(s,b)
$$

For a complete path:

$$
D(P)=
\min_{s\in P}D(s)
$$

Thus, \(D(P)\) represents the minimum safety margin of the path.

A larger minimum safety distance indicates that the path stays farther away from bad states.

---

## 12. Bad-State Avoidance

Bad states are treated as hard constraints.

If a transition leads to a bad state:

$$
to(e)\in B
$$

the transition is rejected.

Its effective cost is treated as:

$$
c(e)=\infty
$$

Therefore, the planner cannot select a transition that intentionally enters a bad state.

The expected number of bad states visited by a successful path is:

$$
0
$$

---

## 13. Safety-Aware Cost Function

The planner combines transition cost, safety distance, and reliability.

The effective transition cost is represented as:

$$
C'(e)=
C(e)
+
\frac{\lambda}{D+\epsilon}
+
\frac{0.1}{R(e)}
$$

where:

* \(C(e)\) = original transition cost
* \(D\) = distance to the nearest bad state
* \(\lambda\) = safety weight
* \(\epsilon\) = small positive value
* \(R(e)\) = transition reliability

The reciprocal safety-distance term produces a larger penalty when the state is close to a bad state.

Therefore, between otherwise valid paths, the planner can prefer a path with a larger safety margin.

---

## 14. Reliability

Reliability is associated with each transition.

For a path containing transitions \(e_1,e_2,\ldots,e_n\), cumulative reliability can be represented as:

$$
R(P)=
\prod_{i=1}^{n}R(e_i)
$$

For example, if three transitions each have reliability 0.95:

$$
R(P)=
0.95\times0.95\times0.95
$$

$$
R(P)=0.857375
$$

A higher value indicates a more reliable sequence of transitions.

---

## 15. Path Reconstruction

After the search determines that the goal is reachable, the path is reconstructed starting from the initial state.

At every state, the planner examines available successors and selects the successor that minimizes:

$$
C'(e)+g(successor)
$$

The resulting state sequence is stored in `statePath`.

The transition IDs are stored separately in `transitionPath`.

For example:

```text
State Path:
0 -> 1 -> 2 -> 3

Transition Path:
0 -> 1 -> 2
```

---

## 16. Dynamic Environment

The planning environment may change after an initial solution is found.

Possible changes include:

1. A transition becomes unavailable.
2. A new transition is added.
3. The goal state changes.
4. Bad states are changed.
5. Existing transitions are removed.

The planner responds by calculating a new path using the updated planning problem.

The general process is:

```text
Initial Environment
        |
        v
Initial Planning
        |
        v
Environment Change
        |
        v
Updated Planning Problem
        |
        v
Replanning
        |
        v
New Safe Path
```

---

## 17. Test Case 1 – Basic Reachability

The graph contains a single valid route:

```text
S -> A -> B -> G
```

Expected path:

```text
S -> A -> B -> G
```

The planner should successfully reach the goal.

The number of bad states visited should be:

```text
0
```

---

## 18. Test Case 2 – Bad-State Avoidance

The graph contains two routes.

Unsafe route:

```text
S -> A -> X -> G
```

where `X` is a bad state.

Safe route:

```text
S -> C -> D -> G
```

The transition leading to `X` is rejected.

Expected path:

```text
S -> C -> D -> G
```

The expected number of bad states visited is:

```text
0
```

---

## 19. Test Case 3 – Safety Margin

Two valid paths are provided.

The first path has lower normal transition cost but passes close to a bad state.

The second path has higher transition cost but maintains a greater distance from bad states.

The safety penalty changes the effective cost of the paths.

This test demonstrates the trade-off between:

* Total transition cost.
* Safety distance.

The safety weight controls how strongly the planner prefers safer paths.

---

## 20. Test Case 4 – Dynamic Transition

Initially, the route:

```text
S -> A -> G
```

is available.

The planner first calculates the path.

Then the transition:

```text
A -> G
```

becomes unavailable.

The updated environment is then planned again.

If an alternative exists, the planner should select the alternative route.

This demonstrates adaptation to transition failure.

---

## 21. Test Case 5 – Goal Update

Initially, the planner has one goal state.

It calculates a path from the initial state to that goal.

The goal is then changed to another state.

The planner is executed again using the new goal.

The resulting path should terminate at the updated goal.

This demonstrates adaptation to changing planning objectives.

---

## 22. Test Case 6 – Transition Addition

Initially, the planner operates using the original set of transitions.

A new shortcut transition is then added.

For example:

```text
S -> A -> G
```

can be improved by adding:

```text
S -> G
```

If the shortcut has lower effective cost, the planner should select it.

This demonstrates adaptation to newly available transitions.

---

## 23. Performance Evaluation

The following metrics are used to evaluate the planner.

### Goal Success Rate

$$
SuccessRate=
\frac{SuccessfulPlans}
{TotalPlans}
\times100
$$

### Bad States Visited

The number of bad states occurring in the final path.

Expected value:

$$
0
$$

### Total Path Cost

$$
C(P)=
\sum_{e\in P}C(e)
$$

### Minimum Safety Distance

$$
D(P)=
\min_{s\in P}
\min_{b\in B}
d(s,b)
$$

### States Explored

The number of states processed by the search algorithm.

### Planning Time

The execution time required to calculate a path.

### Replanning Time

The execution time required to calculate a new path after an environmental change.

---

## 24. Time Complexity

Let:

$$
V=|S|
$$

represent the number of states and:

$$
E=|T|
$$

represent the number of transitions.

Building the graph requires:

$$
O(V+E)
$$

time.

Priority queue operations require approximately:

$$
O(\log V)
$$

time.

A complete graph-search operation has a typical upper-bound behavior of approximately:

$$
O(E\log V)
$$

depending on the number of processed states and queue operations.

For dynamic changes, the practical replanning time depends on the portion of the graph affected by the change.

---

## 25. Space Complexity

The graph representation requires:

$$
O(V+E)
$$

space.

Additional memory is required for:

* State map.
* Transition lists.
* `g` values.
* `rhs` values.
* Priority queue.
* Bad-state set.
* Auxiliary search structures.

Therefore, the overall space complexity is approximately:

$$
O(V+E)
$$

---

## 26. Advantages

The proposed planner provides:

1. Finite Cartesian state representation.
2. Directed graph support.
3. Bad-state avoidance.
4. Safety-distance calculation.
5. Reliability consideration.
6. Transition availability handling.
7. Goal update handling.
8. Transition addition handling.
9. Path-cost calculation.
10. Planning-time measurement.
11. Support for dynamic replanning experiments.

---

## 27. Limitations

The current implementation demonstrates replanning by running the planner again after the planning environment is changed.

The `plan()` function rebuilds and initializes the search structures for each planning call. Therefore, the current implementation does not fully preserve all previous search information between separate calls.

As a result, the implementation demonstrates the required replanning behavior, but it does not obtain the complete computational advantage of a persistent incremental D* Lite implementation.

This limitation should be considered when comparing initial planning time and replanning time.

---

## 28. Experimental Results

The experimental results should be obtained by running all six test cases.

The following table can be filled using the actual program output.

| Test Case   | Success | Path | Total Cost | Minimum Safety Distance | Reliability | States Explored | Planning Time |
| ----------- | ------- | ---- | ---------: | ----------------------: | ----------: | --------------: | ------------: |
| Test Case 1 | —       | —    |          — |                       — |           — |               — |             — |
| Test Case 2 | —       | —    |          — |                       — |           — |               — |             — |
| Test Case 3 | —       | —    |          — |                       — |           — |               — |             — |
| Test Case 4 | —       | —    |          — |                       — |           — |               — |             — |
| Test Case 5 | —       | —    |          — |                       — |           — |               — |             — |
| Test Case 6 | —       | —    |          — |                       — |           — |               — |             — |

The values should be copied from actual program execution.

---

## 29. User Manual

### Requirements

The program requires a C++17 compatible compiler.

No external libraries are required.

### Compilation

Open a terminal in the project directory and run:

```bash
g++ -std=c++17 -O2 safe_planner.cpp -o safe_planner
```

### Execution

Run:

```bash
./safe_planner
```

The program provides the following options:

```text
1. Test Case 1 - Basic Reachability
2. Test Case 2 - Bad State Avoidance
3. Test Case 3 - Safety Margin
4. Test Case 4 - Dynamic Transition
5. Test Case 5 - Goal Update
6. Test Case 6 - Transition Addition
7. Run All Test Cases
8. Exit
```

Selecting option 7 executes all six test cases.

---

## 30. Conclusion

A safe semantic planner was designed for a finite Cartesian state space using a D* Lite based graph-search approach.

The planner represents states using Cartesian embeddings and transitions using cost, safety, reliability, and availability information.

Bad states are treated as hard constraints, ensuring that successful paths avoid forbidden states. The distance to the nearest bad state is incorporated into the effective transition cost so that paths passing close to bad states receive an additional penalty.

The planner also demonstrates adaptation to changes in the planning environment, including transition availability changes, goal updates, and transition additions.

The six test cases provide a basis for evaluating reachability, safety, dynamic planning, path cost, reliability, explored states, and execution time.

The implementation can be extended in future work to provide fully persistent incremental replanning, multi-goal planning, time-dependent transitions, parallel search, learning-based heuristics, and knowledge-graph integration.

---

## 31. Future Enhancements

Possible future improvements include:

* Persistent D* Lite search state between replanning operations.
* Multi-goal planning.
* Time-dependent transition availability.
* Parallel search.
* Learning-based heuristic functions.
* Dynamic safety models.
* Larger randomly generated graphs.
* Knowledge-graph based planning.
* Visualization of states and paths.
* More advanced multi-objective optimization.
