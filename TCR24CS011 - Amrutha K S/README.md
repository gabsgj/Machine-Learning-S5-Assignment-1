# PCCST503 – Machine Learning

## Assignment 1: Design of a Safe Semantic Planner in a Finite Cartesian State Space

## Overview

This project implements a safe semantic planner for a finite Cartesian state space using a D* Lite based graph search approach.

The planner computes a path from an initial state to a goal state while avoiding bad states and considering transition cost, safety, reliability, and transition availability.

The implementation also demonstrates replanning when the environment changes, such as when transitions become unavailable, the goal changes, or a new transition is added.

## Objectives

The planner is designed to:

* Reach the goal state.
* Never visit a bad state.
* Minimize total transition cost.
* Prefer paths with larger distances from bad states.
* Consider transition reliability.
* Handle changes in the planning environment.
* Measure planning performance.

## Algorithm

The implementation uses a D* Lite based planning algorithm.

Each state maintains:

* `g` value
* `rhs` value

A priority queue stores inconsistent states. The planner repeatedly updates states until a safe path from the initial state to the goal state is obtained.

The heuristic function is the Euclidean distance between two states.

## Safety Model

Bad states are treated as hard constraints.

Any transition leading to a bad state is rejected.

The distance from a state to the nearest bad state is calculated using Euclidean distance.

The effective transition cost is:

C'(e) = C(e) + λ / (D + ε) + 0.1 / R(e)

where:

* `C(e)` is the transition cost.
* `D` is the distance to the nearest bad state.
* `R(e)` is transition reliability.
* `λ` is the safety weight.
* `ε` avoids division by zero.

This penalizes paths that pass close to bad states.

## Features

* Cartesian state representation.
* Directed graph transitions.
* Euclidean heuristic.
* Bad-state avoidance.
* Safety-aware path selection.
* Reliability-aware transition cost.
* Transition availability handling.
* Dynamic replanning demonstrations.
* Planning time measurement.
* Explored state count.
* Six assignment test cases.

## Project Structure

```text
safe-semantic-planner/
├── README.md
├── safe_planner.cpp
├── report/
│   └── design_report.md
└── results/
    └── experimental_results.md
```

## Requirements

* C++17 compiler
* Linux, Windows, or macOS
* No external libraries are required

## Compilation

Compile the program using:

```bash
g++ -std=c++17 -O2 safe_planner.cpp -o safe_planner
```

For debugging:

```bash
g++ -std=c++17 -Wall -Wextra -g safe_planner.cpp -o safe_planner
```

## Running the Program

On Linux or macOS:

```bash
./safe_planner
```

The program displays the following menu:

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

## Test Cases

### Test Case 1 – Basic Reachability

Graph:

```text
S → A → B → G
```

Expected result:

```text
S → A → B → G
```

### Test Case 2 – Bad State Avoidance

Unsafe path:

```text
S → A → X → G
```

where `X` is a bad state.

Safe path:

```text
S → C → D → G
```

Expected result:

```text
S → C → D → G
```

### Test Case 3 – Safety Margin

Two valid paths are available.

* Path 1 has lower cost but passes close to a bad state.
* Path 2 has higher cost but remains farther from the bad state.

The safety-aware cost function is used to balance cost and safety.

### Test Case 4 – Dynamic Transition

Initially:

```text
S → A → G
```

The planner first finds this path.

Then transition `A → G` becomes unavailable.

The planner computes an alternative path.

### Test Case 5 – Goal Update

The planner first computes a path to the original goal.

The goal is then changed.

The planner computes a revised path to the new goal.

### Test Case 6 – Transition Addition

The planner first computes a path using the available transitions.

A new shortcut transition is then added.

The planner recomputes the path and selects the improved route if it has lower effective cost.

## Output

For each test case, the program displays:

```text
Success
State Path
Transition Path
Total Cost
Minimum Safety Distance
Cumulative Reliability
States Explored
Planning Time
Bad States Visited
```

Example:

```text
========================================
           PLANNING RESULT
========================================
Success              : YES
State Path            : 0 -> 1 -> 2 -> 3
Transition Path       : 0 -> 1 -> 2
Total Cost            : 3.0000
Minimum Safety Dist.  : 0.0000
Cumulative Reliability: 0.8574
States Explored       : 4
Planning Time (ms)    : 0.0350
Bad States Visited    : 0
========================================
```

## Evaluation Metrics

The implementation evaluates:

* Goal success rate
* Number of bad states visited
* Total path cost
* Minimum distance to bad states
* Number of explored states
* Planning time
* Replanning behavior after environment changes

## Limitations

The current implementation demonstrates replanning by solving the updated planning problem after each change. It rebuilds the search data structures for each `plan()` call rather than preserving the previous search state for fully incremental D* Lite updates.

This limitation is documented so that the implementation is described accurately.

## Conclusion

The project demonstrates a safe semantic planner operating on a finite Cartesian state space. It successfully avoids bad states, incorporates safety and reliability into path selection, and demonstrates adaptation to dynamic changes such as transition failures, goal updates, and newly added transitions.

The implementation provides a complete framework for evaluating safe graph-based planning algorithms in a finite state space.

## Author

**Name:** Amrutha
**Course:** PCCST503 – Machine Learning
**Assignment:** Assignment 1 – Design of a Safe Semantic Planner in a Finite Cartesian State Space
