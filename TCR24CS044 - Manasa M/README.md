# Safe Semantic Planner in a Finite Cartesian State Space

## Overview

This project implements a **Safe Semantic Planner** using the **A* (A-star) search algorithm** in Python. The planner computes a safe path from an **initial state** to a **goal state** in a finite Cartesian state space while avoiding bad states, minimizing transition cost, and preferring safer paths that remain farther away from dangerous states.

The implementation is based on graph search and heuristic optimization and demonstrates dynamic replanning when the environment changes.

## Objective

The objective of this project is to design and implement a planning algorithm that:

* Finds a valid path from the initial state to the goal state.
* Avoids all bad states.
* Minimizes total transition cost.
* Maximizes the minimum distance from bad states.
* Supports dynamic replanning when transitions or goals change.

## Features

* Cartesian state representation
* Directed weighted transitions
* A* heuristic search
* Safety-aware path planning
* Bad state avoidance
* Dynamic transition updates
* Goal state updates
* Path reconstruction
* Evaluation metrics
* Planning time measurement

## Project Structure

```text
SafeSemanticPlanner/
│── planner.py      # Main Python implementation
│── README.md       # Project documentation
```

## Requirements

* Python 3.8 or above

No external libraries are required. The program uses only Python standard libraries:

* math
* heapq
* time
* dataclasses

## How to Run

1. Open Command Prompt or VS Code Terminal.
2. Navigate to the project folder.
3. Run:

```bash
python planner.py
```

The program automatically executes all test cases and displays the computed paths and evaluation metrics.

## State Representation

Each state is represented by:

* State ID
* X coordinate
* Y coordinate

Example:

```text
S = (0,0)
A = (1,0)
G = (3,2)
```

## Transition Representation

Each transition contains:

* Source state
* Destination state
* Cost
* Safety score
* Reliability
* Availability flag

The availability flag enables dynamic environmental updates.

## Algorithm

The planner uses the **A* search algorithm**.

Evaluation function:

```text
f(n) = g(n) + h(n)
```

where:

* **g(n)** = accumulated path cost
* **h(n)** = Euclidean distance to the goal

A **safety penalty** is added to discourage paths that pass close to bad states.

Safety penalty:

```text
Penalty = 5 / (SafetyDistance + 0.001)
```

This encourages the planner to choose safer routes.

## Test Cases

The implementation includes six test cases.

### Test Case 1: Basic Reachability

Graph:

```text
S → A → B → G
```

Expected result:

```text
S → A → B → G
```

### Test Case 2: Bad State Avoidance

Bad state:

```text
X
```

Paths:

```text
S → A → X → G
S → C → D → G
```

Expected result:

```text
S → C → D → G
```

### Test Case 3: Safety Margin

Two valid paths exist.

The planner prefers the path with a larger safety margin from bad states.

### Test Case 4: Dynamic Transition

A transition becomes unavailable.

Expected result:

The planner computes an alternative path.

### Test Case 5: Goal Update

The goal state changes during execution.

Expected result:

The planner generates a revised path.

### Test Case 6: Transition Addition

A new shortcut transition is added.

Expected result:

The planner discovers the improved solution.

## Evaluation Metrics

For each test case, the program reports:

* Goal reached
* Bad states visited
* Total cost
* Minimum safety distance
* Explored states
* Planning time
* Path length

## Complexity Analysis

| Operation          | Complexity |
| ------------------ | ---------- |
| Graph construction | O(E)       |
| Safety computation | O(B)       |
| A* search          | O(E log V) |
| Space complexity   | O(V + E)   |

where:

* **V** = number of states
* **E** = number of transitions
* **B** = number of bad states

## Advantages

* Efficient heuristic search
* Safe path generation
* Dynamic replanning support
* Modular and extensible design
* Suitable for AI planning and robotic navigation

## Future Enhancements

* LPA* implementation
* D* Lite replanning
* Multi-goal planning
* Time-dependent transition availability
* Learning-based heuristic
* Parallel search
* Knowledge graph integration

## Conclusion

The project successfully implements a **Safe Semantic Planner** using the **A* search algorithm**. The planner computes safe and efficient paths while avoiding bad states and optimizing both cost and safety. Dynamic transition updates and goal changes are handled effectively, demonstrating the applicability of heuristic search techniques for planning problems in finite Cartesian state spaces.

## Author

**Manasa M**
B.Tech Computer Science and Engineering
Government Engineering College, Thrissur
Kerala Technological University (KTU)
