# Safe Path Planning Using A* and LPA*

## Assignment

**Name:** Abhirami S  
**Roll No:** 3  
**Register No:** TCR24CS003

## Project Description

This project implements a safe path planning system for finding a path from a starting state to a goal state while avoiding dangerous or forbidden states.

The system considers:

* Path cost
* State safety
* Transition reliability
* Transition availability
* Dynamic changes in the environment

The project demonstrates initial path planning and dynamic replanning when the environment changes.

## Problem Definition

The planner is given a graph containing different states and transitions between them.

Each transition has:

* Cost
* Reliability
* Safety score
* Availability

Some states are marked as **bad states** and cannot be visited.

The objective is to find a path from the start state `S` to the goal state `G` while avoiding bad states and maintaining a reasonable path cost and safety.

## State Representation

Each state is represented using Cartesian coordinates `(x, y)`.

The states used in this project are:

| State | Coordinates |
| ----- | ----------- |
| S     | (0, 0)      |
| A     | (2, 3)      |
| B     | (2, 0)      |
| C     | (3, 5)      |
| D     | (4, 0)      |
| E     | (5, 3)      |
| G     | (7, 0)      |
| X     | (3, 1)      |

Here:

* `S` is the initial state.
* `G` is the goal state.
* `X` is initially a bad state.

The Cartesian coordinates are used to calculate the distance between states and determine the safety of a path.

## Distance Calculation

Euclidean distance is used between two states.

```text
d = √((x₂ - x₁)² + (y₂ - y₁)²)
```

The distance from a state to the nearest bad state is used to evaluate path safety.

## Graph Structure

The graph used in the experiment is:

```text
S → A
S → B

A → C
A → X

B → C
B → D

C → E
C → G

D → E

E → G

X → G
```

The transitions contain information about cost, reliability, safety score, and availability.

## Initial Bad State

Initially:

```text
Bad State = X
```

The planner is not allowed to enter `X`.

Therefore, the path:

```text
S → A → X → G
```

is invalid.

A valid path is selected while avoiding the bad state.

## A* Algorithm

A* is used for path planning.

The evaluation function is:

```text
f(n) = g(n) + h(n)
```

where:

* `g(n)` is the cost from the start state to the current state.
* `h(n)` is the estimated cost from the current state to the goal.
* `f(n)` is the total estimated cost.

The heuristic used in this project is the Euclidean distance between the current state and the goal.

## LPA* Algorithm

Lifelong Planning A* (LPA*) is used for dynamic replanning.

LPA* is useful when the environment changes after an initial path has been found.

Instead of treating every change as a completely new problem, LPA* can update the existing search information and find a new path.

The project demonstrates replanning when:

1. A transition becomes unavailable.
2. A previously safe state becomes a bad state.

## Initial Planning

Initially, the environment contains:

```text
Bad State:
X
```

The planner searches for a path from:

```text
Start = S
Goal  = G
```

A valid initial route is selected while avoiding `X`.

## Environment Change 1

The first environmental change is:

```text
C → G becomes unavailable
```

This means the planner can no longer use the transition:

```text
C → G
```

The planner must find an alternative route.

For example:

```text
S → A → C → E → G
```

or another valid route depending on the path evaluation.

This demonstrates **dynamic replanning**.

## Environment Change 2

The second environmental change is:

```text
C becomes a bad state
```

The bad states are now:

```text
X
C
```

The planner must avoid both states.

A valid alternative route is:

```text
S → B → D → E → G
```

This demonstrates how the planner responds when a state that was previously usable becomes unsafe.

## Transition Information

The transitions in the graph contain the following information:

```text
Cost
Reliability
Safety
Availability
```

For example:

```text
S → A
Cost        = 2
Reliability = 0.95
Safety      = 0.90
Available   = 1
```

The same information is provided for the other transitions in the program.

## Path Cost

The total path cost is calculated by adding the costs of all transitions in the selected path.

For example:

```text
S → A → C → G

S → A = 2
A → C = 2
C → G = 5

Total Cost = 2 + 2 + 5
           = 9
```

## Path Reliability

Path reliability is calculated by multiplying the reliability values of the transitions.

For example:

```text
S → A → C → G

= 0.95 × 0.95 × 0.90
= 0.81225
```

A higher value indicates a more reliable path.

## Path Safety

The minimum distance between any state in the selected path and the nearest bad state is used as the minimum safety distance.

A larger minimum distance indicates that the path stays farther away from dangerous states.

## Performance Measures

The program records the following performance measures:

* Total path cost
* Minimum safety distance
* Path reliability
* Number of states explored
* Planning time
* Replanning time

These measurements can be used to evaluate the performance of the path planning algorithm.

## Technologies Used

* C++
* A* Search
* LPA* Dynamic Path Planning
* Graph Search
* Cartesian State Space

## How to Run

### 1. Clone the Repository

```bash
git clone https://github.com/Abbhiiraamii/safe-path-planner.git
```

### 2. Open the Project

```bash
cd safe-path-planner
```

### 3. Compile the Program

```bash
g++ safe_path_planner.cpp -o safe_path_planner
```

### 4. Run the Program

```bash
./safe_path_planner
```

## Expected Demonstration

The program demonstrates the following sequence:

```text
                 INITIAL ENVIRONMENT

                  S → A → C → G
                         ↓
                  Initial path found
                         ↓
              C → G becomes unavailable
                         ↓
                    REPLANNING
                         ↓
                  S → A → C → E → G
                         ↓
                    C becomes BAD
                         ↓
                    REPLANNING
                         ↓
                  S → B → D → E → G
```

## Conclusion

This project demonstrates safe path planning in a dynamic graph environment using A* and LPA* concepts.

The planner finds an initial path while avoiding bad states and responds to changes in the environment by finding alternative routes.

The use of Cartesian coordinates allows the planner to calculate distances from bad states and evaluate path safety. Dynamic changes in transition availability and state conditions demonstrate the importance of replanning in changing environments.
