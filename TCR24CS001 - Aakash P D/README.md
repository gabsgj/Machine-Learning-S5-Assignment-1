# PCCST503 – Machine Learning

## Assignment 1

# Design of a Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**
**GEC Thrissur**

---

## Objective

The objective of this project is to design and implement a **Safe Path Planner** that finds a safe and efficient path from an initial state to a goal state in a finite Cartesian state space.

The planner avoids predefined bad states while considering:

* Path cost
* Safety
* Reliability
* Transition availability
* Distance from bad states

---

## Problem Definition

The environment consists of a finite set of states.

Each state is represented using a Cartesian coordinate:

```text
State = (x1, x2, ..., xd)
```

The planner receives:

* Initial state
* Goal state
* Set of bad states
* Set of directed transitions

Each transition contains:

* Cost
* Reliability
* Safety score
* Availability

The planner must find a path from the initial state to the goal without visiting any bad state.

---

## Optimization Objectives

The planner should:

1. Reach the goal state.
2. Avoid all bad states.
3. Minimize the total path cost.
4. Maintain a safe distance from bad states.
5. Consider transition reliability.
6. Complete the planning process within reasonable time.

A simple scoring function can be represented as:

```text
Score = αG - βC + γD + δR
```

Where:

* `G` = Goal completion
* `C` = Total path cost
* `D` = Minimum distance from bad states
* `R` = Path reliability

---

# State Representation

Each state contains:

```text
State
├── ID
└── Cartesian coordinates
```

Example:

```text
State 1 → (0, 0)
State 2 → (1, 0)
State 3 → (1, 1)
State 4 → (2, 1)
```

---

# Transition Representation

Each transition connects two states.

```text
Transition
├── ID
├── From state
├── To state
├── Cost
├── Safety score
├── Reliability
└── Availability
```

Example:

```text
S1 → S2
Cost: 1
Safety: 0.9
Reliability: 0.95
Available: Yes
```

---

# Planning Problem

The planning problem contains:

```text
PlanningProblem
├── Initial state
├── Goal state
├── Bad states
├── States
└── Transitions
```

---

# Planning Result

The planner produces:

```text
PlanningResult
├── Success
├── State path
├── Transition path
├── Total cost
└── Safety score
```

Example:

```text
Path:
S → A → C → G

Total Cost: 4
Safety Score: 0.87
Success: Yes
```

---

# Planner

The planner takes the planning problem as input and returns a planning result.

```text
Planner
    ↓
Planning Problem
    ↓
Path Search
    ↓
Safety Checking
    ↓
Optimal Safe Path
```

---

# Algorithm

The project can use an incremental path-planning algorithm such as:

* **LPA***
* **D* Lite**

The algorithm uses a heuristic to estimate the distance between the current state and the goal.

A simple Euclidean distance heuristic is:

```text
h(n) = √((x₂-x₁)² + (y₂-y₁)²)
```

This helps the planner find the goal efficiently.

---

# Safety Computation

For every visited state, the planner checks its distance from the nearest bad state.

```text
Safety Distance =
minimum distance to any bad state
```

A path is considered invalid if it visits a bad state.

The planner can also prefer paths that maintain a larger safety distance.

---

# Dynamic Environment

The environment can change during execution.

Possible changes include:

* Goal state changes
* Bad states are added or removed
* Transitions become unavailable
* New transitions are added
* Existing transitions are removed

The planner should replan when the environment changes.

Incremental algorithms such as **LPA*** or **D* Lite** can reduce the amount of computation required during replanning.

---

# Test Cases

## Test Case 1 – Basic Reachability

```text
S → A → B → G
```

Expected result:

```text
S → A → B → G
```

The planner should find the valid path.

---

## Test Case 2 – Bad State Avoidance

Available paths:

```text
S → A → X → G
```

where `X` is a bad state.

Alternative:

```text
S → C → D → G
```

Expected result:

```text
S → C → D → G
```

The planner must avoid `X`.

---

## Test Case 3 – Safety Margin

Two valid paths are available.

```text
Path 1 → Lower cost, lower safety
Path 2 → Higher cost, higher safety
```

The planner should balance cost and safety when selecting the path.

---

## Test Case 4 – Dynamic Transition

Initially:

```text
S → A → G
```

Later:

```text
A → G
```

becomes unavailable.

Expected result:

The planner should find an alternative path.

---

## Test Case 5 – Goal Update

The goal changes during execution.

Expected result:

The planner should calculate a new path to the updated goal.

---

## Test Case 6 – New Transition

A new shortcut is added.

```text
S → A → G
```

Expected result:

The planner should discover the new and improved path.

---

# Evaluation

The following parameters can be measured:

| Parameter               | Description                               |
| ----------------------- | ----------------------------------------- |
| Goal Success Rate       | Percentage of successful plans            |
| Bad States Visited      | Should be zero                            |
| Total Path Cost         | Cost of selected path                     |
| Minimum Safety Distance | Distance from nearest bad state           |
| Explored States         | Number of states searched                 |
| Planning Time           | Time required to find a path              |
| Memory Usage            | Memory used by the planner                |
| Replanning Time         | Time required after an environment change |

---

# Project Structure

```text
safe_path_planner/
│
├── src/
│   ├── main.cpp
│   ├── planner.cpp
│   ├── planner.h
│   ├── state.cpp
│   ├── state.h
│   ├── transition.cpp
│   └── transition.h
│
├── tests/
│   ├── test1.cpp
│   ├── test2.cpp
│   └── test3.cpp
│
├── README.md
│
└── CMakeLists.txt
```

---

# How to Run

### 1. Clone the repository

```bash
git clone https://github.com/Aakash-P-D/safe-path-planner.git
```

### 2. Open the project

```bash
cd safe-path-planner
```

### 3. Compile

```bash
g++ src/*.cpp -o safe_path_planner
```

### 4. Run

```bash
./safe_path_planner
```

---

# Requirements

* C++17 or later
* GCC / MinGW
* Git
* CMake (optional)

---

# Future Enhancements

Possible future improvements:

* Interactive path visualization
* Dynamic obstacle updates
* Incremental replanning
* Multi-goal planning
* Time-dependent transitions
* Parallel search
* Learning-based heuristic
* Web-based visualization
* Knowledge graph integration

---

# Applications

Safe path planning can be used in:

* Robot navigation
* Autonomous vehicles
* Drone navigation
* Route planning
* Game AI
* Warehouse robots
* Emergency evacuation
* Smart transportation systems

---

# Author

**Aakash P D**
**TCR24CS001**

---

# License

This project is licensed under the **MIT License**.
