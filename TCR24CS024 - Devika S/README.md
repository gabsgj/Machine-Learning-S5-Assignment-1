# Safe Semantic Planner in a Finite Cartesian State Space

A C++ implementation of a safe graph-based planner for a finite Cartesian state space, developed for **PCCST503 – Machine Learning, Assignment 1**.

## Objective

The planner computes a path from an initial state to a goal state while:

- avoiding bad states
- considering transition cost
- considering transition safety
- considering transition reliability
- calculating minimum Euclidean distance from bad states
- handling changes in transition availability
- handling goal updates
- discovering newly added transitions

The assignment requires a C++ source code, design report, experimental results, user manual and demonstration.

## Algorithm

This project uses a simplified **D\* Lite-style incremental graph-search approach**.

The implementation:

1. Represents states as points in Cartesian space.
2. Represents directed transitions using cost, safety, reliability and availability.
3. Removes unavailable transitions and transitions involving bad states.
4. Performs a reverse shortest-path search from the goal.
5. Uses Euclidean distance as the heuristic.
6. Selects the lowest combined transition value while constructing the final path.
7. Calculates the minimum Euclidean distance between the resulting path and bad states.
8. Re-runs planning when the environment changes.

## Project Structure

```text
safe-semantic-planner/
├── README.md
├── LICENSE
├── src/
│   └── main.cpp
├── docs/
│   └── Design_Report.md
└── results/
    └── experimental_results.md
```

## Requirements

- C++17 or later
- GCC, MinGW, Clang or another C++ compiler

## Compile

### Linux / macOS

```bash
g++ -std=c++17 -O2 src/main.cpp -o planner
```

### Windows MinGW

```bash
g++ -std=c++17 -O2 src/main.cpp -o planner.exe
```

## Run

Linux/macOS:

```bash
./planner
```

Windows:

```bash
planner.exe
```

## Test Cases

The program includes the six test cases from the assignment:

1. Basic Reachability
2. Bad State Avoidance
3. Safety Margin
4. Dynamic Transition
5. Goal Update
6. Transition Addition

## Output Metrics

The program reports:

- State path
- Transition path
- Total path cost
- Minimum safety distance
- Number of explored states
- Planning time

## Complexity

For the underlying graph search, the priority-queue implementation has approximately:

**Time:** `O((V + E) log V)`

**Space:** `O(V + E)`

where `V` is the number of states and `E` is the number of available transitions.
