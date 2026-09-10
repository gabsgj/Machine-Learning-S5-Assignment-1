# PCCST503 – Safe Semantic Planner

**Name:** MUHAMMED JIYAD U  
**University Registration Number:** LTCR24CS074

## Machine Learning Assignment 1

**Design of a Safe Semantic Planner in a Finite Cartesian State Space**

This repository contains the complete implementation and documentation for PCCST503 Assignment 1. The project implements a safe graph planner for a finite Cartesian state space, with support for cost optimization, safety-aware path selection, reliability, and dynamic replanning.

The assignment requires the planner to reach the goal, never visit bad states, minimize transition cost, maximize the minimum distance from bad states, and operate within reasonable execution time. It also requires consideration of dynamic changes such as goal updates, bad-state updates, transition availability changes, additions, and removals.

## Features

- Finite Cartesian state representation
- Directed transition graph
- Hard avoidance of bad states
- Transition cost optimization
- Safety-margin optimization
- Transition reliability handling
- LPA* incremental graph search
- Heuristic based on reverse shortest-hop distance
- Dynamic transition availability updates
- Dynamic transition insertion
- Dynamic goal updates
- Experimental test cases corresponding to the assignment specification
- C++17 implementation with no third-party dependencies

## Algorithm

The implementation uses **Lifelong Planning A\* (LPA\*)**.

LPA* maintains `g` and `rhs` values and incrementally repairs the shortest-path solution after graph changes. This makes it suitable for the assignment's dynamic environment.

### Safe transition objective

For an available transition `e = (u, v)`, the implementation uses an additive scalar objective of the form:

`w(e) = cost(e) + λs/(d(v,B)+ε) + λs(1-safety(e)) + λr(1-reliability(e))`

where:

- `cost(e)` is the transition cost.
- `d(v,B)` is the Euclidean distance from the destination state to the nearest bad state.
- `safety(e)` is the transition safety score.
- `reliability(e)` is the transition reliability.
- `λs` controls the safety emphasis.
- `λr` controls the reliability emphasis.

Bad states remain a **hard constraint** and are never allowed as path states.

## Repository Structure

```text
PCCST503-Safe-Semantic-Planner/
│
├── README.md
├── LICENSE
├── .gitignore
│
├── src/
│   └── safe_planner.cpp
│
├── docs/
│   ├── design_report.docx
│   └── user_manual.docx
│
├── results/
│   └── experimental_results.csv
│
└── screenshots/
    └── screenshots
```

## Requirements

- C++17-compatible compiler
- GCC, Clang, or MSVC
- No external libraries are required

## Build

### Linux / macOS

```bash
g++ -std=c++17 -O2 -Wall -Wextra src/safe_planner.cpp -o safe_planner
```

### Windows with MinGW

```bash
g++ -std=c++17 -O2 -Wall -Wextra src/safe_planner.cpp -o safe_planner.exe
```

## Run

Linux / macOS:

```bash
./safe_planner
```

Windows:

```powershell
.\safe_planner.exe
```

The program runs the assignment test scenarios and reports:

- Planning success
- State path
- Transition path
- Total path cost
- Minimum distance to bad states
- Number of explored states
- Planning time

## Test Cases

| Test | Scenario | Expected behaviour |
|---|---|---|
| 1 | Basic Reachability | Return the unique valid path |
| 2 | Bad State Avoidance | Reject the path containing a bad state |
| 3 | Safety Margin | Prefer the safer path when safety weighting is enabled |
| 4 | Dynamic Transition | Replan after a transition becomes unavailable |
| 5 | Goal Update | Produce a revised path after the goal changes |
| 6 | Transition Addition | Discover an improved path after adding a shortcut |

These scenarios correspond to the illustrative test cases specified in the assignment.

## Experimental Results

The recorded results are available in:

`results/experimental_results.csv`

The main outcomes are:

| Case | Result | Selected path | Cost | Minimum bad-state distance |
|---|---|---|---:|---:|
| Test 1 | Success | 1 → 2 → 3 → 4 | 3.0 | ∞ |
| Test 2 | Success | 1 → 4 → 5 → 6 | 3.1 | 1.0 |
| Test 3 | Success | 1 → 5 → 6 | 2.8 | 1.4142 |
| Test 4 | Success | 1 → 3 → 4 | 2.4 | ∞ |
| Test 5 | Success | 1 → 5 | 0.5 | ∞ |
| Test 6 | Success after addition | 1 → 2 → 4 | 1.2 | ∞ |

## Complexity

Let:

- `|V|` = number of states
- `|E|` = number of transitions
- `|B|` = number of bad states
- `d` = Cartesian embedding dimension

The adjacency-list representation requires:

- **Time:** approximately `O((|V| + |E|) log |V|)` for a full LPA* repair episode under standard assumptions.
- **Heuristic construction:** `O(|V| + |E|)` for reverse BFS.
- **Safety evaluation:** `O(|P||B|d)` for a returned path of `|P|` states using direct nearest-bad-state distance.
- **Space:** `O(|V| + |E| + |B|)`.

Incremental replanning can avoid recomputing the entire search from scratch when only a small portion of the graph changes.

## Documentation

- [`design_report.docx`](docs/design_report.docx) – full design report
- [`user_manual.docx`](docs/user_manual.docx) – build and usage instructions
- [`experimental_results.csv`](results/experimental_results.csv) – experimental data

## Demonstration

The executable test driver demonstrates the six assignment scenarios. For a formal submission demonstration, run the program and capture the terminal output showing each test result.

A place for screenshots is included under `screenshots/`.

## Limitations

The assignment's multiple objectives are handled through scalarization rather than a complete Pareto-front optimization. The direct nearest-bad-state calculation also scales linearly with the number of bad states. For a larger deployment, a spatial index such as a KD-tree and more selective affected-vertex tracking could improve performance.

## Optional Extensions

Possible extensions include:

- Multi-goal planning
- Time-dependent transition availability
- Incremental replanning improvements
- Parallel search
- Learning-based heuristic
- Knowledge-graph testing


## License

See [`LICENSE`](LICENSE).
