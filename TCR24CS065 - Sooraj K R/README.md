# Safe Semantic Planner

A Python implementation of a safe path planning algorithm in a finite Cartesian state space. This project uses A* search to find the cheapest safe path from a start state to a goal state while avoiding bad (unsafe) states.

Built for **PCCST503 - Machine Learning, Assignment 1**.

## What It Does

- Finds the shortest safe path from start to goal
- Avoids all bad (unsafe) states
- Calculates how far the path stays from bad states (safety distance)
- Computes a combined score: `Score = α·G - β·C + γ·D + δ·R`
- Supports replanning when the environment changes (transitions break, goals change, new shortcuts appear)

## System Design

```
┌─────────────────────────────────────────────────────┐
│                     main.py                         │
│         (Runs test cases, prints results)           │
└──────────────────────┬──────────────────────────────┘
                       │ uses
                       ▼
┌─────────────────────────────────────────────────────┐
│                   planner.py                        │
│                                                     │
│  ┌───────────────────────────────────────────────┐  │
│  │             SafePlanner.plan()                 │  │
│  │                                               │  │
│  │  1. Build lookup tables (state map,           │  │
│  │     adjacency list, transition map)           │  │
│  │                                               │  │
│  │  2. Run A* Search:                            │  │
│  │     - Priority queue (min-heap by f-score)    │  │
│  │     - f(n) = g(n) + h(n)                     │  │
│  │     - g(n) = actual cost from start           │  │
│  │     - h(n) = Euclidean distance to goal       │  │
│  │     - Skip bad states and unavailable edges   │  │
│  │                                               │  │
│  │  3. Reconstruct path using parent pointers    │  │
│  │                                               │  │
│  │  4. Compute metrics:                          │  │
│  │     - Total cost                              │  │
│  │     - Safety distance (min dist to bad state) │  │
│  │     - Reliability                             │  │
│  │     - Objective score                         │  │
│  └───────────────────────────────────────────────┘  │
│                                                     │
│  Replanning Functions:                              │
│  • replan_with_unavailable_transition()             │
│  • replan_with_new_goal()                           │
│  • replan_with_new_transition()                     │
└──────────────────────┬──────────────────────────────┘
                       │ uses
                       ▼
┌─────────────────────────────────────────────────────┐
│                 structures.py                       │
│                                                     │
│  Data Classes:                                      │
│  • State        - id, embedding (position)          │
│  • Transition   - id, from, to, cost, safety,       │
│                   reliability, available             │
│  • PlanningProblem - start, goal, bad states,        │
│                      all states, all transitions     │
│  • PlanningResult  - success, path, cost, safety,    │
│                      metrics                         │
└─────────────────────────────────────────────────────┘
                       ▲
                       │ uses
┌──────────────────────┴──────────────────────────────┐
│                 test_cases.py                       │
│                                                     │
│  Creates 6 test scenarios:                          │
│  1. Basic Reachability    (S → A → B → G)           │
│  2. Bad State Avoidance   (avoid state X)           │
│  3. Safety Margin         (cost vs safety tradeoff) │
│  4. Dynamic Transition    (edge becomes unavailable)│
│  5. Goal Update           (goal changes mid-run)    │
│  6. Transition Addition   (new shortcut added)      │
└─────────────────────────────────────────────────────┘
```

## Files

| File | What it does |
|------|-------------|
| `structures.py` | Data classes for State, Transition, PlanningProblem, PlanningResult |
| `planner.py` | A* search algorithm with safety distance and scoring |
| `test_cases.py` | All 6 test cases from the assignment |
| `main.py` | Runs everything and prints results |

## How to Run

```bash
python3 main.py
```

No external libraries needed. Uses only Python standard library (`heapq`, `math`, `time`).

## Test Cases

| # | Name | What it tests |
|---|------|--------------|
| 1 | Basic Reachability | Simple path S → A → B → G |
| 2 | Bad State Avoidance | Avoids bad state X, picks safe detour |
| 3 | Safety Margin | Cost vs. safety distance tradeoff |
| 4 | Dynamic Transition | Replans when a transition becomes unavailable |
| 5 | Goal Update | Replans when the goal changes |
| 6 | Transition Addition | Finds improved path when a shortcut is added |

## How Replanning Works

For dynamic environments (test cases 4-6), the approach is simple:

1. **Modify** the `PlanningProblem` (change goal, disable transition, add transition)
2. **Re-run** `planner.plan()` on the modified problem

This is the simplest approach. For large graphs, algorithms like D* Lite or LPA* can replan more efficiently by reusing previous search results, but for small graphs this works great.

## Scoring Function

```
Score(P) = α·G - β·C + γ·D + δ·R
```

| Symbol | Meaning | Default Weight |
|--------|---------|---------------|
| G | Goal reached (1 or 0) | α = 10 |
| C | Total path cost (lower is better) | β = 1 |
| D | Min distance to bad states (higher is better) | γ = 2 |
| R | Total reliability of transitions | δ = 1 |
