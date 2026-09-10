# Safe Semantic Planner — D* Lite

A C++17 implementation of a safety-aware path planner for finite Cartesian state spaces using the **D\* Lite** algorithm.

**Course:** PCCST503 — Machine Learning, Assignment 1

## Features

| Feature | Description |
|---------|-------------|
| **D\* Lite search** | Optimal path planning with Euclidean heuristic |
| **Safety-aware cost** | Penalises paths near bad states via Euclidean distance |
| **Bad-state avoidance** | Hard constraint — bad states are never visited |
| **Dynamic replanning** | Edge availability, additions, removals, bad-state changes |
| **Goal updates** | Full re-initialisation when the goal changes |
| **Multi-goal planning** *(bonus)* | Evaluates multiple candidate goals, picks the best by score |
| **Incremental replanning** *(bonus)* | Reuses prior search state for ~300× faster replans |

## Building

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
```

Requires: CMake ≥ 3.14, C++17 compiler (GCC 9+, Clang 10+, MSVC 2019+).

## Running

```bash
./planner
```

This executes all 6 required test cases (TC1–TC6) plus bonus demonstrations
and prints a metrics table:

- Goal success (YES/NO)
- Bad states visited (expected 0)
- Total path cost
- Safety score (min distance to nearest bad state)
- States explored
- Planning time (ms)
- Memory usage (bytes, approximate)
- Full path

## Project Structure

```
├── CMakeLists.txt        Build system
├── README.md             This file
└── src/
    ├── types.h           State, Transition, PlanningProblem, PlanningResult
    ├── planner.h         D* Lite planner class declaration
    ├── planner.cpp        D* Lite implementation
    ├── test_runner.h      Test runner declaration
    ├── test_runner.cpp    All test cases + metrics collection
    └── main.cpp           Entry point
```

## Algorithm

### Scoring Function

```
Score(P) = αG − βC + γD + δR
```

| Symbol | Meaning | Default Weight |
|--------|---------|----------------|
| G | Goal completion (1 or 0) | α = 100 |
| C | Cumulative transition cost | β = 1.0 |
| D | Min Euclidean distance to nearest bad state | γ = 2.0 |
| R | Cumulative reliability | δ = 0.5 |

### Effective Edge Cost

Cost and safety are unified into a single edge weight:

```
effective_cost(s→s') = β·cost + γ·max(0, threshold − dist_to_bad(s')) + δ·(1 − reliability)
```

### Heuristic

Euclidean distance in ℝ^d — admissible and consistent.

### Replanning Complexity

| Change Type | Cost |
|-------------|------|
| Edge cost/availability | O(affected · log n) |
| New/removed transition | O(log n) |
| Bad-state add/remove | O(affected edges · log n) |
| Goal change | O(n log n) — full reinit |

## Tuning

Edit the constants at the top of `src/planner.h`:

```cpp
constexpr double ALPHA            = 100.0;
constexpr double BETA             =   1.0;
constexpr double GAMMA            =   2.0;
constexpr double DELTA            =   0.5;
constexpr double SAFETY_THRESHOLD =   5.0;
```

## Test Cases

| # | Name | Tests |
|---|------|-------|
| TC1 | Basic Reachability | Unique path S→A→B→G |
| TC2 | Bad State Avoidance | Path through bad state vs safe alternative |
| TC3 | Safety Margin | Cost vs. safety trade-off |
| TC4 | Dynamic Transition | Edge becomes unavailable → replan |
| TC5 | Goal Update | Goal changes mid-execution |
| TC6 | Transition Addition | New shortcut inserted |
| Bonus | Multi-Goal | Best goal selected from candidates |
| Bonus | Incremental Replan | Timing comparison: full plan vs replan |
