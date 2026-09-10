Joseph Mathew — Register No: TCR24CS038

Semantic Planner

This repository contains a C++ LPA\* semantic planner for a finite Cartesian state space.

Build and run (PowerShell):

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
..\Release\semantic_planner.exe
```

The demo runs all six assignment test cases: reachability, bad-state avoidance,
safety margin selection, dynamic transition removal, goal updates, and transition addition.

**Design Report**

- **State representation:** `State` stores `tid` and `embedding` (vector<double>) representing coordinates in R^d.
- **Transitions:** `Transition` stores `from`, `to`, `cost`, `safety`, `reliability`, `available`.
- **Problem container:** `PlanningProblem` holds states, transitions, bad states, and an adjacency map built by `buildAdjacency()`.
- **Planner:** `LPAStarPlanner` implements LPA\* with `g` and `rhs` values, lexicographic priority keys, hard bad-state rejection, weighted cost, and a safety bonus (higher distance from bad states reduces effective cost). Planner returns `PlanningResult` with state/transition path, total cost, and safety score. The planner caches adjacency between calls; goal updates reuse the cache, while transition additions, cost changes, and availability changes invalidate it.
- **Heuristic:** Euclidean distance to goal on embeddings.
- **Safety computation:** For any state the planner computes the minimum Euclidean distance to the nearest bad state; this value is used as a safety bonus in edge scoring.
- **Complexity:** LPA\* search is O(E log V) in the worst case; rebuilding the transition cache is O(E). Space is O(V+E) for graph, search, and cached adjacency data.

**Experimental Results**

- Tests included: all six assignment test cases.
- The executable also reports goal success rate, bad states visited, total cost, minimum safety distance, cumulative reliability, explored states, planning time, and replanning time.
- Example run output (from local build):
  - Test1: Basic Reachability — Success: 1, cost=3, path: 1 2 3 4
  - Test2: Bad State Avoidance — Success: 1, cost=3.2, path: 1 4 5 6
  - Test3: Safety Margin — Success: 1, cost=4.5, path: 1 4 5 9
  - Test4: Dynamic Transition — Success: 1, cost=4, path: 1 3 4
  - Test5: Goal Update — Success: 1, cost=2, path: 1 3 5
  - Test6: Transition Addition — Success: 1, cost=1, path: 1 4

**How to build & run (PowerShell, from project root)**

1. Quick build with `g++` (no CMake required):

```powershell
g++ -std=c++17 -I src src\main.cpp src\LPAStarPlanner.cpp -O2 -o semantic_planner.exe
.\semantic_planner.exe
```

2. Or with CMake (recommended):

```powershell
mkdir build
cd build
cmake ..
cmake --build . --config Release
# Run from project root:
..\build\Release\semantic_planner.exe
```
