# Safe Semantic Planner

C++17 implementation of **LPA\* (Lifelong Planning A\*)** for PCCST503 (Machine
Learning) Assignment 1 — a safe path planner over a finite Cartesian state
space that avoids bad states, minimizes cost, and replans efficiently when
the goal, bad states, or transition graph change.

## Contents

```
include/            State, Transition, PlanningProblem, PlanningResult,
                     Planner interface, LPAStarPlanner
src/main.cpp         Runs the assignment's 6 illustrative test cases
src/experiment.cpp    Scaling benchmark: incremental vs cold replanning
src/bonus_tests.cpp    Multi-goal planning, time-dependent availability,
                       and a knowledge-graph demo
experiment_results.csv  Benchmark output referenced in the design report
WINDOWS_BUILD.md      Windows build instructions (MinGW / MSVC)
.vscode/               Build + debug tasks for VS Code (MinGW toolchain)
```

## Bonus features implemented

- **Incremental replanning** — inherent to the core LPA* design (see below).
- **Multi-goal planning** — `planMultiGoal()` finds the cheapest reachable
  goal among several candidates.
- **Time-dependent transition availability** — edges can be restricted to a
  `[availableFrom, availableUntil)` time window; `setCurrentTime()` updates
  the plan accordingly.
- **Tested on a knowledge graph** — a small non-geometric ML-pipeline concept
  graph, confirming the planner isn't tied to spatial data.

Run `make bonus_tests && ./bonus_tests` to see all three in action.

## Build & run

**VS Code (Windows, MinGW):** open this folder in VS Code, install the
`C/C++` and `C/C++ Extension Pack` extensions, then press `Ctrl+Shift+B` to
build, or `F5` to debug. See `WINDOWS_BUILD.md` if you're setting up MinGW
for the first time.

**Command line (Linux/macOS/MSYS2):**
```
make
./safe_planner_test
./experiment > experiment_results.csv
```

## Design notes

Full design rationale (state representation, heuristic choice, safety-cost
weighting, complexity analysis, and experimental results) is in the
accompanying design report submitted with this assignment.

Headline result: incremental replanning after a single environment change
is **40–220× faster** than replanning from scratch, measured across grid
graphs from 100 to 2,500 states.
