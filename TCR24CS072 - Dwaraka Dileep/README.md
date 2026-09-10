# Safe Semantic Planner in a Finite Cartesian State Space

**PCCST503 – Machine Learning, Assignment 1**

A C++17 implementation of a safe planning algorithm over a finite set of
states embedded in `R^d`. Given a start state, a goal state, a set of
"bad" states to avoid, and a set of directed, weighted, possibly-unreliable
transitions, the planner computes a path that reaches the goal while never
visiting a bad state, minimizing cost, and maximizing clearance from danger
— and can efficiently **replan** when the environment changes (transitions
added/removed/toggled, or the goal itself moving).

## Algorithm

The planner is built on **LPA\* (Lifelong Planning A\*)**, chosen because
its whole reason for existing is exactly the assignment's dynamic-replanning
requirement: it incrementally repairs a shortest-path solution after
localized graph changes instead of resolving from scratch. See
[`report/Design_Report.md`](report/Design_Report.md) for the full
justification, complexity analysis, and discussion of what is — and isn't —
incrementally cheap.

## Repository layout

```
include/            Public headers: State, Transition, PlanningProblem,
                     PlanningResult, Planner (abstract interface), LPAStar.
src/                 LPAStar.cpp (the algorithm) and main.cpp (CLI demo).
tests/               Automated suite covering all six assignment test cases.
experiments/         Randomized evaluation harness + results.csv output.
docs/User_Manual.md  How to build, run, and use the planner.
report/Design_Report.md  Design report (state repr., data structures,
                     heuristic, safety computation, complexity, dynamic
                     environment handling).
```

## Quick start

```bash
mkdir build && cd build
cmake ..
cmake --build .

./planner_demo          # runs a worked example + two live replans
ctest                   # or: ./planner_tests
./run_experiments        # writes experiments/results.csv
```

No external dependencies beyond a C++17 compiler and CMake — see
[`docs/User_Manual.md`](docs/User_Manual.md) for details, including a
no-CMake fallback build command.

## Deliverables checklist

- [x] C++ source code — `include/`, `src/`
- [x] Design report — `report/Design_Report.md`
- [x] Experimental results — `experiments/run_experiments.cpp`, `experiments/results.csv`
- [x] User manual — `docs/User_Manual.md`
- [ ] Demonstration — record a short walkthrough of `./planner_demo` and `./planner_tests` (see User Manual)

## Status of bonus items

Not attempted in this submission; candidates for future work are listed at
the end of the design report (multi-goal planning, time-dependent
availability, parallel search, learning-based heuristic, knowledge-graph
test).
