# SafePath — A Safe Semantic Planner in a Finite Cartesian State Space

**PCCST503 — Machine Learning, Assignment 1**

A C++17 implementation of a **D\* Lite**-based planner that computes a safe,
cost/safety/reliability-optimized path through a finite, directed, Cartesian-
embedded state graph — while never visiting a "bad" state — and that
**replans incrementally** (not from scratch) when the goal, bad states, or
transitions change at runtime. Python is used strictly as a downstream
visualization layer over data the C++ program exports; no planning logic
exists in Python.

## Student Details

Name: Catherine Maria Benny

Register Number: TCR24CS020

Course: Machine Learning

## Assignment Objective

Reach a goal state from an initial state while (1) never visiting a bad
state, (2) minimizing total transition cost, (3) maximizing the minimum
Euclidean distance from the path to the nearest bad state, and (4) doing all
of this — and replanning after dynamic changes — within reasonable time.
Full problem statement: `PCCST503_Assignment_1.pdf` (as given).

## Features

- Genuine D\* Lite: `g`/`rhs` values, lexicographic keys with a `km`
  key-modifier, lazy deletion for stale priority-queue entries, and
  predecessor-driven incremental propagation — not A\* renamed.
- Bad-state avoidance as a **hard constraint** (infinite edge cost into a bad
  state, independent pre-flight and post-hoc path validation), never a soft
  penalty.
- A genuine cost/safety/reliability **tradeoff** the search itself optimizes
  (not just a reported score) — see `report.md` §11 for the composite edge
  weight and why this distinction matters.
- Six dynamic update operations (`updateGoal`, `addBadState`,
  `removeBadState`, `setTransitionAvailability`, `addTransition`,
  `removeTransition`) that reuse prior search state instead of rebuilding.
- Cartesian graph visualization (states plotted at their actual embedding
  coordinates, not a force-directed layout) and before/after dynamic
  replanning visualizations.
- Reproducible experiments (fixed seed, configurable graph sizes) exported to
  CSV, with performance plots generated purely from that CSV.

## D\* Lite, in one paragraph

D\* Lite maintains two value estimates per state, `g` (the best known
cost-to-goal) and `rhs` (a one-step lookahead estimate from `g`), and keeps a
priority queue of states where the two disagree ("locally inconsistent").
Search grows *backward* from the goal so that, when something changes near
the start, only a small, bounded region of the graph needs to be
re-examined — the queue processes states in order of a heuristically-guided
key, propagating updates along **predecessor** edges until the start becomes
locally consistent again. This is what makes it suitable for a dynamic
environment: an edge disappearing, a new edge appearing, or the goal moving
triggers a local `UpdateVertex` call and a short re-run of
`ComputeShortestPath`, not a full re-solve.

## Safety Mechanism

Every incoming edge to a bad state costs `+infinity` at search time,
independent of any objective weight — so no configuration can ever "trade
away" safety. Two more independent checks back this up: the initial/goal
states are validated as non-bad before search even starts, and the final
returned path is re-validated from scratch against the raw problem data
(never against the planner's own internal state) before being reported as
successful. `Bad states visited: 0` is asserted, not assumed, for every
successful run.

## Dynamic Replanning

See `report.md` §12 for the full table of what each of the six update
operations invalidates and how little work each one actually redoes. In the
5000-state experiment, disabling one edge on the found path costs **11 ms** of
replanning (16 states re-explored) versus **35 ms** for the initial solve —
concrete evidence the incremental machinery is doing its job rather than
silently rebuilding.

## Graph Visualization

`visualization/visualize_graph.py` renders the actual Cartesian graph
(states, directed edges, initial/goal/bad markers, the selected path
highlighted, unavailable edges dashed) straight from JSON the C++ program
exports — see `include/VisualisationData.h` for the schema.
`visualization/visualize_dynamic.py` renders a before/after comparison for
the three dynamic test cases. Example outputs live in `results/graphs/`.

## Experimental Evaluation

`./safepath --experiment` generates reproducible random Cartesian graphs
(seed 42 by default, configurable) at sizes 100/500/1000/5000, measures
success rate, cost, minimum safety distance, reliability, explored states,
planning time, memory estimate, and replanning time, and writes everything to
`results/experiments.csv`. A second sweep reproduces the Test-Case-3
safety-weight tradeoff numerically in `results/safety_weight_sweep.csv`.
`visualization/plot_experiments.py` turns both into the five required
performance plots — every value plotted comes from an actual program run,
nothing is hard-coded.

## Project Structure

```
SafePath/
├── CMakeLists.txt
├── README.md                  (this file)
├── USER_MANUAL.md
├── include/                   State.h, Transition.h, PlanningProblem.h,
│                               PlanningResult.h, Planner.h, DStarLite.h,
│                               Metrics.h, GraphGenerator.h,
│                               VisualisationData.h, TestScenarios.h
├── src/                       matching .cpp files + main.cpp (CLI)
├── tests/test_cases.cpp       31 unit tests
├── visualization/              visualize_graph.py, visualize_dynamic.py,
│                               plot_experiments.py, requirements.txt
├── data/generated/
├── results/                   experiments.csv, safety_weight_sweep.csv, graphs/
└── report/                    report.md, demo_output.txt
```

## Build Instructions

```bash
g++ -std=c++17 -O2 -Iinclude src/*.cpp -o safepath
```
or with CMake:
```bash
mkdir build && cd build && cmake .. && make -j4
```
Full details, including the unit-test build: `USER_MANUAL.md` §3.

## Usage

```bash
./safepath --test 1        # ... through --test 6
./safepath --demo          # all six, back to back
./safepath --experiment    # reproducible experiments -> CSV
./safepath --test 2 --export results/graphs/test2.json
```
Full CLI reference: `USER_MANUAL.md`.

## Visualization Instructions

```bash
pip install -r visualization/requirements.txt --break-system-packages
python3 visualization/visualize_graph.py results/graphs/test2.json
python3 visualization/visualize_dynamic.py results/graphs/test4_before.json results/graphs/test4_after.json
python3 visualization/plot_experiments.py results/experiments.csv --sweep results/safety_weight_sweep.csv
```

## Limitations

See `report.md` §23 — in short: the per-edge safety term is a documented
local approximation of the true (path-level) minimum-safety-distance
objective; bad-state changes currently re-examine every vertex rather than
using a spatial index; and the bonus features (multi-goal planning,
time-dependent availability, parallel search) were intentionally left out per
the assignment's "do not overengineer" guidance, in favor of correctness and
demonstrability of the core, required assignment.
