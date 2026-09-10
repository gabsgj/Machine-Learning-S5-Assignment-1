# Safe Semantic Planner (D* Lite)

PCCST503 — Machine Learning, Assignment 1: *Design of a Safe Semantic
Planner in a Finite Cartesian State Space.*

A generic, replanning-capable path planner over a finite set of states
embedded in `R^d`, built on **D* Lite** and extended with a safety- and
reliability-aware edge cost so it satisfies all five optimization
objectives in the brief: reach the goal, never touch a bad state,
minimize cost, maximize distance from bad states, and run fast enough to
replan online as the environment changes.

## Contents

| Deliverable (per brief) | Location |
|---|---|
| 1. C++ source code | [`include/`](include/), [`src/`](src/) |
| 2. Design report | [`DesignReport.md`](DesignReport.md) |
| 3. Experimental results | [`ExperimentalResults.md`](ExperimentalResults.md) |
| 4. User manual | [`UserManual.md`](UserManual.md) |
| 5. Demonstration | `make run` → runs all 6 illustrative test cases from the brief |

## Quick Start

```bash
make        # build
make run    # run all six test cases, print results, write results.csv
```

No dependencies beyond a C++17 compiler and `make`.

## Design Summary

* **State/Transition/PlanningProblem/PlanningResult/Planner** classes
  match the interfaces given in the brief exactly (`PlanningResult` is
  additively extended with telemetry fields for the experimental
  report — `exploredStates`, `planningTimeMs`, `cumulativeReliability`
  — the required fields are untouched).
* **Algorithm**: D* Lite, chosen for its incremental-replanning
  guarantees (Sec. "Dynamic Environment" of the design report) — the
  brief's Test Cases 4–6 are exactly the scenario D* Lite is built for.
* **Safety**: any transition into a bad state, or any unavailable
  transition, is assigned infinite search cost — a hard guarantee, not a
  soft preference. Cost, safety, reliability, and proximity-to-bad-state
  are combined into one non-negative edge weight (full derivation in the
  design report, Sec. 4) so the underlying search remains a standard
  shortest-path problem.
* **Dynamic updates**: `setEdgeAvailability`, `addTransition`,
  `removeTransition`, `addBadState`, `removeBadState`, `updateStart` are
  all handled incrementally (only the locally-affected fringe of the
  search is reopened). `updateGoal` is a bounded reinitialization
  (explained, with the reasoning, in the design report) since classical
  D* Lite anchors its search at a fixed goal.

Full algorithmic detail, complexity analysis, and the reasoning behind
every design choice are in **[DesignReport.md](DesignReport.md)**.

## Test Cases

All six illustrative test cases from the brief are implemented in
[`src/main.cpp`](src/main.cpp) and pass; see
[`ExperimentalResults.md`](ExperimentalResults.md) for the full
results table (goal success rate, bad states visited, cost, safety
margin, explored states, and planning time per case).

## Bonus / Possible Extensions

Not implemented in this submission, but the architecture leaves room
for them (noted here as candidates, per the brief's bonus list):

* **Incremental goal changes** without reinitialization, via a
  dual-graph / bidirectional D* Lite formulation.
* **Multi-goal planning**: solve as a small TSP over per-goal D* Lite
  distances, reusing the same incremental machinery per goal.
* **Time-dependent availability**: extend `Transition` with a validity
  window and re-evaluate `edgeCost` against a simulation clock.
* **Learning-based heuristic**: replace `minCostRate * distance` with a
  learned estimator, keeping the same `heuristic()` call site.

## License

Coursework submission for PCCST503. No license granted for reuse outside
the course context unless the instructor specifies otherwise.
