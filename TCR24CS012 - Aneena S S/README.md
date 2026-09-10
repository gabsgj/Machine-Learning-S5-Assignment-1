[README.md](https://github.com/user-attachments/files/31580965/README.md)
# Safe Semantic Planner — D* Lite in a Finite Cartesian State Space

**PCCST503 — Machine Learning, Assignment 1**

A C++17 implementation of an incremental, safety-aware path planner. Given a
directed graph of states embedded in ℝᵈ, it finds a route from a start state
to a goal state that never enters a forbidden ("bad") state, balances cost
against distance-to-danger and transition reliability, and — the core of the
assignment — **replans efficiently when the environment changes**, without
rebuilding the search from scratch.

## Why D* Lite

The assignment allows LPA* or D* Lite. D* Lite was chosen because the
assignment's required test cases are all about *dynamic* environments:
transitions disappearing, transitions appearing, and the goal moving. D* Lite
searches backward from the goal and maintains two values per state, `g` and
`rhs`; after a local change, only the states whose values actually became
stale are reprocessed, instead of re-running the search over the whole graph.

## Features

- **State / Transition / PlanningProblem / PlanningResult / Planner**
  interfaces matching the assignment's suggested C++ signatures.
- Safety-aware edge weighting: cost, a bounded penalty for passing close to
  a bad state, and a bounded reliability discount — bounded specifically so
  the search heuristic stays admissible and the algorithm remains correct.
- Bad states are always excluded from the graph — a route can never enter one.
- Incremental updates without a full graph rebuild:
  - `updateTransition()` — change cost / safety / reliability / availability
    of an existing edge
  - `addTransition()` — insert a new edge (e.g. a shortcut)
  - `updateGoal()` — change the goal state
  - `moveStart()` — change the current start state
- Reports goal success, full state/transition path, total cost, minimum
  safety distance, cumulative reliability, states explored, planning /
  replanning time, and estimated memory usage.

## Repository Contents

| File | Description |
|---|---|
| `safe_planner.cpp` | Complete, self-contained C++17 implementation, including six demonstration test cases in `main()`. No external dependencies. |
| `Design_Report.docx` | Algorithm design, data structures, heuristic, safety computation, pseudocode, and time/space complexity analysis. |
| `Experimental_Results.docx` | Verified output of all six test cases: paths, costs, safety margins, reliability, explored states, timing, and memory. |
| `User_Manual.docx` | How to compile, run, and interpret the program's output. |
| `Demonstration.docx` | A presentation/viva script for walking through the six test cases live. |

## Build & Run

Requires a C++17 compiler. No external libraries.

```bash
g++ -std=c++17 -O2 -Wall -Wextra safe_planner.cpp -o safe_planner
./safe_planner
```

On Windows:

```bash
g++ -std=c++17 -O2 safe_planner.cpp -o safe_planner.exe
.\safe_planner.exe
```

## Test Cases

All six scenarios required by the assignment are implemented and run
automatically from `main()`:

| # | Scenario | What it demonstrates |
|---|---|---|
| 1 | Basic Reachability | Finds the unique valid path. |
| 2 | Bad State Avoidance | Never routes through a forbidden state. |
| 3 | Safety Margin | Prefers a costlier route that stays farther from danger. |
| 4 | Dynamic Transition | Replans after an edge becomes unavailable mid-plan. |
| 5 | Goal Update | Replans after the goal changes, without rebuilding the graph. |
| 6 | Transition Addition | Immediately adopts a newly inserted shortcut edge. |

Sample output (values are deterministic; timing/memory vary slightly by machine):

```
Test Case 4A: Before transition removal
  Success: true
  State path: 1 -> 2 -> 4
  Total path cost: 2.000

Test Case 4B: After transition removal
  Success: true
  State path: 1 -> 3 -> 4
  Total path cost: 2.800
  Explored states: 4
  Planning/replanning time: 0.02 ms
```

## Output Fields

- **Success** — whether a valid path to the goal was found
- **State path / Transition ID path** — the route and the edges used
- **Total path cost** — sum of raw transition costs along the path
- **Minimum safety distance** — closest approach to any bad state along the
  path; shown as `N/A` when the scenario defines no bad states
- **Cumulative reliability** — sum of transition reliability values used
- **Explored states** — states processed during this search or incremental update
- **Planning/replanning time** — wall-clock time (`std::chrono::steady_clock`)
- **Memory usage** — estimated bytes used by the planner's internal `g`/`rhs`
  tables, transitions, and states

## Complexity

Let *V* be the number of states and *E* the number of transitions, *k* the
number of bad states, and *d* the embedding dimension.

- **Time:** priority-queue operations are O(log V); a full repair after a
  local change is bounded by O(E log V) in the worst case, though D* Lite's
  incremental repair only touches states whose values actually became stale,
  so practical replanning cost is typically much smaller when changes are
  localized. Each safety-distance computation is O(k·d).
- **Space:** O(V + E) for the graph, O(V) for the search state (`g`, `rhs`,
  version, open list), O(k) for the bad-state set, and O(V·d) for state
  coordinates.

## Extending to a New Problem

```cpp
PlanningProblem p;
p.initialState = ...;
p.goalState    = ...;
p.states        = { /* State{id, embedding} ... */ };
p.badStates     = { /* forbidden state ids */ };
p.transitions   = { /* Transition{id, from, to, cost, safety, reliability, available} ... */ };

DStarLite planner;
PlanningResult result = planner.plan(p);
```

For dynamic operation, keep the `DStarLite` object alive and call
`updateTransition()`, `addTransition()`, `moveStart()`, or `updateGoal()` as
the environment changes, then read `currentResult()` for the repaired plan.

## Course

PCCST503 — Machine Learning, Department of Computer Science and Engineering.
