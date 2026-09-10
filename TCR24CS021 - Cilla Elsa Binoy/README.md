# Safe Semantic Planner — User Manual

## What this is

A C++17 implementation of the planner described in the assignment: given a
finite set of states embedded in R^d, a start, a goal, a set of bad states,
and a set of directed, weighted, possibly-unavailable transitions, compute a
cost-minimizing, bad-state-avoiding path — and **replan efficiently** when
the goal, the bad states, or the transitions change, instead of resolving
from scratch. The algorithm is D\* Lite.

## Build & Run

```bash
make          # compiles ./planner
make run      # compiles (if needed) and runs the full test suite
make clean    # removes the binary
```

Or directly:

```bash
g++ -std=c++17 -O2 -Wall -Iinclude src/DStarLitePlanner.cpp src/main.cpp -o planner
./planner
```

Running it prints, for each of the 6 required test cases plus one bonus
scenario: the resulting path, its total cost, its minimum distance to any
bad state, how many states the search expanded, the peak open-set size, and
the wall-clock planning time — everything needed for the "students should
evaluate" section of the assignment.

## Project Layout

```
include/
  State.hpp             -- a point in R^d
  Transition.hpp        -- a directed weighted edge with cost/safety/reliability/available
  PlanningProblem.hpp   -- the full problem instance
  PlanningResult.hpp    -- the returned plan + diagnostics
  Planner.hpp           -- abstract interface (plan())
  DStarLitePlanner.hpp  -- the D* Lite planner's public API
src/
  DStarLitePlanner.cpp  -- the algorithm itself
  main.cpp              -- builds the 6 test cases + 1 bonus case and prints results
Makefile
REPORT.md               -- the design report (state rep, complexity, heuristic, etc.)
README.md               -- this file
```

## Using it in your own code

```cpp
#include "DStarLitePlanner.hpp"

PlanningProblem problem;
problem.states = { State{1, {0,0}}, State{2, {1,0}} };
problem.initialState = 1;
problem.goalState = 2;
problem.transitions = { Transition{100, 1, 2, /*cost*/1.0, /*safety*/1.0, /*reliability*/1.0, /*available*/true} };

DStarLitePlanner planner;             // default weights: cost=1.0, safety=1.0, margin=0.5, heuristic=0
PlanningResult result = planner.plan(problem);

if (result.success) {
    // result.statePath, result.transitionPath, result.totalCost, result.safetyScore
}

// Later, the environment changes -- these are all O(affected states), not O(whole graph):
planner.setTransitionAvailability(100, false);   // an edge goes down
planner.addTransition(Transition{101, 1, 2, 2.0, 1.0, 1.0, true}); // a new edge appears
planner.addBadState(2);                           // a state is discovered unsafe
planner.updateGoal(3);                            // the goal moves (full resolve, see REPORT.md §3)
```

Each of these calls returns a fresh `PlanningResult` already re-extracted
from the updated search state — you don't need to call `plan()` again.

## Tuning the cost/safety/reliability tradeoff

The constructor takes four weights:

```cpp
DStarLitePlanner(double costWeight = 1.0,
                  double safetyWeight = 1.0,
                  double marginWeight = 0.5,
                  double heuristicWeight = 0.0);
```

- `costWeight`: how much the raw `Transition.cost` matters.
- `safetyWeight`: how much an edge's own `1 - safety` matters.
- `marginWeight`: how strongly to prefer routes that stay far from every bad
  state (see REPORT.md §4 for the exact formula and Test Case 3 for a live
  demonstration of raising this).
- `heuristicWeight`: 0 = exact incremental Dijkstra (always safe); >0 makes
  the search closer to A* using Euclidean distance to the start as a guide
  — only turn this up if your `cost` values are known to correlate with
  physical distance (see REPORT.md §5).

Bad states themselves are never merely "penalized" — they are always a hard
constraint, regardless of these weights.

## Extending to a knowledge graph (bonus)

Map each entity to a `State` (its `embedding` can be any vector
representation you have, e.g. a KG embedding) and each relation instance to
a `Transition` (`cost` = 1 or a domain weight, `safety`/`reliability` =
domain-specific confidence scores if you have them, `available` = true).
The planner then finds cost-minimizing, "bad-entity"-avoiding paths through
the graph with no other code changes.
