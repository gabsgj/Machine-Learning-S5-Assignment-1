# User Manual

## Requirements

- A C++17 compiler (tested with GCC 13).
- CMake 3.10+ (optional — a direct `g++` build command is given below if
  CMake is not installed).

## Building

### With CMake (recommended)

```bash
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build .
```

This produces three executables in `build/`:

| Executable        | Purpose                                             |
|--------------------|------------------------------------------------------|
| `planner_demo`     | Worked example: initial plan + two live replans      |
| `planner_tests`    | Automated suite, all six assignment test cases       |
| `run_experiments`  | Randomized evaluation sweep, writes `results.csv`     |

### Without CMake

```bash
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude src/LPAStar.cpp src/main.cpp -o planner_demo
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude src/LPAStar.cpp tests/test_planner.cpp -o planner_tests
g++ -std=c++17 -O2 -Wall -Wextra -Iinclude src/LPAStar.cpp experiments/run_experiments.cpp -o run_experiments
```

## Running

```bash
./planner_demo
```

Builds the "bad state avoidance" scenario from the assignment (Test Case
2), solves it, then demonstrates two incremental replans: a shortcut
transition being added, and that same shortcut later becoming unavailable.
For each stage it prints whether a solution was found, the state path, the
total cost, the minimum clearance to any bad state, the cumulative
reliability, the number of states explored, and the planning/replanning
time.

```bash
./planner_tests
# or, from the build directory: ctest
```

Runs the full automated suite (20 checks across the six assignment test
cases: basic reachability, bad-state avoidance, safety-margin trade-off,
dynamic transition removal, goal update, and transition addition) and
prints a pass/fail summary. Exits non-zero if any check fails.

```bash
./run_experiments [output.csv]
```

Generates random grid-shaped planning problems of increasing size (5×5 up
to 30×30, 5 trials per size), solves each from scratch, applies a batch of
dynamic edits (~10% of transitions disabled, five new shortcuts added), and
records the incremental replan. Writes one row per trial to
`experiments/results.csv` (or the path you supply) with: grid size, state
and transition counts, success flag, bad states visited (structurally
always zero), total path cost, minimum clearance, states explored,
planning time, replanning time, and an approximate memory footprint.

## Using the library in your own code

```cpp
#include "PlanningProblem.h"
#include "LPAStar.h"

using namespace planner;

PlanningProblem problem;
problem.states = { State(0, {0.0, 0.0}), State(1, {1.0, 0.0}) /* ... */ };
problem.transitions = { Transition(/*id=*/1, /*from=*/0, /*to=*/1, /*cost=*/1.0) };
problem.initialState = 0;
problem.goalState = 1;
problem.badStates = {};          // ids of states to avoid
problem.safetyRadius = 0.0;      // hard minimum clearance, 0 disables it
problem.beta = 1.0;              // cost weight
problem.gamma = 1.0;             // safety weight (per-edge score + clearance)
problem.delta = 0.25;            // reliability weight

LPAStarPlanner planner;
PlanningResult result = planner.plan(problem);
if (result.success) {
    // result.statePath, result.transitionPath, result.totalCost,
    // result.safetyScore, result.reliabilityScore
}

// Later, when the environment changes:
planner.updateTransition(/*id=*/1, /*newCost=*/2.5, /*available=*/false);
PlanningResult replanned = planner.replan();          // fast, incremental

// A new transition appearing:
planner.addTransition(Transition(/*id=*/2, 0, 1, 0.5));
PlanningResult afterAdd = planner.replan();

// A goal (or bad-state-set) change needs a full reinitialization:
problem.goalState = 5;
PlanningResult afterGoalChange = planner.replanWithNewGoal(problem);
```

## Tuning the objective

`Score(P) = alpha*G - beta*C + gamma*D + delta*R` from the assignment maps
onto the fields on `PlanningProblem`:

- `beta` — weight on cumulative transition cost (`C`).
- `gamma` — weight on the safety terms: per-edge safety score and
  geometric clearance from bad states (together approximate `D`).
- `delta` — weight on per-edge reliability (`R`).
- `safetyRadius` — a **hard** minimum clearance; any state closer than this
  to a bad state is excluded from the graph entirely, rather than merely
  penalized. Leave at `0.0` to rely purely on the soft `gamma` penalty.

Raising `gamma` relative to `beta` shifts the planner from "cheapest path"
toward "path that stays as far from danger as possible", as shown in the
Safety Margin test in `tests/test_planner.cpp`.

## Demonstration recording

For the "Demonstration" deliverable, a short screen recording (or terminal
transcript) of the following sequence is sufficient:

```bash
mkdir build && cd build && cmake .. && cmake --build .
./planner_tests
./planner_demo
./run_experiments
```
