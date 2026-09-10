# D* Lite Planner — User Manual

## Table of Contents

1. [Quick Start](#quick-start)
2. [Installation & Build](#installation--build)
3. [Basic Usage](#basic-usage)
4. [API Reference](#api-reference)
5. [Configuration & Tuning](#configuration--tuning)
6. [Advanced Workflows](#advanced-workflows)
7. [Troubleshooting](#troubleshooting)
8. [Examples](#examples)

---

## Quick Start

**This planner solves the dynamic path-planning problem:** given a set of states in Euclidean space, a start, a goal, bad states to avoid, and weighted transitions, find the cost-minimal path that avoids bad states — and replan efficiently when the environment changes.

```cpp
#include "DStarLitePlanner.hpp"

// 1. Create a planning problem
PlanningProblem problem;
problem.states = { State{1, {0, 0}}, State{2, {1, 0}}, State{3, {2, 0}} };
problem.initialState = 1;
problem.goalState = 3;
problem.transitions = {
    Transition{100, 1, 2, 1.0, 1.0, 1.0, true},
    Transition{101, 2, 3, 1.0, 1.0, 1.0, true}
};

// 2. Create the planner
DStarLitePlanner planner;

// 3. Solve
PlanningResult result = planner.plan(problem);

// 4. Check results
if (result.success) {
    std::cout << "Found path of cost: " << result.totalCost << std::endl;
    std::cout << "Safety score: " << result.safetyScore << std::endl;
}
```

---

## Installation & Build

### Prerequisites

- **C++17** or later
- **g++** or compatible compiler
- **make**

### Building

From the project root:

```bash
# Build the planner
make

# Build and run all tests
make run

# Clean build artifacts
make clean
```

### Manual Compilation

```bash
g++ -std=c++17 -O2 -Wall -Iinclude src/DStarLitePlanner.cpp src/main.cpp -o planner
./planner
```

### Using in Your Project

1. Copy the `include/` directory into your project
2. Include `#include "DStarLitePlanner.hpp"` in your code
3. Compile with `-std=c++17 -Ipath/to/include`

---

## Basic Usage

### Step 1: Define States

States are points in R^d (any dimension):

```cpp
// 2D state space
State s1{1, {0.0, 0.0}};    // id=1, position=(0,0)
State s2{2, {1.5, 2.0}};    // id=2, position=(1.5, 2.0)
State s3{3, {3.0, 1.0}};    // id=3, position=(3.0, 1.0)

// 3D state space
State s4{4, {0.0, 0.0, 0.0}};
State s5{5, {1.0, 2.0, 3.0}};
```

**Important:** State IDs must be unique positive integers.

### Step 2: Define Transitions

Transitions are directed, weighted edges:

```cpp
Transition edge{
    id,              // unique ID
    from,            // from state id
    to,              // to state id
    cost,            // edge cost (double)
    safety,          // safety measure in [0,1] (1=safe, 0=unsafe)
    reliability,     // reliability in (0,1] (1=always available)
    available        // currently available? (bool)
};

// Example: safe, direct path
Transition t1{100, 1, 2, 1.0, 0.95, 1.0, true};

// Example: risky, cheaper path
Transition t2{101, 1, 3, 0.5, 0.5, 1.0, true};

// Example: unreliable edge (might fail)
Transition t3{102, 2, 3, 2.0, 0.9, 0.8, true};
```

### Step 3: Create the Problem

```cpp
PlanningProblem problem;
problem.states = {s1, s2, s3};
problem.initialState = 1;
problem.goalState = 3;
problem.transitions = {t1, t2, t3};
problem.badStates = {};  // initially empty; add ids as needed
```

### Step 4: Plan and Get Results

```cpp
DStarLitePlanner planner;
PlanningResult result = planner.plan(problem);

if (result.success) {
    // Path found!
    std::cout << "Path length: " << result.statePath.size() << std::endl;
    std::cout << "Total cost: " << result.totalCost << std::endl;
    std::cout << "Min distance to bad state: " << result.minDistToBadState << std::endl;
    std::cout << "States expanded: " << result.statesExpanded << std::endl;
} else {
    std::cout << "No path found." << std::endl;
}
```

---

## API Reference

### `PlanningProblem`

Represents the planning instance. Members:

| Member | Type | Description |
|--------|------|-------------|
| `states` | `vector<State>` | All states in the domain |
| `initialState` | `int` | Start state id |
| `goalState` | `int` | Goal state id |
| `transitions` | `vector<Transition>` | All edges |
| `badStates` | `set<int>` | States to strictly avoid (hard constraint) |

### `State`

Represents a state in the search space.

```cpp
struct State {
    int id;                    // unique identifier
    vector<double> position;   // coordinates in R^d
};
```

### `Transition`

Represents a directed edge with cost and quality metrics.

```cpp
struct Transition {
    int id;
    int from;
    int to;
    double cost;              // edge cost
    double safety;            // safety in [0,1]
    double reliability;       // reliability in (0,1]
    bool available;           // is edge currently usable?
};
```

### `DStarLitePlanner`

Main planner class.

#### Constructor

```cpp
DStarLitePlanner(double costWeight = 1.0,
                  double safetyWeight = 1.0,
                  double marginWeight = 0.5,
                  double heuristicWeight = 0.0);
```

**Parameters:**

- `costWeight` (default: 1.0): Weight for edge cost in the objective
- `safetyWeight` (default: 1.0): Weight for edge safety penalty (1 - safety)
- `marginWeight` (default: 0.5): Weight for staying far from bad states
- `heuristicWeight` (default: 0.0): Weight for Euclidean heuristic (0 = exact Dijkstra)

#### Methods

```cpp
// Initial planning
PlanningResult plan(const PlanningProblem& problem);

// Dynamic environment updates (all return PlanningResult)
PlanningResult setTransitionAvailability(int transitionId, bool available);
PlanningResult addTransition(const Transition& t);
PlanningResult addBadState(int stateId);
PlanningResult removeBadState(int stateId);
PlanningResult updateGoal(int newGoalId);
```

**Note:** `updateGoal()` requires a full replanning (O(n log n)) because D* Lite's cost estimates are defined relative to a fixed goal.

All other updates are incremental (O(affected states)).

### `PlanningResult`

Returned by all planning methods. Members:

| Member | Type | Description |
|--------|------|-------------|
| `success` | `bool` | Did planning succeed? |
| `statePath` | `vector<int>` | Sequence of state ids from start to goal |
| `transitionPath` | `vector<int>` | Sequence of transition ids along the path |
| `totalCost` | `double` | Sum of transition costs |
| `safetyScore` | `double` | Path safety score |
| `minDistToBadState` | `double` | Closest distance to any bad state along the path |
| `statesExpanded` | `int` | Search efficiency metric |
| `peakOpenSetSize` | `int` | Peak open list size |
| `planningTimeMs` | `double` | Wall-clock planning time in milliseconds |

---

## Configuration & Tuning

### Balancing Cost, Safety, and Reliability

The planner combines four objectives into a single scalar cost used during search:

```
effective_cost = (costWeight * transition.cost + safetyWeight * (1 - safety)) / reliability
                 + marginWeight / (distance_to_bad_state + 0.15)
```

**Tuning strategies:**

1. **High-cost environment, low safety margins:**
   ```cpp
   DStarLitePlanner planner(1.0, 1.0, 0.1, 0.0);  // prioritize cost
   ```

2. **Safety-critical (e.g., autonomous driving):**
   ```cpp
   DStarLitePlanner planner(0.5, 2.0, 2.0, 0.0);  // heavy safety/margin weights
   ```

3. **Large open-space with some obstacles:**
   ```cpp
   DStarLitePlanner planner(1.0, 1.0, 0.5, 0.2);  // Euclidean heuristic helps
   ```

4. **Small, complex graphs (costs unrelated to distance):**
   ```cpp
   DStarLitePlanner planner(1.0, 1.0, 0.5, 0.0);  // disable heuristic
   ```

### Performance Tuning

- **Heuristic weight:** Setting `heuristicWeight > 0` is only correct if your edge costs correlate with Euclidean distance. If costs are arbitrary, use 0 (exact Dijkstra).
- **Problem size:** The planner uses a balanced BST for the open set, so scaling is O(n log n) in the number of states.
- **Incremental updates:** Small changes to edges or bad states trigger only local replanning; watch `statesExpanded` to verify.

---

## Advanced Workflows

### Scenario 1: Reactive Replanning

Initially solve; then react to environment changes:

```cpp
DStarLitePlanner planner(1.0, 1.0, 0.5, 0.0);
PlanningResult result = planner.plan(problem);

// Path in place... then an edge fails
PlanningResult result2 = planner.setTransitionAvailability(100, false);
if (result2.success) {
    std::cout << "Replanned; new path cost: " << result2.totalCost << std::endl;
}

// Another obstacle discovered
PlanningResult result3 = planner.addBadState(2);
std::cout << "Expanded " << result3.statesExpanded << " states." << std::endl;
```

### Scenario 2: Goal Changes

**Important:** Goal changes require full replanning. Use sparingly in performance-critical loops:

```cpp
// Initial goal
PlanningResult result = planner.plan(problem);

// Goal moved to state 5
PlanningResult result2 = planner.updateGoal(5);
// This internally re-initializes all cost estimates and re-solves.
```

### Scenario 3: Iterative Refinement

Solve with loose weights, then tighten:

```cpp
// Quick, cost-focused plan
DStarLitePlanner planner1(2.0, 0.5, 0.1, 0.0);
PlanningResult quick = planner1.plan(problem);

// If time allows, solve more carefully
DStarLitePlanner planner2(1.0, 1.5, 1.0, 0.0);
PlanningResult safe = planner2.plan(problem);

std::cout << "Quick cost: " << quick.totalCost 
          << ", Safety: " << quick.safetyScore << std::endl;
std::cout << "Safe cost: " << safe.totalCost 
          << ", Safety: " << safe.safetyScore << std::endl;
```

---

## Troubleshooting

### No Path Found (`result.success == false`)

**Causes:**
1. **Start or goal unreachable** — check that transitions connect them
2. **All paths blocked by bad states** — remove some bad states or add new transitions
3. **Goal is itself a bad state** — remove it from `badStates`

**Debugging:**
```cpp
// Check connectivity
std::cout << "Start: " << problem.initialState 
          << ", Goal: " << problem.goalState << std::endl;
std::cout << "Bad states: ";
for (int id : problem.badStates) std::cout << id << " ";
std::cout << std::endl;
```

### Path Goes Through Bad State

This should not happen (bad states are hard constraints). If it does:
- Verify `badStates` is non-empty and contains the state you think
- Verify no transition marked `available=true` has a bad state as `from` or `to`

### Slow Planning

1. **Check heuristic weight** — if not correlated with distance, set to 0
2. **Reduce state space** — fewer states = faster search
3. **Check reliability weights** — very low reliability inflates costs; use a floor of ~0.1

### Unexpected Path After Update

All incremental updates are correct by construction. If a path changes unexpectedly after, e.g., removing a bad state, it may simply be that the new path is cheaper/safer than before — this is the correct behavior.

---

## Examples

### Example 1: Simple 3-State Problem

```cpp
#include "DStarLitePlanner.hpp"
#include <iostream>

int main() {
    // Create problem: 1 -> 2 -> 3
    PlanningProblem problem;
    problem.states = {
        State{1, {0.0, 0.0}},
        State{2, {1.0, 0.0}},
        State{3, {2.0, 0.0}}
    };
    problem.initialState = 1;
    problem.goalState = 3;
    problem.transitions = {
        Transition{10, 1, 2, 1.0, 1.0, 1.0, true},
        Transition{11, 2, 3, 1.0, 1.0, 1.0, true}
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(problem);

    if (result.success) {
        std::cout << "Success! Path: ";
        for (int s : result.statePath) std::cout << s << " ";
        std::cout << "\nCost: " << result.totalCost << std::endl;
    }
    return 0;
}
```

### Example 2: Safety-Aware Planning

```cpp
// High-dimension goal: find the safest path even if it costs more
DStarLitePlanner safePlanner(0.5, 2.0, 2.0, 0.0);  // cost=0.5, safety=2.0, margin=2.0

PlanningResult safeResult = safePlanner.plan(problem);
std::cout << "Safe path cost: " << safeResult.totalCost << std::endl;
std::cout << "Min dist to bad state: " << safeResult.minDistToBadState << std::endl;
```

### Example 3: Dynamic Obstacle Avoidance

```cpp
DStarLitePlanner planner;
PlanningResult result = planner.plan(problem);

// Obstacle discovered at state 5
result = planner.addBadState(5);
std::cout << "After obstacle: cost=" << result.totalCost 
          << ", expanded=" << result.statesExpanded << " states" << std::endl;

// Obstacle cleared
result = planner.removeBadState(5);
std::cout << "After clearing: cost=" << result.totalCost 
          << ", expanded=" << result.statesExpanded << " states" << std::endl;
```

---

## For More Information

- **REPORT.md** — Technical design, algorithm explanation, complexity analysis, heuristic design
- **README.md** — Project overview, build instructions, quick API summary
- **Header files** — Detailed inline documentation in `include/*.hpp`

---

**Questions?** Check the REPORT.md for design rationale and the inline comments in `src/DStarLitePlanner.cpp` for implementation details.
