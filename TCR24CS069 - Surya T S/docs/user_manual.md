# Safe Semantic Planner — User Manual

## PCCST503 Machine Learning — Assignment
**Author:** Surya T S  
**University Registration Number:** TCR24CS069  

---

## Table of Contents

1. [System Requirements](#1-system-requirements)
2. [Project Structure](#2-project-structure)
3. [Building the Project](#3-building-the-project)
4. [Running the Demonstration](#4-running-the-demonstration)
5. [Running the Test Suite](#5-running-the-test-suite)
6. [Visual Demonstration (HTML)](#6-visual-demonstration-html)
7. [Understanding the Output](#7-understanding-the-output)
8. [API Reference](#8-api-reference)
9. [Configuring Weights](#9-configuring-weights)
10. [Creating Custom Test Cases](#10-creating-custom-test-cases)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. System Requirements

| Requirement | Details |
|-------------|---------|
| **Compiler** | g++ (GCC) 6.3.0 or later with C++14 support |
| **OS** | Windows 10/11 (tested), Linux, macOS |
| **Dependencies** | Standard C++ library only — no external dependencies |
| **Browser** | Any modern browser (for HTML visual demo) |
| **Disk space** | < 5 MB |

---

## 2. Project Structure

```
Safe_Semantic_Planner/
|
+-- include/                    Header files
|   +-- state.h                 State class (id + embedding vector)
|   +-- transition.h            Transition class (cost, safety, reliability)
|   +-- planning_problem.h      Complete problem specification
|   +-- planning_result.h       Planning result (path + metrics)
|   +-- planner.h               Abstract Planner interface
|   +-- d_star_lite_planner.h   D* Lite implementation header
|   +-- safety_utils.h          Safety distance utilities
|
+-- src/                        Source files
|   +-- d_star_lite_planner.cpp D* Lite implementation
|   +-- safety_utils.cpp        Safety computation
|   +-- main.cpp                Console demonstration (6 test cases)
|
+-- tests/
|   +-- test_planner.cpp        Automated test suite (14 tests)
|
+-- demo/
|   +-- index.html              Interactive visual demonstration
|
+-- docs/
|   +-- design_report.md        Design report
|   +-- user_manual.md          This manual
|
+-- build/                      Compiled binaries
|   +-- safe_planner.exe        Main demonstration
|   +-- test_planner.exe        Automated tests
|
+-- CMakeLists.txt              CMake build system
+-- build.bat                   Windows build script
```

---

## 3. Building the Project

### Method 1: Windows Build Script (Recommended)

Open a command prompt or PowerShell in the project directory and run:

```
build.bat
```

This compiles both executables into the `build/` directory.

### Method 2: Direct g++ Commands

```bash
# Create build directory
mkdir build

# Compile the main demonstration
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude ^
    src/main.cpp src/d_star_lite_planner.cpp src/safety_utils.cpp ^
    -o build/safe_planner.exe

# Compile the automated test suite
g++ -std=c++14 -Wall -Wextra -O2 -Iinclude ^
    tests/test_planner.cpp src/d_star_lite_planner.cpp src/safety_utils.cpp ^
    -o build/test_planner.exe
```

### Method 3: CMake (if installed)

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

### Verifying the Build

After building, you should see:
```
build/safe_planner.exe    (~200 KB)
build/test_planner.exe    (~200 KB)
```

---

## 4. Running the Demonstration

### Command

```
.\build\safe_planner.exe
```

### What It Does

The demonstration runs all **6 required test cases** sequentially:

| Test | Scenario | What Is Demonstrated |
|------|----------|---------------------|
| **TC1** | S -> A -> B -> G | Basic pathfinding through a linear graph |
| **TC2** | Two paths, one blocked by bad state X | Bad state avoidance |
| **TC3** | Cheap-but-unsafe vs expensive-but-safe | Tunable cost/safety tradeoff |
| **TC4** | Edge A->G disabled after initial plan | Dynamic replanning after edge removal |
| **TC5** | Goal changes from G1 to G2 | Replanning after goal update |
| **TC6** | Shortcut A->G added after initial plan | Discovery of new, better path |

### Sample Output

Each test case displays:
1. **Graph Layout** — ASCII diagram of the graph structure
2. **Phase Labels** — For dynamic tests, Phase 1 (before) and Phase 2 (after)
3. **Results Table** with:

```
+===========================================================+
| TC1: Basic Reachability                                    |
+===========================================================+
| Success:              YES                                  |
| State Path:           0 -> 1 -> 2 -> 3                     |
| Transition Path:      T0, T1, T2                           |
| Total Cost:           3.00                                 |
| Safety Score (D):     INF (no bad states)                  |
| States Explored:      3                                    |
| Planning Time:        5.2 us                               |
+-----------------------------------------------------------+
```

---

## 5. Running the Test Suite

### Command

```
.\build\test_planner.exe
```

### What It Tests

The suite runs **14 automated assertions**:

**Safety utility tests (4):**
- Euclidean distance in 2D
- Euclidean distance in 4D
- Safety map with no bad states
- Safety map with one bad state

**Assignment test cases (6):**
- TC1 through TC6 with full assertion checks

**Edge cases (4):**
- No path exists between start and goal
- Start equals goal (trivial path)
- All transitions unavailable
- Bad state blocks the only path

### Expected Output

```
==================================================
  Safe Semantic Planner - Automated Test Suite
==================================================

  [TEST] Euclidean distance 2D... PASS
  [TEST] Euclidean distance 4D... PASS
  [TEST] Safety map with no bad states... PASS
  [TEST] Safety map with one bad state... PASS

  [TEST] TC1: Basic Reachability... PASS
  [TEST] TC2: Bad State Avoidance... PASS
  [TEST] TC3: Safety Margin tradeoff... PASS
  [TEST] TC4: Dynamic Transition Unavailability... PASS
  [TEST] TC5: Goal Update... PASS
  [TEST] TC6: Shortcut Transition Addition... PASS

  [TEST] Edge: No path exists... PASS
  [TEST] Edge: Start equals goal... PASS
  [TEST] Edge: All transitions unavailable... PASS
  [TEST] Edge: Bad state blocks only path... PASS

==================================================
  Results: 14 / 14 tests passed   ALL PASS
==================================================
```

Exit code: `0` = all pass, `1` = some failures.

---

## 6. Visual Demonstration (HTML)

### Opening

Open `demo/index.html` in any modern web browser:

```
start demo/index.html
```

Or double-click the file in Windows Explorer.

### Features

The visual demonstration provides:

1. **Interactive test case selection** — Click buttons at the top to switch between TC1-TC6

2. **Animated graph visualization** — Shows:
   - Green circle = Start state (S)
   - Amber circle = Goal state (G)
   - Red circle = Bad state (with red dashed danger zone)
   - Blue circles = Normal states
   - Green animated line = Solution path
   - Grey lines = Unused transitions
   - Red dashed line = Disabled transition
   - Gold dashed line = Shortcut transition

3. **Phase switching** — For dynamic test cases (TC3, TC4, TC5, TC6), buttons allow switching between "Before" and "After" phases to see how the path changes

4. **Metrics panel** — Displays success status, total cost, safety distance, and states explored

5. **Explanation panel** — Describes what the planner is doing in plain English for each test case and phase

### What to Demonstrate

When presenting the project:
1. Start with **TC1** to show basic pathfinding
2. Move to **TC2** to show bad state avoidance (note the red danger zone around X)
3. Show **TC3** to demonstrate the cost/safety tradeoff — switch between "Default Weights" and "High Safety" phases
4. Show **TC4** — switch from "All Available" to "A->G Disabled" to show replanning
5. Show **TC5** — switch goals to show goal update
6. Show **TC6** — switch to "After Shortcut" to show path improvement

---

## 7. Understanding the Output

### 7.1 Result Fields

| Field | Meaning |
|-------|---------|
| **Success** | Whether a valid path from start to goal was found |
| **State Path** | Ordered sequence of state IDs visited: `0 -> 1 -> 2 -> 3` |
| **Transition Path** | Ordered sequence of transition IDs used: `T0, T1, T2` |
| **Total Cost** | Sum of raw transition costs along the path |
| **Safety Score (D)** | Minimum Euclidean distance from ANY path state to the nearest bad state. Higher is safer. "INF" means no bad states exist. |
| **States Explored** | Number of states expanded by D* Lite during search |
| **Planning Time** | Wall-clock time for the initial plan (microseconds) |
| **Replan Time** | Wall-clock time for incremental replan (microseconds) |

### 7.2 What "Good" Results Look Like

- **Bad states visited: 0** — Always expected. If > 0, the planner has a bug.
- **Low cost** — The planner minimizes total transition cost.
- **High safety score** — Larger distance from bad states is better.
- **Low states explored** — Indicates efficient search (D* Lite doesn't explore unnecessary nodes).
- **Replan time < Plan time** — Incremental replanning should be faster than planning from scratch.

---

## 8. API Reference

### 8.1 Creating a Problem

```cpp
#include "d_star_lite_planner.h"

// 1. Create problem
PlanningProblem prob;
prob.initialState = 0;          // Start state ID
prob.goalState = 3;             // Goal state ID
prob.badStates = {2};           // Bad state IDs (empty if none)

// 2. Define states with d-dimensional coordinates
prob.states = {
    State(0, {0.0, 0.0}),      // State 0 at position (0, 0)
    State(1, {1.0, 0.0}),      // State 1 at position (1, 0)
    State(2, {2.0, 0.0}),      // State 2 at position (2, 0)  [BAD]
    State(3, {3.0, 0.0}),      // State 3 at position (3, 0)
};

// 3. Define directed transitions
//                    id  from  to  cost  safety  reliability  available
prob.transitions = {
    Transition(0,  0, 1,  1.0,  1.0,  1.0,  true),   // 0 -> 1
    Transition(1,  1, 3,  2.0,  1.0,  0.9,  true),   // 1 -> 3
};
```

### 8.2 Planning

```cpp
DStarLitePlanner planner;
PlanningResult result = planner.plan(prob);

if (result.success) {
    // result.statePath       = {0, 1, 3}
    // result.transitionPath  = {0, 1}
    // result.totalCost       = 3.0
    // result.safetyScore     = 1.414...
}
```

### 8.3 Incremental Replanning

```cpp
// After initial plan, apply changes and replan:

planner.updateGoal(5);                           // Change goal
planner.addBadState(3);                          // Add bad state
planner.removeBadState(2);                       // Remove bad state
planner.setTransitionAvailability(1, false);     // Disable transition
planner.addTransition(Transition(10,0,5,1,1,1,true)); // Add edge

PlanningResult newResult = planner.replan();     // Get updated path
```

### 8.4 Weight Configuration

```cpp
DStarLitePlanner planner;
planner.setWeights(
    1.0,    // beta  — cost weight (higher = penalize cost more)
    2.0,    // gamma — safety weight (higher = prefer safer paths)
    0.5     // delta — reliability weight (higher = prefer reliable edges)
);
PlanningResult result = planner.plan(prob);
```

### 8.5 Statistics

```cpp
size_t explored = planner.getExploredCount();  // States expanded in last search
```

---

## 9. Configuring Weights

The objective function is: `Score(P) = alpha*G - beta*C + gamma*D + delta*R`

The three tunable weights control the cost-safety-reliability tradeoff:

| Parameter | Default | Effect of Increasing |
|-----------|---------|---------------------|
| **beta** | 1.0 | Stronger preference for low-cost paths |
| **gamma** | 0.5 | Stronger preference for paths far from bad states |
| **delta** | 0.3 | Stronger preference for reliable transitions |

### Example Configurations

| Scenario | beta | gamma | delta | Behavior |
|----------|------|-------|-------|----------|
| Cost-only | 1.0 | 0.0 | 0.0 | Pure shortest-path (ignores safety) |
| Safety-first | 0.5 | 3.0 | 0.0 | Accepts higher cost for more safety |
| Reliability-focused | 0.5 | 0.5 | 2.0 | Prefers reliable transitions |
| Balanced | 1.0 | 0.5 | 0.3 | Default — reasonable tradeoff |

---

## 10. Creating Custom Test Cases

### Step 1: Define States

Create states with unique IDs and coordinate embeddings:

```cpp
prob.states = {
    State(0, {0.0, 0.0}),    // 2D coordinates
    State(1, {1.0, 2.0, 3.0}), // 3D works too
};
```

### Step 2: Define Transitions

```cpp
//                   id  from  to  cost  safety  reliability  available
Transition(0,  0, 1,  1.5,  1.0,  0.95,  true)
```

### Step 3: Set Bad States

```cpp
prob.badStates = {3, 7, 12};  // IDs of forbidden states
```

### Step 4: Run

```cpp
DStarLitePlanner planner;
PlanningResult result = planner.plan(prob);
```

### Step 5: Add to main.cpp

Follow the pattern of existing test cases in `src/main.cpp`. Each test function:
1. Prints an ASCII graph layout
2. Creates a PlanningProblem
3. Runs the planner
4. Calls printResult() to display results

---

## 11. Troubleshooting

| Problem | Cause | Solution |
|---------|-------|----------|
| `g++` not recognized | MinGW not installed or not in PATH | Install MinGW and add to PATH |
| Compilation errors | Wrong C++ standard | Use `-std=c++14` flag |
| Garbled output characters | Console doesn't support Unicode | Use the updated version (ASCII-only output) |
| "No path found" returned | No connected route exists | Verify transitions connect start to goal |
| Path goes through bad state | Bug (should never happen) | Report — all tests show 0 bad visits |
| High cost path chosen | Safety/reliability weights dominate | Decrease gamma/delta, increase beta |
| Slow planning on large graphs | Expected for large n, m | D* Lite is O((n+m) log n) |
| HTML demo doesn't load | File path issue | Open demo/index.html directly in browser |
