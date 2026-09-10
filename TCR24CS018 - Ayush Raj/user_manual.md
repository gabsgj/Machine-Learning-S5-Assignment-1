# User Manual — Quality-Aware D* Lite Engine

This document provides a comprehensive guide to building, running, testing, and understanding the core functionality of the Quality-Aware D* Lite engine. This manual deliberately focuses exclusively on the C++ core engine and algorithms (excluding the UI layer). For a brief project overview, see `README.md`.

---

## 1. System Requirements & Prerequisites

The engine is built around modern C++ and depends strictly on the standard library.

| Dependency | Purpose | How to Check |
|---|---|---|
| C++17 Compiler (g++, clang++, MSVC) | Compiling the core engine | `g++ --version` |
| CMake (3.16+) | Generating the build system | `cmake --version` |
| Make / Ninja | Building the targets | `make --version` or `ninja --version` |

There are no external library requirements to build and run the core engine or its tests.

---

## 2. Building the Engine

From the project root directory, configure the build using CMake and then compile it:

```bash
# 1. Configure the build directory
cmake -S . -B build

# 2. Build the targets
cmake --build build
```

This sequence compiles the core library (`quality_dstar_lite`), the demonstration executable (`quality_dstar_demo`), and the test suite (`quality_dstar_tests`).

---

## 3. Running the Planner Demonstration

After a successful build, you can run the primary demonstration executable:

```bash
./build/quality_dstar_demo
```

**Expected Output:**
The demo instantiates a `DStarLitePlanner` and sets up a basic spatial planning problem. The output will print the path taken by the agent, cost breakdowns, and safety scores.

**How to Interpret the Results:**
- **Path**: The sequence of state IDs from the `initialState` to the `goalState`.
- **Total Cost**: The cumulative sum of raw transition costs along the chosen path.
- **Safety Score**: A normalized metric (0.0 to 1.0) indicating the minimum clearance from `badStates` across the chosen path.
- **Reliability**: A multiplied probability representing how likely the path is to be traversed successfully without edge failures.

---

## 4. Running the Test Suite

To ensure the engine behaves correctly on your system, you can run the integrated tests:

```bash
cd build
ctest --output-on-failure
```
Alternatively, directly run the test binary:
```bash
./build/quality_dstar_tests
```

**What to Look For:**
The test suite validates basic reachability, edge failure handling, bad state avoidance, and dynamic replanning (e.g., changes in goal state or edge costs). All tests should pass. If you encounter failures, your local repository state might have diverged (see Troubleshooting).

---

## 5. Core Concepts & Configuration

### Data Structures
The engine operates on several key primitive objects defined in `interfaces.hpp`:
- **State**: A discrete point in the environment (`id`, `embedding`).
- **Transition**: A directed edge between two states (`from`, `to`, `cost`, `reliability`, `available`).
- **PlanningProblem**: Holds `states`, `transitions`, `initialState`, `goalState`, and a list of `badStates`.

### The Quality-Aware Heuristic
Unlike standard D* Lite, this engine uses a multi-objective key function that weighs the travel cost against a composite "quality" score.

The quality of an edge is calculated as:
```cpp
quality = (alpha * reliability) + (beta * continuationSafety)
```

### Tuning Quality Weights
You can adjust the planner's behavior by altering the `alpha` and `beta` parameters (which must sum to 1.0):
```cpp
DStarLitePlanner planner(0.6, 0.4); // default: 60% reliability focus, 40% safety focus
planner.setQualityWeights(0.2, 0.8); // switch to heavily prioritize safety
```

### Replanning
If the environment changes (e.g., an edge becomes blocked, or a state becomes "bad"):
1. Update your `PlanningProblem` object.
2. Call `planner.plan(updatedProblem)`.
The planner leverages backward search state to efficiently recalculate the optimal path without restarting from scratch, unless the goal or bad states change substantially.

---

## 6. Troubleshooting

**"quality weights must be non-negative and sum to one" Exception**
You passed invalid weights to the `DStarLitePlanner` constructor or `setQualityWeights`. Ensure `alpha + beta == 1.0` and both are `>= 0`.

**Path Extraction Fails (Empty Result)**
If `plan()` returns a `PlanningResult` with `success = false` or an empty `statePath`:
1. The goal might be completely unreachable due to missing/unavailable transitions.
2. The start or goal state is listed in `badStates`.
3. The graph may be entirely disconnected.

**CMake complains about C++17**
Ensure your compiler is updated. On Ubuntu, `sudo apt install build-essential` usually resolves this. The engine uses modern features like structured bindings and `std::unordered_map` operations that require C++17 support.

**Full Reset**
If things get tangled, clear out your build directory and start over:
```bash
rm -rf build/
cmake -S . -B build
cmake --build build
```
