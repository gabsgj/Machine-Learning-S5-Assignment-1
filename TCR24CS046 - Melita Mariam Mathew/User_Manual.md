# PCCST503 – Machine Learning | Assignment 1
# User Manual & Developer Reference Guide
## Safe Semantic Planner in a Finite Cartesian State Space

**Department of Computer Science and Engineering**  
**Course Code:** PCCST503  
**Deliverable:** 4 — Software User Manual & Developer Handbook  

---

## 1. System Overview & Prerequisites

The **Safe Semantic Planner** is a high-performance C++17 planning engine with an interactive web visualizer designed to solve safe trajectory computation problems in discrete Cartesian state spaces $\mathbb{R}^d$.

### System Requirements:
- **Operating Systems:** Windows 10/11, Linux (Ubuntu 20.04+), macOS (11.0+)
- **C++ Compiler:** Any C++17 or C++20 compliant compiler:
  - GCC 9.0+ / MinGW-W64
  - Clang / LLVM 10.0+
  - Microsoft Visual Studio C++ (MSVC 2019+)
- **Build Tools (Optional):** CMake 3.16+
- **Web Browser (For Interactive Demonstration):** Any modern browser (Chrome, Edge, Firefox, Safari).

---

## 2. Quickstart & Compilation Instructions

### 2.1 One-Click Build & Run (Windows)
Run the pre-configured automated build script in PowerShell or Command Prompt:

```cmd
.\run.bat
```
Or with PowerShell:
```powershell
.\run.ps1
```

### 2.2 Direct Compilation with G++ / Clang++
```bash
# Compile with C++17 and optimization
g++ -std=c++17 -O3 -Iinclude src/*.cpp -o SafeSemanticPlanner.exe

# Execute test suite & benchmarks
./SafeSemanticPlanner.exe
```

### 2.3 Building with CMake
```bash
mkdir build
cd build
cmake ..
cmake --build . --config Release
./SafeSemanticPlanner
```

---

## 3. Software Interfaces & Class Reference

### 3.1 `State` Class
Represents a discrete semantic entity with a continuous $d$-dimensional coordinate:
```cpp
#include "State.hpp"

// Construct State: id, embedding vector
State s1(0, {1.5, 2.0, 0.0});
State s2(1, {4.0, 6.0, 0.0});

// Compute Euclidean distance in R^d
double dist = s1.euclideanDistance(s2);
```

### 3.2 `Transition` Class
Represents a directed link between two states:
```cpp
#include "Transition.hpp"

// Transition: id, from, to, cost, safety, reliability, available
Transition t(1, 0, 1, 3.5, 1.0, 0.98, true);
```

### 3.3 `PlanningProblem` Class
Encapsulates problem parameters:
```cpp
#include "PlanningProblem.hpp"

std::vector<State> states = { ... };
std::vector<Transition> transitions = { ... };
std::vector<uint64_t> badStates = { 4, 7 }; // Hazardous states

PlanningProblem problem(
    0,           // Initial State ID (s_I)
    3,           // Goal State ID (s_G)
    badStates,   // Hazardous State IDs
    states,      // Vector of States
    transitions  // Vector of Transitions
);
```

### 3.4 `Planner` Interface & `LPAStarPlanner`
Abstract planning base class and incremental search engine:
```cpp
#include "LPAStarPlanner.hpp"

// Weights: alpha (goal), beta (cost), gamma (safety), delta (reliability), safetyRadius
LPAStarPlanner planner(100.0, 1.0, 5.0, 2.0, 3.0);

// Compute initial shortest safe path
PlanningResult result = planner.plan(problem);

if (result.success) {
    std::cout << "Optimal Path: " << result.pathString() << "\n";
    std::cout << "Total Cost  : " << result.totalCost << "\n";
    std::cout << "Min Clearance: " << result.minSafetyDistance << "\n";
}
```

---

## 4. Dynamic Replanning API

The planner supports real-time dynamic environment modifications without full graph reconstruction:

### 4.1 Modifying Edge Availability or Cost
```cpp
// Disable transition with ID = 2 (e.g. edge blockage)
planner.updateTransition(2, false);

// Re-run incremental search
PlanningResult replanned = planner.computeShortestPath();
```

### 4.2 Adding a New Shortcut Transition
```cpp
// Insert a new high-speed bridge transition
Transition shortcut(100, 1, 5, 1.2, 1.0, 0.99, true);
planner.addTransition(shortcut);

PlanningResult replanned = planner.computeShortestPath();
```

### 4.3 Dynamically Updating the Goal State
```cpp
// Change goal state to State ID = 9
planner.updateGoal(9);

PlanningResult replanned = planner.computeShortestPath();
```

### 4.4 Dynamic Obstacle (Bad State) Updates
```cpp
// Add new bad states dynamically
planner.updateBadStates({ 4, 7, 8 });

PlanningResult replanned = planner.computeShortestPath();
```

---

## 5. Illustrative Test Scenarios Guide

The system includes built-in automated test scenarios matching the assignment requirements:

| Test ID | Method | Description |
|---|---|---|
| **1** | `TestScenarios::runTestCase1()` | Basic reachability: $S \to A \to B \to G$. |
| **2** | `TestScenarios::runTestCase2()` | Bad state avoidance: ensures hazardous node $X$ is never visited. |
| **3** | `TestScenarios::runTestCase3()` | Safety margin clearance: verifies selection of high-clearance bypass. |
| **4** | `TestScenarios::runTestCase4()` | Dynamic edge failure: tests incremental rerouting when $(A,G)$ fails. |
| **5** | `TestScenarios::runTestCase5()` | Dynamic goal update: verifies key-shifted replanning to new target. |
| **6** | `TestScenarios::runTestCase6()` | Shortcut insertion: confirms instant adoption of newly added edges. |

To run all test cases programmatically:
```cpp
std::vector<TestCaseResult> results = TestScenarios::runAllTestCases();
```

---

## 6. Interactive Web Demonstration

To launch the visual demonstration dashboard:
1. Navigate to the `demo/` directory in the repository.
2. Open `index.html` in any modern web browser.
3. Use the top toolbar to:
   - Select and load any of the **Test Cases (1 to 6)** or generate a **Random Graph**.
   - Adjust objective weights ($\alpha, \beta, \gamma, \delta$) via interactive sliders.
   - Click nodes to toggle **Bad State (Hazard)** status in real time.
   - Click edges to **cut / disable** them and observe instant real-time incremental replanning!
   - Step through the search process to inspect $g(s)$, $rhs(s)$, and priority queue keys.

---

## 7. Troubleshooting & FAQ

**Q1: `g++` is not recognized on Windows.**  
**A:** Ensure MinGW-W64 or WinLibs is added to your system `PATH`, or use the bundled `run.bat` script which automatically resolves the local compiler path.

**Q2: How do I change the embedding dimensionality from 2D to $N$-D?**  
**A:** `State::embedding` is an arbitrary-dimension `std::vector<double>`. The Euclidean distance and heuristic calculations automatically adapt to any dimensionality $d \ge 1$.

**Q3: How are bad states avoided?**  
**A:** Transitions leading to or originating from bad states have an effective cost of $\infty$, ensuring mathematical pruning during graph relaxation. Proximity to bad states is continuously penalized through a repulsive barrier potential.
