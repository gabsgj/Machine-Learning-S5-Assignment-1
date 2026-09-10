# User Manual: Safe Semantic Planner in a Finite Cartesian State Space

**Course**: PCCST503 – Machine Learning  
**Assignment 1**: Safe Semantic Planner  

---

## 1. Project Overview

The **Safe Semantic Planner** is a high-performance C++ software framework for computing risk-averse, optimal paths in finite Cartesian state spaces $\mathbb{R}^d$. It features real-time dynamic replanning powered by **Lifelong Planning A\* (LPA\*)** and **D\* Lite**, ensuring fast path repair when obstacles appear, edge costs change, or goals shift.

---

## 2. Directory Structure

```
MLassi/
├── include/
│   ├── State.hpp               # State representation & Cartesian distance metrics
│   ├── Transition.hpp          # Directed transition data structure
│   ├── PlanningProblem.hpp     # Problem definition (Start, Goal, Bad states, Graph)
│   ├── PlanningResult.hpp      # Output result (Paths, Cost, Safety, Metrics)
│   ├── Planner.hpp             # Abstract Planner interface
│   ├── LPAStarPlanner.hpp      # Lifelong Planning A* (LPA*) engine
│   ├── DStarLitePlanner.hpp    # D* Lite reverse dynamic search engine
│   └── TestHarness.hpp         # Automated test suite & benchmarking harness
├── src/
│   ├── LPAStarPlanner.cpp      # LPA* algorithm implementation
│   ├── DStarLitePlanner.cpp    # D* Lite algorithm implementation
│   └── main.cpp                # Test runner executing Test Cases 1-6 & Grid Benchmarks
├── docs/
│   ├── Design_Report.md        # Mathematical formulation, proofs, and complexity
│   ├── User_Manual.md          # User manual and configuration guide
│   └── demo_visualizer.html    # Interactive Web visualizer for simulation
├── Makefile                    # MinGW GCC build configuration
└── safe_planner.exe            # Compiled native executable
```

---

## 3. Build & Compilation Instructions

### Prerequisites
- Any C++14/C++17 compatible compiler (e.g. MinGW GCC `g++`, Clang, or MSVC).

### Compiling on Windows (PowerShell / Command Prompt)
Using `g++`:
```powershell
g++ -std=c++14 -O3 -Wall -Wextra -Iinclude src/LPAStarPlanner.cpp src/DStarLitePlanner.cpp src/main.cpp -o safe_planner.exe
```

Or using `make` (if GNU Make is installed):
```powershell
make
```

---

## 4. Running the Test Suite & Benchmarks

To execute the test harness running all 6 assignment test cases plus the 50-node grid benchmark:
```powershell
.\safe_planner.exe
```

### Expected Output
The program executes sequentially:
1. **Test Case 1 (Basic Reachability)**: Returns unique path $S \to A \to B \to G$.
2. **Test Case 2 (Bad State Avoidance)**: Avoids bad state $X$, selecting $S \to C \to D \to G$.
3. **Test Case 3 (Safety Margin)**: Trade-off between short risky path and safe detour.
4. **Test Case 4 (Dynamic Transition)**: Fast incremental replan when edge $(A,G)$ fails.
5. **Test Case 5 (Dynamic Goal Update)**: Fast goal redirection without graph re-allocation.
6. **Test Case 6 (Transition Addition)**: Incremental path optimization when shortcut $(B,D)$ is added.
7. **Empirical Grid Benchmark**: Measures speedup, memory overhead, and state expansions.

---

## 5. Configuration & Parameter Tuning

The objective function is configured via `PlannerWeights`:
$$\text{Score}(P) = \alpha G - \beta C + \gamma D_{\min} + \delta R$$

```cpp
#include "LPAStarPlanner.hpp"

PlannerWeights weights;
weights.alpha = 100.0;                 // Goal reachability weight
weights.beta  = 1.0;                   // Cumulative transition cost weight
weights.gamma = 15.0;                  // Safety distance repulsion weight
weights.delta = 2.0;                   // Reliability weight (-ln(r))
weights.safeDistanceThreshold = 0.1;   // Minimum strict clearance threshold

LPAStarPlanner planner(weights);
PlanningResult result = planner.plan(problem);
```

### Parameter Tuning Guide
- **Aggressive / Cost-Optimized**: Set $\gamma = 0.5$, $\beta = 2.0$. The agent prioritizes short paths even if closer to obstacles.
- **Safety-Critical**: Set $\gamma = 25.0$, $\text{safeDistanceThreshold} = 0.5$. The agent selects wider detours around hazards.
- **High-Reliability**: Set $\delta = 10.0$. The agent avoids low-reliability links even if cheaper.

---

## 6. Dynamic Replanning API Guide

### 1. Updating Edge Costs or Availability
```cpp
// Disable edge 402 dynamically:
planner.updateTransition(402, /*cost=*/1.0, /*safety=*/1.0, /*reliability=*/0.99, /*available=*/false);
PlanningResult updated = planner.replan();
```

### 2. Adding Dynamic Shortcut
```cpp
Transition shortcut(999, /*from=*/3, /*to=*/7, /*cost=*/0.5);
planner.addTransition(shortcut);
PlanningResult updated = planner.replan();
```

### 3. Dynamic Goal Modification
```cpp
planner.updateGoal(newGoalId);
PlanningResult updated = planner.replan();
```

---

## 7. Interactive Demonstration

An interactive, responsive HTML5 visualizer is provided in [docs/demo_visualizer.html](file:///c:/Users/subra/Downloads/MLassi/docs/demo_visualizer.html). Simply open this file in any web browser to:
- Visually inspect the Cartesian 2D/3D state embeddings.
- Interact with all 6 test scenarios.
- Trigger dynamic environmental updates (blocking edges, moving obstacles, updating goals).
- View step-by-step LPA* key values and replanning animations.
