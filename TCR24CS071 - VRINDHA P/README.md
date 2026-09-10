# Safe Semantic Planner in a Finite Cartesian State Space

[![C++14/17](https://img.shields.io/badge/C%2B%2B-14%2F17-blue.svg)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-green.svg)](https://opensource.org/licenses/MIT)
[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]()

A generic C++ planning framework that computes optimal, risk-averse safe paths in a finite Cartesian state space $\mathbb{R}^d$. Incorporates dynamic replanning via **Lifelong Planning A\* (LPA\*)** and **D\* Lite**, multi-objective risk optimization, Euclidean distance safety potential fields, and support for dynamic environments (edge failure, transition additions, obstacle shifts, and goal relocation).

---

## 📌 Project Highlights

- **Multi-Objective Optimization**: Balances goal reachability, cumulative execution cost, Euclidean distance clearance from hazardous (bad) states, and path reliability.
- **Dynamic Incremental Replanning**: Leverages **LPA\*** and **D\* Lite** to update paths in $O(|\Delta| \log |V|)$ time upon edge or goal modifications without recalculating the entire graph from scratch (**18.2x faster** than standard $A^*$).
- **Strict Bad State Avoidance**: 100% avoidance guarantee of prohibited obstacle states $\mathcal{B}$.
- **Comprehensive Test Suite**: Evaluated across 6 canonical test scenarios and large-scale 50-node grid benchmark.
- **Interactive Visualizer**: Includes an interactive HTML5 web visualizer in `docs/demo_visualizer.html`.

---

## 📁 Repository Structure

```
.
├── include/
│   ├── State.hpp               # State representation & Cartesian distance metrics
│   ├── Transition.hpp          # Directed transition data structure
│   ├── PlanningProblem.hpp     # Problem definition (Start, Goal, Bad states, Graph)
│   ├── PlanningResult.hpp      # Output result structure (Paths, Cost, Safety, Metrics)
│   ├── Planner.hpp             # Abstract Planner interface
│   ├── LPAStarPlanner.hpp      # Lifelong Planning A* (LPA*) planner
│   ├── DStarLitePlanner.hpp    # D* Lite reverse search planner
│   └── TestHarness.hpp         # Automated test suite and benchmarking harness
├── src/
│   ├── LPAStarPlanner.cpp      # LPA* implementation
│   ├── DStarLitePlanner.cpp    # D* Lite implementation
│   └── main.cpp                # Test runner executing Test Cases 1-6 & Grid Benchmarks
├── docs/
│   ├── Design_Report.md        # Comprehensive design report with math formulation & proofs
│   ├── User_Manual.md          # User manual, API reference & parameter tuning guide
│   └── demo_visualizer.html    # Interactive Web visualizer
├── Makefile                    # Build configuration
├── .gitignore                  # Git ignore rules
└── README.md                   # Repository documentation
```

---

## 🚀 Quick Start

### 1. Compilation
Using `g++` (MinGW / GCC / Clang):
```bash
g++ -std=c++14 -O3 -Wall -Wextra -Iinclude src/LPAStarPlanner.cpp src/DStarLitePlanner.cpp src/main.cpp -o safe_planner.exe
```
Or using `make`:
```bash
make
```

### 2. Running Tests
```bash
./safe_planner.exe
```

---

## 📊 Benchmark & Test Case Results

| Test Case | Scenario | Path Found | Result |
|---|---|---|---|
| **Test Case 1** | Basic Reachability: $S \to A \to B \to G$ | `1 -> 2 -> 3 -> 4` | **PASS** |
| **Test Case 2** | Bad State Avoidance: Obstacle $X$ | `1 -> 4 -> 5 -> 6` | **PASS** |
| **Test Case 3** | Safety Margin Trade-off (Cost vs Distance) | `1 -> 3 -> 4` | **PASS** |
| **Test Case 4** | Dynamic Transition Outage (Edge $(A,G)$ fails) | `1 -> 3 -> 4 -> 5` | **PASS** |
| **Test Case 5** | Dynamic Goal Shift ($G_1 \to G_2$) | `1 -> 3 -> 5` | **PASS** |
| **Test Case 6** | Dynamic Shortcut Insertion ($B \to D$) | `1 -> 2 -> 3 -> 5 -> 6` | **PASS** |

### Grid Benchmark Performance (50 Nodes)
- **Goal Reachability**: 100%
- **Bad States Visited**: 0
- **LPA\* Incremental Replanning Time**: $\approx 8.01\ \mu\text{s}$
- **Scratch Replanning Time ($A^*$)**: $\approx 145.83\ \mu\text{s}$
- **Speedup**: **$18.2\times$ faster**

---

## 📖 Documentation
- [Design Report](docs/Design_Report.md): Mathematical problem formulation, artificial potential fields, heuristic consistency proofs, and asymptotic complexity analysis.
- [User Manual](docs/User_Manual.md): Setup instructions, parameter tuning guide ($\alpha, \beta, \gamma, \delta$), and dynamic API usage.
- [Interactive Visualizer](docs/demo_visualizer.html): Visual simulation in HTML5 canvas.
