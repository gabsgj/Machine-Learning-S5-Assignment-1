# Safe Semantic Planner in a Finite Cartesian State Space

[![C++17](https://img.shields.io/badge/C%2B%2B-17%2F20-blue.svg)](https://isocpp.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Build Status](https://img.shields.io/badge/Build-Passing-brightgreen.svg)]()
[![Frontend: Interactive Visualizer](https://img.shields.io/badge/GUI-Web_Visualizer-cyan.svg)]()

> **PCCST503 — Machine Learning: Assignment 1**  
> **Department of Computer Science & Engineering**

---

## 🌟 Executive Overview

The **Safe Semantic Planner** is a high-performance, safety-critical planning system operating over a finite Cartesian state space $\mathcal{S} \subset \mathbb{R}^d$. It computes optimal, collision-free paths from an initial state $s_I$ to a goal state $s_G$, enforcing **hard bad-state avoidance invariants** and **multi-criteria safety margin optimization**.

Built around **Lifelong Planning A\* (LPA\*)** and **D\* Lite**, the planner excels in **dynamic, non-stationary environments**, reusing search trees to achieve up to **49.3× faster replanning** when edges fail, shortcuts appear, obstacles appear, or goals shift.

Additionally, this project features a **state-of-the-art Web Frontend Visualizer** providing real-time canvas interaction, step-by-step search inspection, dynamic obstacle injection, and multi-objective weight tuning.

---

## 📁 Repository Structure

```
.
├── include/                       # Modular C++17 Header Architecture
│   ├── types.hpp                  # Exact Assignment Interfaces (State, Transition, Problem, Result, Planner)
│   ├── geometry.hpp               # Euclidean distance & Cartesian safety metrics in R^d
│   ├── heuristic.hpp              # Admissible scaled Euclidean Cartesian heuristics
│   ├── lpa_star.hpp               # Lifelong Planning A* (LPA*) incremental engine
│   ├── d_star_lite.hpp            # D* Lite backward incremental planner
│   ├── multi_objective.hpp        # Multi-objective score & multi-goal sequencing
│   ├── test_cases.hpp             # Test Cases 1 through 6 + Semantic Knowledge Graph
│   └── benchmark.hpp              # Stress test suite on random geometric graphs (N=50 to 2000)
│
├── src/
│   └── main.cpp                   # C++ CLI Entry Point with ANSI formatted reporting
│
├── frontend/                      # Interactive Modern Web GUI Visualizer
│   ├── index.html                 # Single-page visualizer layout
│   ├── style.css                  # Dark glassmorphic styling, neon glows, responsive grid
│   ├── app.js                     # 2D Canvas engine, drag & drop, particle trails, telemetry
│   ├── engine.js                  # In-browser JS LPA* engine mirroring C++ core
│   └── scenarios.js               # Preset test scenarios & benchmarks
│
├── data/                          # Problem definitions in JSON format
│   ├── test_case_1_reachability.json
│   ├── test_case_2_bad_state_avoidance.json
│   ├── test_case_3_safety_margin.json
│   ├── test_case_4_dynamic_transition.json
│   ├── test_case_5_goal_update.json
│   ├── test_case_6_transition_addition.json
│   └── knowledge_graph_semantic.json
│
├── bin/                           # Compiled binaries
│   └── planner.exe                # High-performance C++ executable
│
├── build.bat                      # Windows one-click compile script (g++ -O3 -Wall -Wextra)
├── run.bat                        # Windows one-click execution script
├── DESIGN_REPORT.md               # 28-Section Academic Design Report
├── EXPERIMENTAL_RESULTS.md        # Comprehensive Empirical Benchmark Tables
├── USER_MANUAL.md                 # CLI & GUI User Manual
└── DEMONSTRATION.md               # Viva demonstration script for Test Cases 1 to 6
```

---

## ⚡ Quick Start

### 1. Build and Run the C++ Core Engine

```bash
# Windows (cmd/PowerShell)
build.bat
run.bat

# Or direct compilation via g++ (Linux / macOS / MinGW):
g++ -std=c++17 -O3 -Wall -Wextra -Iinclude src/main.cpp -o bin/planner
./bin/planner --test
```

### 2. Launch the Interactive Web Visualizer

Open `frontend/index.html` in any modern web browser or start a local server:

```bash
python -m http.server 8080 --directory frontend
# Navigate to http://localhost:8080
```

---

## 🔬 Core Highlights & Capabilities

| Feature | Description | Assignment Requirement |
|---|---|---|
| **LPA\* Core Engine** | Forward incremental search with $O(1)$ local vertex consistency updates and warm restart. | Core Algorithm |
| **D\* Lite Option** | Backward incremental search comparing goal-dependent vs start-dependent $g$-values. | Algorithm Guidelines |
| **Hard Bad-State Avoidance** | Infinite cost & structural exclusion from priority queue guaranteeing 0 bad visits. | Hard Constraint |
| **Multi-Objective Balancing** | Optimization of $Score(P) = \alpha G - \beta C + \gamma D + \delta R$ with customizable safety barriers. | Objective 4 & Score Function |
| **Dynamic Replanning** | Real-time response to edge toggles, cost fluctuations, goal updates, and obstacle insertions. | Dynamic Environment |
| **Interactive Web GUI** | Real-time 2D Cartesian canvas, drag-and-drop state positioning, step-by-step search playback, and live telemetry. | Deliverable 5 & GUI |
| **Knowledge Graph Navigation** | Semantic concept navigation in $\mathbb{R}^d$ avoiding poisoned/malicious entities. | Bonus Deliverable |

---

## 📊 Summary of Experimental Benchmarks

Empirical performance evaluation across all 6 assignment test cases and large-scale random geometric graphs:

```
====================================================================================================
Graph Scale   States    Edges     Cold Start (us) Incremental (us)  Speedup     Cold Exp.     Incr. Exp.    
----------------------------------------------------------------------------------------------------
Scale_N50     50        614       399.3 us        8.1 us            49.29x      17            3             
Scale_N100    100       1586      694.4 us        101.4 us          6.85x       21            8             
Scale_N250    250       5310      1437.0 us       65.2 us           22.03x      6             4             
Scale_N500    500       10156     2378.4 us       68.3 us           34.82x      38            11            
Scale_N1000   1000      21360     5443.3 us       134.7 us          40.41x      89            18            
Scale_N2000   2000      43078     12176.1 us      679.9 us          17.90x      149           42            
====================================================================================================
```

---

## 📖 Complete Documentation Suite

- 📄 [DESIGN_REPORT.md](DESIGN_REPORT.md): Comprehensive 28-section academic design report covering mathematical proofs, data structures, heuristic design, safety barriers, and complexity.
- 📈 [EXPERIMENTAL_RESULTS.md](EXPERIMENTAL_RESULTS.md): Empirical test case outputs, speedup plots, and comparative analysis.
- 📘 [USER_MANUAL.md](USER_MANUAL.md): Step-by-step compilation, CLI arguments, JSON schemas, and web visualizer operation guide.
- 🎯 [DEMONSTRATION.md](DEMONSTRATION.md): Scripted viva demonstration covering Test Cases 1 through 6 with ASCII graph diagrams and evaluation checkpoints.
