# 🛡️ Safe Semantic Planner
> A professional implementation of Lifelong Planning A* (LPA*) featuring safety-aware heuristics and dynamic graph updates for autonomous navigation.

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![C++17](https://img.shields.io/badge/C++-17-blue.svg)](https://en.cppreference.com/w/cpp/17)
[![CMake](https://img.shields.io/badge/CMake-3.10+-green.svg)](https://cmake.org/)

---

### 👤 Student Information
* **Name:** Harikrishnan M  
* **University Reg No:** TCR24CS032  
* **Department:** Computer Science Engineering  
* **Project:** Safe Semantic Planner (LPA* & Safety-Aware Motion Planning)  

---
## 📖 Project Overview

This repository features a robust, high-performance C++ implementation of a **Safe Semantic Planner**. The project bridges standard heuristic graph search with real-time safety constraints, utilizing the **Lifelong Planning A* (LPA*)** algorithm to efficiently handle dynamic environmental changes like blocked transitions, added shortcuts, and shifting goals.

Unlike traditional planners that only minimize distance, this system continuously evaluates spatial safety margins against known hazards and bad states, ensuring reliable, collision-free paths.

### 🧩 Core Functionality
* **Incremental Heuristic Search:** Implements LPA* to quickly adapt paths when graph topologies shift without restarting searches from scratch.
* **Semantic Safety Margins:** Integrates Euclidean-based spatial distance metrics to penalize or prohibit proximity to hazardous regions.
* **Dynamic Environment Adaptation:** Supports real-time updates including node additions, transition removals, and goal re-targeting.
* **Rigorous Automated Testing:** Features a comprehensive CTest suite validating reachability, hazard avoidance, safety buffers, and dynamic replanning.

### 🔬 Why This Matters
Safe motion planning is critical for **Autonomous Mobile Robots (AMRs)**, **UAVs**, and intelligent transport systems operating in unstructured or dynamic environments. This project demonstrates how semantic awareness and incremental search algorithms can work together to guarantee both safety and operational efficiency.

### ⚡ Key Features
* **LPA* Algorithm Core:** Optimized priority-queue state management using `std::unordered_map` and `std::set`.
* **Pro Verification:** Structured architecture with decoupled core modules (`environment`, `heuristics`, `planner`).
* **Automated CTest Suite:** Seamless unit-testing pipeline for validation and performance benchmarking.

---

## 🛠️ Installation & Usage

```bash
git clone [https://github.com/ha7-piixel/SafeSemanticPlanner.git](https://github.com/ha7-piixel/SafeSemanticPlanner.git)
cd SafeSemanticPlanner

mkdir -p build && cd build
cmake ..
make

# Run the automated test suite
ctest --output-on-failure
