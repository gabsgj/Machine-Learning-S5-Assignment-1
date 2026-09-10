<div align="center">
  <h1>🛡️ Safe Semantic Planner</h1>
  <p><i>A C++17 safety-aware path planner for finite Cartesian state spaces</i></p>

  [![C++](https://img.shields.io/badge/C++-17-blue.svg?style=flat-square&logo=c%2B%2B)](https://isocpp.org/)
  [![Build](https://img.shields.io/badge/build-passing-brightgreen.svg?style=flat-square)](#)
  [![Course](https://img.shields.io/badge/course-PCCST503-orange.svg?style=flat-square)](#)
</div>

---

## 📖 Overview

The **Safe Semantic Planner** is a heuristic-based pathfinding algorithm implementation that ensures optimal pathing while avoiding dangerous states. By balancing traditional distance costs with safety penalties, the planner dynamically adapts to hazardous environments. 

Developed as **Assignment 1** for the **PCCST503 — Machine Learning** course.

---

## ✨ Features

- **Euclidean Heuristics**: Admissible and consistent estimations in $\mathbb{R}^d$ space.
- **Dynamic Safety Computation**: Path costs are inversely penalized based on the minimum distance to defined "bad states".
- **Real-Time Adaptability**: Effectively recalculates and routes around newly updated obstacles or removed transitions.
- **Configurable Risk Factors**: Tune your safety margins on the fly via `$\alpha, \beta, \gamma$` parameters.

---

## 🛠️ Building the Project

The planner is lightweight and can be built using any modern C++ compiler. 

**Prerequisites**: A compiler supporting C++17 (`GCC 9+`, `Clang 10+`, `MSVC 2019+`).

```bash
# Compile the main application and planner logic
g++ -std=c++17 main.cpp planner.cpp -o planner_test
```

---

## 🚀 Running the Tests

Once built, execute the test suite which contains 6 unique scenarios validating the planner's edge-case handling:

```bash
./planner_test
```

### Expected Output
The test suite will print the paths, calculated cost, and safety margin for each of the following scenarios:
- 🟢 **Test 1**: Basic Reachability
- 🛑 **Test 2**: Bad State Avoidance *(Rerouting around danger)*
- 📐 **Test 3**: Safety Margin Evaluation
- 🔄 **Test 4**: Dynamic Transition *(Handling removed critical edges)*
- 🎯 **Test 5**: Goal State Updates
- 🛤️ **Test 6**: Transition Addition *(Exploiting new shortcuts)*

---

## 📁 Project Structure

```text
📦 ml
 ┣ 📜 Report.md          # Comprehensive design report & metrics
 ┣ 📜 assignment.pdf     # Original assignment specification
 ┣ 📜 planner.hpp        # Data structures and interface declarations
 ┣ 📜 planner.cpp        # Core algorithm implementation
 ┗ 📜 main.cpp           # Entry point and test harness
```

---

## ⚙️ How It Works

### Safety Computation
Safety is dynamically determined by finding the minimum Euclidean distance from a given state to any known **bad state**. 
- If a state is explicitly listed as a bad state, its safety distance is `0.0`.
- For other states, the path cost is penalized inversely proportional to this safety margin.

### The Heuristic
The algorithm employs the **Euclidean distance** between the current state and the goal state embedding. This guarantees an **admissible and consistent** heuristic, ensuring the shortest path is found optimally.

---

## 🎛️ Tuning

You can easily adjust how the planner balances distance vs. safety by configuring the weights on the `SafePlanner` instance. 

```cpp
SafePlanner planner;

// Adjust core parameters
planner.setAlpha(1.0);   // Base heuristic weight
planner.setBeta(1.0);    // Transition cost weight
planner.setGamma(10.0);  // 🛡️ Safety penalty weight

// Execute planning
PlanningResult result = planner.plan(problem);
```

---
