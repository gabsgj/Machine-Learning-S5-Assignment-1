# User Manual

## Safe Semantic Planner

### 1. Overview

The Safe Semantic Planner is a C++17 program that finds a path from an initial state to a goal state in a finite Cartesian state space.

The planner considers:

- Path cost
- State safety
- Transition safety
- Transition reliability
- Bad states
- Dynamic changes to the graph

The program automatically runs six test cases when executed.

---

## 2. Software Requirements

The following software is required:

- C++17-compatible compiler
- Git
- GitHub account

The program can be compiled on:

- Windows
- Linux
- macOS

---

## 3. Project Structure

```text
safe-semantic-planner/

├── src/
│   └── main.cpp
│
├── results/
│   └── results.txt
│
├── screenshots/
│   ├── test_case_1_3.png
│   ├── test_case_4_5.png
│   └── test_case_5_6.png
│
├── docs/
│   ├── design_report.md
│   └── user_manual.md
│
├── README.md
└── .gitignore