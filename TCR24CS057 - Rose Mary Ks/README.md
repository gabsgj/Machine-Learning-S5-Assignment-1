# Safe Semantic Planner

**PCCST503 - Machine Learning — Assignment 1**

NAME: ROSE MARY K S

REGISTER NUMBER:TCR24CS057

## Objective

Design and implement a planner that finds a safe path in a finite Cartesian state space while:

- Reaching the goal
- Avoiding bad states
- Minimizing transition cost
- Maximizing distance from bad states
- Considering transition reliability
- Supporting dynamic replanning

## Algorithm

The implementation uses a **D* Lite-style incremental replanning approach** with:

- Euclidean-distance heuristic
- Directed graph
- `g` and `rhs` values
- Priority queue
- Safety-aware edge cost
- Bad-state filtering

## Project Structure

```text
safe-semantic-planner/

├── src/
│   └── main.cpp
│
├── results/
│   └── results.txt
│
├── docs/
│   ├── design_report.pdf
│   └── user_manual.pdf

│
├── screenshots/
│   ├── test_case_1_3.png
│   ├── test_case_4_5.png
│   └── test_case_5_6.png
│
├── README.md
└── .gitignore
