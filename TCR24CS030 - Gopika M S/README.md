PCCST503 – Safe Semantic Planner

Assignment 1

Design of a Safe Semantic Planner in a Finite Cartesian State Space

Department: Computer Science and Engineering
Implementation Language: Python 3
Algorithm: D* Lite

---

1. Project Description

This project implements a safe path planner for a finite Cartesian state space.

The planner receives:

- Initial state
- Goal state
- Bad states
- Directed transitions
- Transition cost
- Safety information
- Reliability
- Transition availability

The planner computes a path from the initial state to the goal while avoiding bad states.

---

2. Main Objectives

The planner aims to:

1. Reach the goal state.
2. Avoid all bad states.
3. Minimize total transition cost.
4. Maintain a safe distance from bad states.
5. Consider transition reliability.
6. Support dynamic changes in the environment.
7. Replan when transitions or goals change.

---

3. Algorithm

The implementation uses the D Lite* path-planning algorithm.

D* Lite is suitable for dynamic environments because it can update an existing plan when the environment changes instead of performing a completely new search every time.

---

4. Project Files

File| Description
"planner.py"| Main D* Lite planner implementation
"testcases.py"| Test scenarios and test execution
"test.py"| Runs experiments and generates CSV results
"experimental_results.csv"| Experimental measurements
"design_report.md"| Detailed design report
"user_manual.md"| Instructions for running the project
".gitignore"| Files ignored by Git

---

5. Test Scenarios

The project evaluates:

- Basic reachability
- Bad-state avoidance
- Safety margin
- Dynamic transition failure
- Dynamic goal update
- Transition addition / shortcut

---

6. How to Run

Make sure Python 3 is installed.

Run:

python test.py

The program displays the test results and generates:

experimental_results.csv

---

7. Expected Behaviour

A successful plan should:

- Reach the goal.
- Never contain a bad state.
- Use available transitions.
- Produce a valid state and transition path.

---

8. Author

Student Name: __________________________

Roll Number: __________________________

Course: PCCST503 – Machine Learning
