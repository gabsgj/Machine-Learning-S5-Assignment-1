
# Safe Semantic Planner Using D* Lite

## PCCST503 – Machine Learning

### Assignment 1

**Design of a Safe Semantic Planner in a Finite Cartesian State Space**

---

## 1. Project Overview

This project implements a safe semantic planner for a finite Cartesian state space using the D* Lite path planning algorithm.

The planner receives:

- An initial state
- A goal state
- A set of bad states
- A set of directed transitions

Each state is represented using a Cartesian embedding.

Each transition contains:

- Transition cost
- Safety score
- Reliability
- Availability status

The planner computes a valid path from the initial state to the goal while avoiding all predefined bad states.

---

## 2. Objectives

The main objectives of the planner are:

1. Reach the goal state.
2. Never visit a bad state.
3. Minimize total transition cost.
4. Maximize the minimum distance from visited states to bad states.
5. Provide reasonable planning performance.
6. Handle changes in the planning environment.

---

## 3. Algorithm

The project uses the D* Lite path planning algorithm.

The implementation uses:

- `g` values
- `rhs` values
- Priority queue
- Euclidean heuristic
- Vertex updates
- Shortest-path computation

D* Lite is particularly useful for environments where the graph can change after a path has already been computed.

---

## 4. State Representation

Each state contains:

```text
State ID
Cartesian embedding
Example:

S = (0, 0)
A = (2, 2)
B = (4, 3)
C = (2, -2)
G = (6, 0)
X = (4, 1)

State X is treated as a bad state in the test environment.

5. Transition Representation

Each directed transition contains:

Transition ID
Source state
Destination state
Cost
Safety score
Reliability
Availability

For example:

S → A
Cost = 5
Safety = 0.9
Reliability = 0.95
Available = Yes
6. Safety

Bad states are treated as forbidden states.

The planner does not select transitions that lead to a bad state.

The minimum safety distance of a path is calculated using Euclidean distance.

For a visited state s and bad states B:

D(s,B) = minimum Euclidean distance from s to any bad state

For an entire path:

D(P) = minimum D(s,B) for all visited states

A larger value indicates a larger minimum geometric separation from bad states.

7. Dynamic Environment

The implementation demonstrates planning under changing conditions.

The following changes are tested:

A transition becoming unavailable
Goal state changing
A new transition being added

After these changes, the planner computes a revised path.

8. Test Cases
Test Case 1 – Basic Reachability

Tests whether the planner can reach the goal in a connected graph.

Test Case 2 – Bad State Avoidance

Tests whether the planner avoids the dangerous state X.

Test Case 3 – Safety Margin

Tests paths with different costs and safety characteristics.

Test Case 4 – Dynamic Transition

Tests replanning when a transition becomes unavailable.

Test Case 5 – Goal Update

Tests planning after the goal state changes.

Test Case 6 – Transition Addition

Tests whether the planner can discover a newly inserted shortcut.

9. Experimental Evaluation

The planner records the following measurements:

Goal success
Number of bad states visited
Total path cost
Minimum safety distance
Cumulative reliability
Number of explored states
Planning time

The results are stored in:

results/results.csv
10. Project Structure
safe-semantic-planner/
│
├── src/
│   └── main.cpp
│
├── results/
│   └── results.csv
│
├── report/
│   └── Design_Report.pdf
│
├── README.md
│
└── User_Manual.md
11. Requirements
Windows 10/11
Visual Studio Code
GNU G++ compiler
C++17 compatible compiler
12. Compilation

Open the VS Code terminal in the project directory and run:

g++ -std=c++17 src/main.cpp -o planner
13. Execution

Run:

.\planner.exe

The program automatically executes all six test cases.

14. Results

The program prints the planning results to the terminal and creates:

results/results.csv

The CSV file can be opened using Microsoft Excel or another spreadsheet application.

15. Documentation

Detailed information about the implementation, algorithm, complexity, experiments and conclusions is provided in:

report/Design_Report.pdf

Instructions for compiling and running the project are provided in:

User_Manual.md
16. Conclusion

The Safe Semantic Planner demonstrates graph-based path planning using D* Lite concepts in a finite Cartesian state space.

The implementation considers transition cost, safety, reliability and availability while avoiding predefined bad states.

The project also demonstrates how the planner can respond to changes in the planning environment.


---

# 7.4 Save

Press:

```text
Ctrl + S