# Safe Semantic Planner Using LPA*

## PCCST503 - Machine Learning

### Assignment 1

Design and Implementation of a Safe Semantic Planner in a Finite Cartesian State Space.

---

## 1. Overview

This project implements a safe path planner using the Lifelong Planning A* (LPA*) algorithm.

The planner operates on a finite directed graph whose states are represented in a Cartesian space.

Each transition contains:

- Transition ID
- Source state
- Destination state
- Cost
- Safety score
- Reliability
- Availability

The planner avoids bad states and searches for a safe path from an initial state to a goal state.

---

## 2. Features

The implementation supports:

- Cartesian state representation
- Directed graph transitions
- Bad-state avoidance
- Transition cost optimisation
- Safety-aware planning
- Reliability-aware planning
- Euclidean heuristic
- Dynamic transition availability
- Goal updates
- Transition addition
- Replanning
- Experimental performance measurement

---

## 3. Algorithm

The project uses:

Lifelong Planning A* (LPA*)

LPA* maintains two values for each state:

- g(s)
- rhs(s)

and uses a priority queue called OPEN.

The heuristic used is Euclidean distance between the current state and the goal state.

---

## 4. Safety

A state is considered unsafe if it belongs to the set of bad states.

Bad states are never expanded or included in the final path.

The planner also calculates the Euclidean distance from each visited state to the nearest bad state.

The minimum of these distances is reported as:

Minimum Safety Distance.

---

## 5. Reliability

Transition reliability is represented as a value between 0 and 1.

The planner introduces a small penalty for transitions with lower reliability.

---

## 6. Dynamic Environment

The planner supports:

- Enabling/disabling transitions
- Adding transitions
- Removing transitions
- Changing the goal
- Adding bad states
- Removing bad states

After an environmental change, the planner can calculate a new path.

---

## 7. Test Cases

### Test Case 1

Basic reachability:

S -> A -> B -> G

### Test Case 2

Bad-state avoidance:

S -> A -> X -> G

where X is a bad state.

The planner selects the safe alternative.

### Test Case 3

Safety margin.

Two valid paths exist. The planner balances path cost with distance from bad states.

### Test Case 4

Dynamic transition.

Initially:

S -> A -> G

Then A -> G becomes unavailable.

The planner finds an alternative path.

### Test Case 5

Goal update.

The goal changes during execution and the planner produces a new route.

### Test Case 6

Transition addition.

A new shortcut is inserted and the planner discovers the improved route.

---

## 8. Project Structure

```text
safe-semantic-planner/
│
├── include/
│   ├── model.h
│   └── planner.h
│
├── src/
│   ├── planner.cpp
│   └── main.cpp
│
├── results/
│   └── experimental_results.csv
│
├── docs/
│   └── Algorithm.txt
│
├── report/
│   └── SafeSemanticPlanner_Report.txt
│
├── README.md
└── Makefile