# Safe Semantic Planner

## PCCST503 - Machine Learning
### Assignment 1

Design and Implementation of a Safe Semantic Planner in a Finite Cartesian State Space.

## Objective

The objective of this project is to design and implement a safe path planner operating on a finite Cartesian state space.

The planner:

- Finds a path from an initial state to a goal state.
- Avoids predefined bad states.
- Minimizes transition cost.
- Maximizes safety distance from bad states.
- Considers transition reliability.
- Supports dynamic changes.
- Supports replanning.

## Algorithm

D* Lite

## State Representation

Each state contains:

- Unique state ID
- Cartesian embedding

Example:

```text
State S = (0, 0)
State A = (1, 0)
State G = (2, 0)