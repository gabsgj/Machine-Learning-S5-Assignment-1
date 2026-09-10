# Safe Semantic Planner
**Name:** Amaya Raveendran

**University Register Number:** TCR24CS010
## PCCST503 - Machine Learning
### Assignment 1


Design of a Safe Semantic Planner in a Finite Cartesian State Space

## Objective

The objective of this project is to design and implement a safe planner that finds a path from an initial state to a goal state while avoiding bad states.

The planner considers:

- Transition cost
- Safety
- Reliability
- Transition availability
- Euclidean distance between states
- Dynamic changes in the environment

## State Representation

Each state is represented by:

- State ID
- Cartesian embedding vector

Example:

```text
State 0 = [0, 0]
State 1 = [1, 1]
State 2 = [2, 2]
