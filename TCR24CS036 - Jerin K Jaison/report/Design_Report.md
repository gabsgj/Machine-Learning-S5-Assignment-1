# PCCST503 – Machine Learning

# Assignment 1

# Design of a Safe Semantic Planner in a Finite Cartesian State Space

1. Introduction

This project implements a Safe Semantic Planner for a finite Cartesian state space using the D* Lite planning algorithm. The planner finds a path from an initial state to a goal state while avoiding bad states and considering transition cost, safety, reliability, and dynamic changes in the environment.

2. Problem Definition

The planner operates on a finite set of states represented using Cartesian embeddings. Directed transitions connect states. Each transition has a cost, safety score, reliability score, and availability status.

The planner receives an initial state, a goal state, a set of bad states, a set of states, and a set of transitions. The objective is to reach the goal without visiting bad states while maintaining a good balance between cost and safety.

3. Objectives

- Reach the goal state successfully.
- Avoid all bad states.
- Minimize total transition cost.
- Maximize the minimum distance from bad states.
- Consider transition safety and reliability.
- Support dynamic changes in the planning environment.
- Measure planning and replanning performance.

4. State Representation

Each state is represented by a unique ID and a Cartesian embedding vector.

uint64_t id;
std::vector<double> embedding;
The embedding is used for Euclidean distance calculations and the heuristic function.

5. Transition Representation

Each transition contains:

-Transition ID
-Source state
-Destination state
-Cost
-Safety
-Reliability
-Availability

Only available and safe transitions are considered during planning.

6. Data Structures

The implementation uses:

-vector for states and transitions.
-unordered_map for g and rhs values.
-Adjacency lists for incoming and outgoing transitions.
-Priority queue for the D* Lite OPEN list.
-unordered_set for visited-state tracking.

7. D* Lite Algorithm

D* Lite is used for path planning and replanning.

    For each state, the algorithm maintains:
    
    g(s)
    rhs(s)

The goal is initialized using:

    rhs(goal) = 0

Other states initially have infinite values.

The algorithm processes states using a priority queue until the start state becomes locally consistent.

For a state u:

    rhs(u) = min(cost(u,v) + g(v))

where v represents a valid successor state.

8. Heuristic Function

The planner uses Euclidean distance between Cartesian state embeddings.

For d-dimensional states:

    h(A,B) = sqrt(Σ(Ai - Bi)²)

The heuristic estimates the remaining distance between the current state and the goal.

9. Safety Model

Bad states are explicitly stored in the planning problem.

A state that belongs to the bad-state set cannot be included in a valid solution.

For every visited state, the planner calculates its Euclidean distance to the nearest bad state:

    D(s) = min distance(s, bad state)

The minimum distance along the selected path is reported as the minimum safety distance.

10. Cost Function

The effective transition cost considers:

-Transition cost
-Safety
-Reliability
-Distance from bad states

The general form is:

Effective Cost =
Transition Cost
+ Safety Penalty
+ Reliability Penalty
+ Safety Distance Penalty

Lower safety and reliability increase the effective cost. States closer to bad states receive an additional penalty.

11. Dynamic Replanning

The planner supports dynamic changes including:

    Goal update
    planner.updateGoal(newGoal);
    Transition availability update
    planner.updateTransition(
        transitionId,
        false
    );
    Bad-state update
    planner.updateBadStates(
        newBadStates
    );
    Transition addition
    planner.addTransition(
        transition
    );
    Transition removal
    planner.removeTransition(
        transitionId
    );

After a change, replan() calculates a new path.

12. Test Cases
-Test Case 1 – Basic Reachability
S -> A -> B -> G

The planner should successfully reach the goal.

-Test Case 2 – Bad State Avoidance
S -> A -> X -> G

where X is a bad state.

An alternative path is:

S -> C -> D -> G

The planner must avoid X.

-Test Case 3 – Safety Margin

Two valid paths are provided. One path has lower cost but passes close to a bad state. The other has higher cost but maintains a larger safety margin.

The planner considers the safety-distance penalty when selecting the path.

-Test Case 4 – Dynamic Transition

A transition used by the initial solution becomes unavailable. The planner must find an alternative path.

-Test Case 5 – Dynamic Goal

The goal state is changed during execution. The planner calculates a new path to the updated goal.

-Test Case 6 – Dynamic Transition Addition

A new shortcut transition is added. The planner replans and evaluates the new route.

13. Evaluation Metrics

The following metrics are recorded:

-Success
-Bad states visited
-Total path cost
-Minimum safety distance
-Safety score
-Explored states
-Planning time
-Replanning time
-Approximate memory usage

The results are stored in:

    results/experimental_results.csv
14. Complexity Analysis

Let:

V = number of states
E = number of transitions

The priority-queue-based graph search has approximately:

O((V + E) log V)

time complexity.

The main graph and planner data structures require approximately:

O(V + E)

space.

15. Project Structure
```text
safe-semantic-planner/
├── include/
├── src/
├── tests/
├── data/
├── results/
└── report/
