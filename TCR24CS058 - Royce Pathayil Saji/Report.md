# Design Report: Safe Semantic Planner

## 1. Introduction
This document contains the design report, experimental results, and user manual for the Safe Semantic Planner implementation.

## 2. Design Report

### State Representation
Each state is represented by a unique integer identifier and a vector of double precision floating point numbers indicating its embedding in a Cartesian space. 

### Data Structures
The planner uses several data structures to manage the state space effectively. Unordered maps store the relationships between states and their corresponding vectors. Unordered sets maintain the collection of bad states for constant time lookup. A priority queue manages the open list for the search algorithm, ensuring the node with the lowest cost is expanded first.

### Heuristic Function
The heuristic function utilizes the Euclidean distance between the current state embedding and the goal state embedding. This provides an admissible and consistent heuristic for the Cartesian state space.

### Safety Computation
Safety is computed by finding the minimum Euclidean distance from a given state to any defined bad state. If a state is in the bad states list, its safety distance is zero. Otherwise, it iterates through all bad states to find the closest one. The path cost is penalized inversely proportional to this safety distance.

### Time Complexity
The time complexity is O(E log V) where E is the number of transitions and V is the number of states. This is dominated by the priority queue operations in the search algorithm.

### Space Complexity
The space complexity is O(V + E) to store the graph structures, transition mappings, and the open list during planning.

## 3. Experimental Results
The planner was tested against six distinct scenarios.
* Test 1 (Basic Reachability): The planner successfully found the shortest path from start to goal.
* Test 2 (Bad State Avoidance): The planner successfully rerouted around a bad state, opting for a slightly longer but completely safe path.
* Test 3 (Safety Margin): The planner successfully evaluated multiple valid paths and chose the one balancing total cost and distance from bad states.
* Test 4 (Dynamic Transition): When a critical transition was removed, the planner correctly detected the failure to reach the goal.
* Test 5 (Goal Update): The planner successfully reached the newly updated goal state.
* Test 6 (Transition Addition): When a new shortcut was added, the planner successfully discovered and utilized it to reduce the total cost.

## 4. User Manual

### Compilation
To compile the planner, run the following command in your terminal:
`g++ -std=c++17 main.cpp planner.cpp -o planner_test`

### Execution
Run the compiled executable:
`./planner_test`

### Usage
The main.cpp file contains the test cases. You can modify the PlanningProblem structures in main.cpp to add new states, transitions, or bad states. The setGamma function on the SafePlanner object allows you to adjust the weight of the safety penalty.
