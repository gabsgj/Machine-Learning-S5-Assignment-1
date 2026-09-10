# User Manual: Safe Semantic Planner

## Requirements
- A modern C++ compiler (supporting C++17 or newer) such as `g++` or `clang++`.
- (Optional) CMake 3.14 or newer.

## Building the Project

### Option 1: Using g++ directly
Navigate to the root directory of the project and run the following command to compile the planner and its test cases:

```bash
g++ -std=c++17 -I include src/DStarLite.cpp tests/test_planner.cpp -o planner_tests.exe
```

### Option 2: Using CMake
Create a build directory, generate the configuration, and compile:

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

## Running the Tests
To execute the predefined test cases (Test Cases 1 through 6), run the compiled executable:

```bash
# On Windows
.\planner_tests.exe

# On Unix/Linux
./planner_tests
```

The output will demonstrate the planner solving basic reachability, avoiding bad states, preferring safer paths over cheaper but hazardous ones, and efficiently replanning when the environment dynamically changes.

## Integrating the Planner into Your Code

### 1. Include Headers
Include the relevant structures and the planner interface:
```cpp
#include "Types.h"
#include "DStarLite.h"
```

### 2. Define the Problem
Populate a `PlanningProblem` object with your states, transitions, initial state, goal state, and any bad states.
```cpp
PlanningProblem problem;
problem.initialState = 0;
problem.goalState = 3;
problem.badStates = {2}; 

// Add states with their embeddings
problem.states = {
    {0, {0.0, 0.0}}, 
    {1, {1.0, 1.0}}
    // ...
};

// Add transitions: {id, from, to, cost, safety, reliability, available}
problem.transitions = {
    {0, 0, 1, 1.0, 1.0, 1.0, true}
    // ...
};
```

### 3. Initialize and Plan
Instantiate the `DStarLite` planner and invoke `plan()`.
```cpp
// You can pass custom weights to the constructor: (costWeight, safetyWeight, reliabilityWeight)
DStarLite planner(1.0, 2.0, 0.5); 
PlanningResult result = planner.plan(problem);

if (result.success) {
    // Process result.statePath and result.totalCost
}
```

### 4. Dynamic Replanning
To update the environment, use the provided dynamic methods, then invoke `replan(current_state)` without having to rebuild all internal data structures:
```cpp
// A transition is blocked
planner.updateTransition(1, 2, false);

// A new shortcut appears
planner.addTransition({99, 0, 3, 1.5, 1.0, 1.0, true});

// The goal state changes
planner.updateGoal(5);

// Re-calculate the path starting from current state
PlanningResult updatedResult = planner.replan(current_state);
```
