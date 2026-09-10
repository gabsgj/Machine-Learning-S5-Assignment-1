# Safe Semantic Planner using D* Lite

## Overview

This project is my implementation for the ML PCCST503 assignment on safe semantic planning.

The planner is implemented in C++ and uses the **D* Lite** algorithm to find a path from an initial state to a goal state. The planning problem is represented as a directed graph, where each state has a Cartesian position and each transition has properties such as cost, reliability, safety, and availability.

The planner is designed to avoid bad states while finding a suitable path to the goal. It also supports replanning when the planning problem changes.

## Main Features

The implementation includes:

* State representation using Cartesian coordinates
* Directed transitions between states
* Transition costs
* Transition reliability
* Transition availability
* Bad-state avoidance
* Safety-distance calculation
* Euclidean heuristic
* D* Lite path planning
* Dynamic transition removal
* Dynamic transition addition
* Goal updates
* Replanning after changes

## Project Structure

```text
safe-semantic-planner/
│
├── README.md
├── CMakeLists.txt
├── LICENSE
│
├── data/
│   └── demo_scenarios.txt
│
├── docs/
│   ├── DESIGN_REPORT.md
│   ├── EXPERIMENTAL_RESULTS.md
│   └── USER_MANUAL.md
│
├── include/
│   └── planner.hpp
│
├── src/
│   ├── main.cpp
│   └── planner.cpp
│
└── tests/
    └── test_planner.cpp
```

### Source Files

* `include/planner.hpp` contains the main data structures and planner interfaces.
* `src/planner.cpp` contains the implementation of the planner and D* Lite algorithm.
* `src/main.cpp` contains the demonstration program and the six test cases.
* `tests/test_planner.cpp` contains the automated tests.

### Documentation

* `docs/DESIGN_REPORT.md` explains the design and implementation.
* `docs/EXPERIMENTAL_RESULTS.md` contains the results from running the test cases.
* `docs/USER_MANUAL.md` contains instructions for building and running the project.

## Requirements

The project requires:

* C++17 compatible compiler
* CMake
* Linux, Windows, or macOS

For Linux, the project can be built using GCC and CMake.

## Building the Project

Open a terminal in the project directory.

Create the build directory and configure the project:

```bash
cmake -S . -B build
```

Build the project:

```bash
cmake --build build
```

## Running the Program

After building, run:

```bash
./build/safe_planner
```

The program runs the six demonstration cases and prints the planning results to the terminal.

The output includes information such as:

* Whether a path was found
* State path
* Total path cost
* Minimum safety distance
* Cumulative reliability
* Number of explored states
* Planning time
* Replanning time for dynamic cases

## Test Cases

The demonstration program contains six cases.

### 1. Basic Reachability

Checks whether a path can be found from the initial state to the goal.

### 2. Bad State Avoidance

Checks whether the planner avoids a state marked as bad and uses another available path.

### 3. Safety Margin

Checks the safety distance of the selected path with respect to bad states.

### 4. Dynamic Transition Removal

A transition is made unavailable after planning, and the planner generates an updated path.

### 5. Goal Update

The goal state is changed and the planner generates a path to the new goal.

### 6. Transition Addition

A new transition is added to the graph and the planner can use the newly available route.

## Running the Tests

The project also includes automated tests.

After building the project, run:

```bash
ctest --test-dir build --output-on-failure
```

A successful run should report that the tests passed.

## Experimental Results

The results from the six demonstration cases are recorded in:

```text
docs/EXPERIMENTAL_RESULTS.md
```

The results include the path selected by the planner, total cost, safety distance, reliability, explored states, planning time, and replanning time where applicable.

The experimental values were obtained by running the program on Linux.

## How D* Lite is Used

D* Lite is used as the main path-planning algorithm.

The planner maintains information about the current planning problem and uses this information to calculate a path to the goal. When part of the problem changes, such as a transition becoming unavailable or the goal changing, the planner can update the planning information and calculate a new path.

This is useful for the dynamic cases included in the assignment.

## Safety and Cost

The planner considers more than just the distance between states.

Transitions have a cost and reliability value, while states can be marked as bad. The planner also calculates the distance between the selected path and bad states.

This allows the test cases to show the difference between a direct route and an alternative route that avoids a bad state.

## Example

A simple path produced by the planner can look like:

```text
0 -> 1 -> 2 -> 5
```

If a transition on this route becomes unavailable, the planner can select another route, for example:

```text
0 -> 1 -> 6 -> 5
```

The exact paths and measurements for the demonstration cases can be found in `docs/EXPERIMENTAL_RESULTS.md`.


## Bonus / Additional Feature

The implementation also supports **incremental/dynamic replanning** for changes to the planning problem.

The demonstration includes:

* Removing an existing transition and finding a new route
* Adding a new transition and using the new route
* Updating the goal and finding a revised route

These changes are demonstrated in Test Cases 4, 5, and 6.

The assignment also mentions other optional extensions such as multi-goal planning, time-dependent transition availability, parallel search, learning-based heuristics, and knowledge-graph testing. These additional extensions are not implemented in this version.

## Documentation

More details about the project are available in:

```text
docs/DESIGN_REPORT.md
docs/EXPERIMENTAL_RESULTS.md
docs/USER_MANUAL.md
```

## Repository

GitHub repository:

https://github.com/NRJ9595/safe-semantic-planner
