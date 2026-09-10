SREE LEKSHMI H
TCR24CS067
# Safe Semantic Planner

A C++-based path planning system that finds a safe and low-cost path from an initial state to a goal state while avoiding unsafe states.

## Overview

The Safe Semantic Planner models a planning problem using:

- States
- Transitions
- Transition costs
- Safety values
- Reliability values
- Transition availability
- Bad/unsafe states

The planner searches for a valid path from the initial state to the goal while avoiding designated bad states and minimizing the total transition cost.

The system also reports experimental planning metrics such as:

- Total path cost
- Minimum safety distance
- Number of explored states
- Planning time
- Number of bad states visited
- Estimated planner memory usage

## Features

- Safe path planning
- Bad-state avoidance
- Cost-based path selection
- Transition availability checking
- Safety-distance calculation
- Path and transition reconstruction
- Planning performance metrics
- Command-line input
- Written in C++

## Project Structure

```text
SafeSemanticPlanner/
│
├── main.cpp
├── LPAStar.cpp
├── LPAStar.h
├── Safety.cpp
├── Safety.h
├── State.h
├── Transition.h
├── PlanningProblem.h
├── PlanningResult.h
└── README.md
````

## Technologies Used

* C++
* C++11/C++17
* MinGW / GCC
* MSYS2 UCRT64

## How It Works

The planner follows this general workflow:

```text
User Input
    ↓
Planning Problem
    ↓
State and Transition Representation
    ↓
Safety / Bad-State Checking
    ↓
Path Planning
    ↓
Path Reconstruction
    ↓
Result + Performance Metrics
```

## Input

The user provides:

1. Number of states
2. State IDs and coordinates
3. Initial state
4. Goal state
5. Bad states
6. Number of transitions
7. Transition information

Each transition contains:

```text
ID FROM TO COST SAFETY RELIABILITY AVAILABLE
```

Example:

```text
1 1 2 2 0.9 0.95 1
```

where:

* `1` = Transition ID
* `1` = Source state
* `2` = Destination state
* `2` = Transition cost
* `0.9` = Safety value
* `0.95` = Reliability
* `1` = Transition available

## Output

The planner reports:

```text
Success
State Path
Transition Path
Total Cost
Minimum Safety Distance
Explored States
Planning Time
Bad States Visited
Estimated Memory Usage
```

Example:

```text
========== PLANNING RESULT ==========

Success: YES
State Path: 1 -> 4
Transition Path: 4
Total Cost: 10.00
Minimum Safety Distance: 1.41
Explored States: 8
Planning Time: 0.03 ms
Bad States Visited: 0
Estimated Memory Usage: 0.19 KB
```

## Compilation

Using the MSYS2 UCRT64 terminal:

```bash
cd /c/SafeSemanticPlanner
```

Compile the project:

```bash
g++ -std=c++17 main.cpp LPAStar.cpp Safety.cpp -o planner.exe
```

## Running the Planner

Run:

```bash
./planner.exe
```

The program will interactively ask for the planning problem.

## Example

For a simple problem:

```text
Initial State: 1
Goal State: 4
Bad State: 3
```

The planner can select:

```text
1 -> 2 -> 4
```

instead of a path that passes through the unsafe state.

## Testing

The planner was tested using multiple test cases covering:

* Basic path finding
* Avoidance of bad states
* Safety-distance calculation
* Transition availability
* Goal changes
* Transition changes

The tested cases successfully produced valid paths and correct cost/safety results.

## Performance Metrics

The planner records several experimental metrics.

### Total Cost

The sum of the costs of all transitions in the selected path.

### Minimum Safety Distance

The minimum calculated distance between the selected path and designated unsafe states.

### Bad States Visited

The number of unsafe states appearing in the final selected path.

A safe solution should have:

```text
Bad States Visited: 0
```

### Explored States

An experimental measure of the amount of planning work performed during path computation.

### Planning Time

The time required to compute the path, measured in milliseconds.

### Estimated Memory Usage

An estimate of memory used by the planner's primary data structures.

## Future Improvements

The current project provides a command-line interface. Future versions can include:

* Web-based frontend
* Interactive visualization of states and paths
* Graph visualization
* Real-time replanning
* More advanced LPA* priority-queue implementation
* Comparison with other path-planning algorithms
* Larger benchmark datasets

