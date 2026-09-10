
# Safe Semantic Planner

## PCCST503 – Machine Learning

A Safe Semantic Planner implemented for a finite Cartesian state space using the D* Lite algorithm.

Features

- Cartesian state representation
- Directed transitions
- Cost-aware planning
- Safety-aware planning
- Reliability-aware planning
- Bad-state avoidance
- Euclidean heuristic
- Safety-distance calculation
- Dynamic goal updates
- Dynamic bad-state updates
- Transition addition and removal
- Dynamic replanning
- Performance evaluation

Project Structure

```text
safe-semantic-planner/
├── include/
│   ├── State.h
│   ├── Transition.h
│   ├── PlanningProblem.h
│   ├── PlanningResult.h
│   └── DStarLitePlanner.h
│
├── src/
│   ├── State.cpp
│   ├── Transition.cpp
│   ├── PlanningProblem.cpp
│   ├── PlanningResult.cpp
│   ├── DStarLitePlanner.cpp
│   └── main.cpp
│
├── tests/
├── data/
├── results/
└── report/
```
Algorithm

  The planner uses D* Lite and considers:
```text  
  Transition cost
  Safety
  Reliability
  Distance from bad states
```
Test Cases
```text
  Basic Reachability
  Bad State Avoidance
  Safety Margin
  Dynamic Transition
  Dynamic Goal
  Dynamic Transition Addition
```
Build
```text
  g++ -std=c++17 -Iinclude src/*.cpp -o planner.
```
Run

  Windows:

      .\planner.exe

Run Tests

      g++ -std=c++17 -Iinclude src/State.cpp src/Transition.cpp src/PlanningProblem.cpp src/PlanningResult.cpp src/DStarLitePlanner.cpp tests/test_cases.cpp -o           tests.exe
      .\tests.exe

Results
  Experimental measurements are saved to:
```  
  results/experimental_results.csv
```
Documentation
```text
  report/Design_Report.md – Technical design and algorithm
  report/User_Manual.md – Installation, execution and usage instructions
```
