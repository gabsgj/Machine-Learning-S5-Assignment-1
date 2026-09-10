# Semantic Planner User Manual

## Build and run

From the repository root in PowerShell:

```powershell
g++ -std=c++17 -Wall -Wextra -pedantic -I semantic_planner/src semantic_planner/src/main.cpp semantic_planner/src/LPAStarPlanner.cpp -O2 -o semantic_planner/semantic_planner.exe
.\semantic_planner\semantic_planner.exe
```

CMake can also be used:

```powershell
cmake -S semantic_planner -B semantic_planner/build
cmake --build semantic_planner/build --config Release
```

## Using the planner

Create a `PlanningProblem`, add `State` values and directed `Transition` values, then call:

```cpp
LPAStarPlanner planner(1.0, 1.0, 1.0, 1.0);
PlanningResult result = planner.plan(problem);
```

The constructor weights are cost, safety distance, heuristic, and reliability penalty. Bad states are always forbidden. Unavailable transitions are ignored.

`PlanningResult` contains the state path, transition path, raw total cost, minimum distance to a bad state, cumulative reliability, explored-state count, planning time, replanning time, estimated memory usage, bad-state visit count, and whether the cached graph was reused.

## Dynamic updates

Keep the same planner instance and modify the problem before calling `plan()` again:

- Change `goalState` to replan to another goal.
- Edit `badStates` to change forbidden states.
- Set `Transition::available` to enable or disable an edge.
- Add or remove transitions.
- Change transition cost, safety, or reliability.

The transition graph is cached between calls. Goal and bad-state changes reuse the graph; transition changes invalidate and rebuild it. LPA\* then computes a safe route for the updated problem.

## Safety and reliability

A route cannot contain a bad state. Among valid routes, the planner combines transition cost, distance from bad states, and reliability into a non-negative LPA\* edge cost. Increase the safety weight to prefer larger margins; increase the reliability weight to penalize unreliable transitions.

## Test cases

Running the executable evaluates the six assignment cases: reachability, bad-state avoidance, safety margin, transition removal, goal update, and transition addition. Each case prints its metrics and a PASS/FAIL result.
