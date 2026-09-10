# User Manual

## 1. Install prerequisites

Install:

- Git
- CMake 3.16 or newer
- a C++17 compiler

## 2. Build

From the repository root:

```bash
cmake -S . -B build
cmake --build build -j
```

## 3. Run

```bash
./build/safe_planner
```

The program prints six demonstration scenarios.

## 4. Run automated tests

```bash
ctest --test-dir build --output-on-failure
```

## 5. Use the planner in your own program

Create a `PlanningProblem`:

```cpp
PlanningProblem p;
p.initialState = 0;
p.goalState = 5;
p.badStates = {3};
p.states = {
    {0, {0.0, 0.0}},
    {1, {1.0, 0.0}},
    {5, {2.0, 0.0}}
};
p.transitions = {
    {0, 0, 1, 1.0, 1.0, 0.95, true},
    {1, 1, 5, 1.0, 1.0, 0.95, true}
};

DStarLitePlanner planner(2.0, 1.0);
PlanningResult result = planner.plan(p);
```

Check:

```cpp
if (result.success) {
    // result.statePath
    // result.transitionPath
    // result.totalCost
    // result.safetyScore
}
```

## 6. Dynamic changes

Disable a transition:

```cpp
planner.setTransitionAvailable(1, false);
auto result = planner.replan(0);
```

Add a transition:

```cpp
planner.addTransition({99, 0, 5, 0.5, 1.0, 0.99, true});
auto result = planner.replan(0);
```

Change the goal:

```cpp
planner.setGoal(4);
auto result = planner.replan(0);
```

Change bad states:

```cpp
planner.setBadStates({2, 3});
auto result = planner.replan(0);
```

## 7. Assignment demonstration

Use the six scenarios in `src/main.cpp`. Explain each output during the demonstration:

- why the chosen path is valid;
- why bad states are rejected;
- how safety affects the route;
- what happens after a transition is removed;
- what happens after the goal changes;
- why a newly added shortcut is selected.

## 8. Submission

Push the repository to GitHub and submit the repository URL. Make sure the repository contains:

- source code;
- report;
- experimental results;
- user manual;
- test code;
- README.
