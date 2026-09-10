# User Manual

## 1. Software requirements

- Linux, macOS, or Windows with a C++17 compiler
- `g++` recommended
- CMake is optional

## 2. Run directly with g++

From the repository root:

```bash
g++ -std=c++17 -O2 -Wall -Wextra -pedantic src/main.cpp -o planner
./planner
```

The program prints each test case, selected state path, transition path, cost, safety distance, reliability, number of explored states, planning/replanning time, and approximate memory usage.

## 3. Run using CMake

```bash
mkdir build
cd build
cmake ..
cmake --build .
./safe_planner
```

## 4. Understanding the output

- **Success**: whether a valid safe path reached the current goal.
- **State path**: ordered sequence of state IDs.
- **Transition path**: ordered sequence of transition IDs.
- **Total path cost**: sum of original transition costs, not the safety/reliability penalty.
- **Minimum safety distance**: minimum Euclidean distance from any visited state to the nearest bad state. `N/A` means no bad states were defined.
- **Cumulative reliability**: sum of reliability values of selected transitions.
- **Bad states visited**: should always be zero for a successful safe plan.
- **Explored states**: number of D* Lite state expansions.
- **Planning time**: initial planning time in milliseconds.
- **Replanning time**: time required by a dynamic update.
- **Approx. memory**: rough memory occupied by the main planner structures; it is not a full process-memory measurement.

## 5. Modifying test cases

The six illustrative test cases are defined near the bottom of `src/main.cpp`. A state is created with:

```cpp
S(id, x, y)
```

A transition is created with:

```cpp
T(id, from, to, cost, safety, reliability, available)
```

Add bad states through `problem.badStates`.

## 6. Dynamic replanning examples

Transition availability:

```cpp
planner.replanAfterTransitionChange(transitionId, false);
```

Goal update:

```cpp
planner.replanAfterGoalChange(newGoalId);
```

Transition addition:

```cpp
planner.replanAfterTransitionAddition(newTransition);
```


```
