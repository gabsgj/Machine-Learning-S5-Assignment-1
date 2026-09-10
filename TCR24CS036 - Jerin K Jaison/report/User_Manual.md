# User Manual

1. Requirements

- C++17 compatible compiler
- Git
- GitHub

2. Project Structure

```text
include/   Header files
src/       Source files
tests/     Test program
data/      Test data
results/   Experimental results
report/    Documentation
```
3. Compile the Planner

Open a terminal in the project root:

    g++ -std=c++17 -Iinclude src/*.cpp -o planner.exe
4. Run the Planner

Windows:

    .\planner.exe

Linux/macOS:

    ./planner.exe
5. Compile the Tests
   
g++ -std=c++17 -Iinclude src/State.cpp src/Transition.cpp src/PlanningProblem.cpp src/PlanningResult.cpp src/DStarLitePlanner.cpp tests/test_cases.cpp -o tests.exe

7. Run the Tests

Windows:

    .\tests.exe

Linux/macOS:

    ./tests.exe
7. Test Cases

The program includes:

Basic Reachability
Bad State Avoidance
Safety Margin
Dynamic Transition
Dynamic Goal
Dynamic Transition Addition

8. Experimental Results

Experimental results are automatically saved in:

    results/experimental_results.csv
9. Modifying States

States can be created using:

State(id, {x, y});

Example:

State(10, {4.0, 2.0});
10. Modifying Transitions

Transitions can be created using:

    Transition(
        id,
        from,
        to,
        cost,
        safety,
        reliability
    );

Example:

    Transition(
        10,
        2,
        5,
        2.0,
        0.95,
        0.98
    );
11. Defining Bad States

Bad states are specified using:

problem.badStates = {
    stateID1,
    stateID2
};

The planner will not use a bad state in a valid path.

12. Dynamic Updates
    
Change Goal
```text
planner.updateGoal(newGoal);
planner.replan();
Disable Transition
planner.updateTransition(
    transitionID,
    false
);
planner.replan();
Update Bad States
planner.updateBadStates(
    newBadStates
);
planner.replan();
Add Transition
planner.addTransition(
    Transition(
        id,
        from,
        to,
        cost,
        safety,
        reliability
    )
);

planner.replan();
Remove Transition
planner.removeTransition(
    transitionID
);

planner.replan();
```
13. Output

The planner displays:

Success/failure
State path
Transition path
Total path cost
Minimum safety distance
Safety score
Explored states
Planning time
Replanning time
Approximate memory usage
