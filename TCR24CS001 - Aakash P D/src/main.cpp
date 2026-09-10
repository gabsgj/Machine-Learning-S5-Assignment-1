#include "DStarLitePlanner.h"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace std;


State makeState(
    uint64_t id,
    double x,
    double y
) {

    return State(
        id,
        {x, y}
    );
}


void printResult(
    const string& testName,
    const PlanningResult& result
) {

    cout << "\n========================================\n";
    cout << testName << "\n";
    cout << "========================================\n";

    cout << "Status: "
         << (result.success ? "SUCCESS" : "FAILURE")
         << "\n";


    if (!result.success) {

        cout << "No valid path found.\n";

        cout << "Explored states: "
             << result.exploredStates
             << "\n";

        cout << "Planning time: "
             << fixed
             << setprecision(3)
             << result.planningTimeMs
             << " ms\n";

        return;
    }


    cout << "State path: ";

    for (size_t i = 0;
         i < result.statePath.size();
         ++i) {

        cout << result.statePath[i];

        if (i + 1 <
            result.statePath.size()) {

            cout << " -> ";
        }
    }

    cout << "\n";


    cout << "Transition path: ";

    for (size_t i = 0;
         i < result.transitionPath.size();
         ++i) {

        cout << result.transitionPath[i];

        if (i + 1 <
            result.transitionPath.size()) {

            cout << " -> ";
        }
    }

    cout << "\n";


    cout << "Total cost: "
         << fixed
         << setprecision(3)
         << result.totalCost
         << "\n";


    cout << "Minimum safety distance: "
         << fixed
         << setprecision(3)
         << result.safetyScore
         << "\n";


    cout << "Cumulative reliability: "
         << fixed
         << setprecision(3)
         << result.cumulativeReliability
         << "\n";


    cout << "Explored states: "
         << result.exploredStates
         << "\n";


    cout << "Planning time: "
         << fixed
         << setprecision(3)
         << result.planningTimeMs
         << " ms\n";
}


/*
==================================================
TEST 1
Basic Reachability
S -> A -> B -> G
==================================================
*/

void testBasicReachability() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 4;


    problem.states = {

        makeState(1, 0, 0),
        makeState(2, 1, 0),
        makeState(3, 2, 0),
        makeState(4, 3, 0)
    };


    problem.transitions = {

        Transition(1, 1, 2, 1.0, 1.0, 0.95),
        Transition(2, 2, 3, 1.0, 1.0, 0.95),
        Transition(3, 3, 4, 1.0, 1.0, 0.95)
    };


    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(problem);


    printResult(
        "TEST 1: Basic Reachability",
        result
    );
}


/*
==================================================
TEST 2
Bad State Avoidance

S -> A -> X -> G   X = bad
S -> C -> D -> G
==================================================
*/

void testBadStateAvoidance() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 7;


    problem.states = {

        makeState(1, 0, 0),
        makeState(2, 1, 1),
        makeState(3, 2, 1),
        makeState(4, 3, 1),

        makeState(5, 1, -1),
        makeState(6, 2, -1),
        makeState(7, 3, 0)
    };


    problem.badStates = {
        3
    };


    problem.transitions = {

        // Bad path
        Transition(1, 1, 2, 1.0, 1.0, 0.95),
        Transition(2, 2, 3, 1.0, 1.0, 0.95),
        Transition(3, 3, 7, 1.0, 1.0, 0.95),

        // Safe path
        Transition(4, 1, 5, 1.0, 1.0, 0.95),
        Transition(5, 5, 6, 1.0, 1.0, 0.95),
        Transition(6, 6, 7, 1.0, 1.0, 0.95)
    };


    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(problem);


    printResult(
        "TEST 2: Bad State Avoidance",
        result
    );
}


/*
==================================================
TEST 3
Safety Margin

Path 1:
S -> A -> G

Path 2:
S -> C -> D -> G

Path 2 stays farther from the bad state.
==================================================
*/

void testSafetyMargin() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 6;


    problem.states = {

        makeState(1, 0, 0),

        makeState(2, 1, 0.2),
        makeState(3, 2, 0.2),

        makeState(4, 1, 3.0),
        makeState(5, 2, 3.0),

        makeState(6, 3, 1.5),

        makeState(7, 1.5, 0.0)
    };


    problem.badStates = {
        7
    };


    problem.transitions = {

        // Shorter but less safe route
        Transition(
            1, 1, 2,
            1.0,
            0.2,
            0.95
        ),

        Transition(
            2, 2, 6,
            1.0,
            0.2,
            0.95
        ),


        // Longer but safer route
        Transition(
            3, 1, 4,
            2.0,
            3.0,
            0.98
        ),

        Transition(
            4, 4, 5,
            2.0,
            3.0,
            0.98
        ),

        Transition(
            5, 5, 6,
            2.0,
            3.0,
            0.98
        )
    };


    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(problem);


    printResult(
        "TEST 3: Safety Margin",
        result
    );
}


/*
==================================================
TEST 4
Dynamic Transition

Initially:

S -> A -> G

Then A -> G becomes unavailable.

Alternative:

S -> B -> G
==================================================
*/

void testDynamicTransition() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 4;


    problem.states = {

        makeState(1, 0, 0),
        makeState(2, 1, 0),
        makeState(3, 1, 2),
        makeState(4, 2, 1)
    };


    problem.transitions = {

        Transition(
            1, 1, 2,
            1.0,
            1.0,
            0.95
        ),

        Transition(
            2, 2, 4,
            1.0,
            1.0,
            0.95
        ),

        Transition(
            3, 1, 3,
            2.0,
            2.0,
            0.95
        ),

        Transition(
            4, 3, 4,
            2.0,
            2.0,
            0.95
        )
    };


    DStarLitePlanner planner;


    PlanningResult initial =
        planner.plan(problem);


    printResult(
        "TEST 4A: Dynamic Transition - Initial",
        initial
    );


    // A -> G becomes unavailable.
    problem.transitions[1].available =
        false;


    PlanningResult updated =
        planner.replan(problem);


    printResult(
        "TEST 4B: Dynamic Transition - After Update",
        updated
    );
}


/*
==================================================
TEST 5
Goal Update
==================================================
*/

void testGoalUpdate() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 4;


    problem.states = {

        makeState(1, 0, 0),
        makeState(2, 1, 0),
        makeState(3, 2, 0),
        makeState(4, 3, 0),
        makeState(5, 2, 2)
    };


    problem.transitions = {

        Transition(
            1, 1, 2,
            1.0,
            1.0,
            0.95
        ),

        Transition(
            2, 2, 3,
            1.0,
            1.0,
            0.95
        ),

        Transition(
            3, 3, 4,
            1.0,
            1.0,
            0.95
        ),

        Transition(
            4, 3, 5,
            1.5,
            1.0,
            0.95
        )
    };


    DStarLitePlanner planner;


    PlanningResult result1 =
        planner.plan(problem);


    printResult(
        "TEST 5A: Original Goal",
        result1
    );


    // Change goal.
    problem.goalState = 5;


    planner.updateGoal(5);


    PlanningResult result2 =
        planner.replan(problem);


    printResult(
        "TEST 5B: Updated Goal",
        result2
    );
}


/*
==================================================
TEST 6
Transition Addition

Initially:

S -> A -> G

Then a cheaper shortcut:

S -> G

is inserted.
==================================================
*/

void testTransitionAddition() {

    PlanningProblem problem;

    problem.initialState = 1;
    problem.goalState = 3;


    problem.states = {

        makeState(1, 0, 0),
        makeState(2, 1, 0),
        makeState(3, 2, 0)
    };


    problem.transitions = {

        Transition(
            1, 1, 2,
            3.0,
            1.0,
            0.95
        ),

        Transition(
            2, 2, 3,
            3.0,
            1.0,
            0.95
        )
    };


    DStarLitePlanner planner;


    PlanningResult result1 =
        planner.plan(problem);


    printResult(
        "TEST 6A: Before Shortcut",
        result1
    );


    // Add shortcut.
    problem.transitions.push_back(
        Transition(
            3, 1, 3,
            1.0,
            1.0,
            0.99
        )
    );


    PlanningResult result2 =
        planner.replan(problem);


    printResult(
        "TEST 6B: After Shortcut Added",
        result2
    );
}


int main() {

    cout << "\n";
    cout << "============================================\n";
    cout << "     SAFE PATH PLANNER - D* LITE PROJECT\n";
    cout << "============================================\n";


    testBasicReachability();

    testBadStateAvoidance();

    testSafetyMargin();

    testDynamicTransition();

    testGoalUpdate();

    testTransitionAddition();


    cout << "\n============================================\n";
    cout << "             ALL TESTS COMPLETED\n";
    cout << "============================================\n";


    return 0;
}