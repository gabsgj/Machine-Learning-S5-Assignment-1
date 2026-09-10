#include "DStarLitePlanner.h"

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>
#include <algorithm>

using namespace std;


// ==================================================
// Print result
// ==================================================

void printResult(
    const string& testName,
    const PlanningResult& result
) {

    cout << "\n========================================\n";
    cout << testName << "\n";
    cout << "========================================\n";

    cout << "Success: "
         << (result.success ? "YES" : "NO")
         << "\n";

    cout << "State Path: ";

    if (result.statePath.empty()) {

        cout << "NONE";

    } else {

        for (size_t i = 0;
             i < result.statePath.size();
             ++i) {

            cout << result.statePath[i];

            if (i + 1 <
                result.statePath.size()) {

                cout << " -> ";
            }
        }
    }

    cout << "\n";

    cout << "Transition Path: ";

    if (result.transitionPath.empty()) {

        cout << "NONE";

    } else {

        for (size_t i = 0;
             i < result.transitionPath.size();
             ++i) {

            cout << result.transitionPath[i];

            if (i + 1 <
                result.transitionPath.size()) {

                cout << " -> ";
            }
        }
    }

    cout << "\n";

    cout << fixed
         << setprecision(3);

    cout << "Total Path Cost: "
         << result.totalCost
         << "\n";

    cout << "Minimum Safety Distance: "
         << result.minimumSafetyDistance
         << "\n";

    cout << "Safety Score: "
         << result.safetyScore
         << "\n";

    cout << "Explored States: "
         << result.exploredStates
         << "\n";

    cout << "Planning Time: "
         << result.planningTimeMs
         << " ms\n";

    cout << "Replanning Time: "
         << result.replanningTimeMs
         << " ms\n";

    cout << "Approx. Memory Usage: "
         << result.memoryUsageBytes
         << " bytes\n";
}


// ==================================================
// Check bad states in path
// ==================================================

size_t countBadStatesVisited(
    const PlanningProblem& problem,
    const PlanningResult& result
) {

    size_t count = 0;

    for (uint64_t state :
         result.statePath) {

        if (problem.isBadState(state))
            ++count;
    }

    return count;
}


// ==================================================
// Test Case 1
// ==================================================

PlanningProblem testCase1() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        State(0, {0.0, 0.0}),
        State(1, {1.0, 0.0}),
        State(2, {2.0, 0.0}),
        State(3, {3.0, 0.0})
    };

    problem.transitions = {

        Transition(0, 0, 1, 1.0, 0.95, 0.98),
        Transition(1, 1, 2, 1.0, 0.95, 0.98),
        Transition(2, 2, 3, 1.0, 0.95, 0.98)
    };

    return problem;
}


// ==================================================
// Test Case 2
// ==================================================

PlanningProblem testCase2() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {

        State(0, {0.0, 0.0}),   // S
        State(1, {1.0, 0.0}),   // A
        State(2, {2.0, 0.0}),   // X
        State(3, {0.0, 1.0}),   // C
        State(4, {1.0, 1.0}),   // D
        State(5, {2.0, 1.0})    // G
    };

    problem.badStates = {
        2
    };

    problem.transitions = {

        Transition(0, 0, 1, 1.0, 0.90, 0.95),
        Transition(1, 1, 2, 0.5, 0.20, 0.80),
        Transition(2, 2, 5, 0.5, 0.20, 0.80),

        Transition(3, 0, 3, 1.2, 0.95, 0.98),
        Transition(4, 3, 4, 1.0, 0.95, 0.98),
        Transition(5, 4, 5, 1.0, 0.95, 0.98)
    };

    return problem;
}


// ==================================================
// Test Case 3
// ==================================================

PlanningProblem testCase3() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {

        State(0, {0.0, 0.0}),   // S

        State(1, {1.0, 0.1}),   // A - close to bad

        State(2, {2.0, 0.1}),   // B - close to bad

        State(3, {0.0, 3.0}),   // C - safe

        State(4, {1.5, 3.0}),   // D - safe

        State(5, {3.0, 3.0}),   // G

        State(6, {1.5, 0.0})    // BAD
    };

    problem.badStates = {
        6
    };

    /*
       Path 1:
       S -> A -> B -> G

       Lower transition cost but close
       to the bad state.

       Path 2:
       S -> C -> D -> G

       Higher cost but much safer.
    */

    problem.transitions = {

        Transition(0, 0, 1, 0.7, 0.90, 0.95),
        Transition(1, 1, 2, 0.7, 0.90, 0.95),
        Transition(2, 2, 5, 0.7, 0.90, 0.95),

        Transition(3, 0, 3, 2.0, 0.98, 0.99),
        Transition(4, 3, 4, 2.0, 0.98, 0.99),
        Transition(5, 4, 5, 2.0, 0.98, 0.99)
    };

    return problem;
}


// ==================================================
// Test Case 4
// ==================================================

PlanningProblem testCase4() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        State(0, {0.0, 0.0}),
        State(1, {1.0, 0.0}),
        State(2, {1.0, 1.0}),
        State(3, {2.0, 0.0})
    };

    problem.transitions = {

        // Initial preferred route
        Transition(0, 0, 1, 1.0, 0.95, 0.98),
        Transition(1, 1, 3, 1.0, 0.95, 0.98),

        // Alternative route
        Transition(2, 0, 2, 1.5, 0.95, 0.98),
        Transition(3, 2, 3, 1.5, 0.95, 0.98)
    };

    return problem;
}


// ==================================================
// Test Case 5
// ==================================================

PlanningProblem testCase5() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 4;

    problem.states = {

        State(0, {0.0, 0.0}),
        State(1, {1.0, 0.0}),
        State(2, {0.0, 1.0}),
        State(3, {1.0, 1.0}),
        State(4, {2.0, 1.0}),
        State(5, {2.0, 0.0})
    };

    problem.transitions = {

        Transition(0, 0, 1, 1.0, 0.95, 0.98),
        Transition(1, 1, 5, 1.0, 0.95, 0.98),
        Transition(2, 5, 4, 1.0, 0.95, 0.98),

        Transition(3, 0, 2, 1.0, 0.95, 0.98),
        Transition(4, 2, 3, 1.0, 0.95, 0.98),
        Transition(5, 3, 4, 1.0, 0.95, 0.98)
    };

    return problem;
}


// ==================================================
// Test Case 6
// ==================================================

PlanningProblem testCase6() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 4;

    problem.states = {

        State(0, {0.0, 0.0}),
        State(1, {1.0, 0.0}),
        State(2, {2.0, 0.0}),
        State(3, {1.0, 1.0}),
        State(4, {3.0, 0.0})
    };

    problem.transitions = {

        Transition(0, 0, 1, 1.0, 0.95, 0.98),
        Transition(1, 1, 2, 1.0, 0.95, 0.98),
        Transition(2, 2, 4, 1.0, 0.95, 0.98),

        Transition(3, 0, 3, 2.0, 0.95, 0.98),
        Transition(4, 3, 4, 2.0, 0.95, 0.98)
    };

    return problem;
}


// ==================================================
// CSV helper
// ==================================================

void writeCSVHeader(
    ofstream& file
) {

    file
        << "TestCase,"
        << "Success,"
        << "BadStatesVisited,"
        << "TotalPathCost,"
        << "MinimumSafetyDistance,"
        << "SafetyScore,"
        << "ExploredStates,"
        << "PlanningTimeMs,"
        << "ReplanningTimeMs,"
        << "MemoryUsageBytes\n";
}


void writeCSVRow(
    ofstream& file,
    int testCase,
    const PlanningProblem& problem,
    const PlanningResult& result
) {

    file
        << testCase << ","
        << (result.success ? 1 : 0) << ","
        << countBadStatesVisited(
               problem,
               result
           ) << ","
        << result.totalCost << ","
        << result.minimumSafetyDistance << ","
        << result.safetyScore << ","
        << result.exploredStates << ","
        << result.planningTimeMs << ","
        << result.replanningTimeMs << ","
        << result.memoryUsageBytes
        << "\n";
}


// ==================================================
// Main
// ==================================================

int main() {

    cout << "============================================\n";
    cout << " SAFE SEMANTIC PLANNER\n";
    cout << " PCCST503 - Machine Learning\n";
    cout << " Assignment 1\n";
    cout << " D* Lite Implementation\n";
    cout << "============================================\n";


    ofstream csv(
        "results/experimental_results.csv"
    );

    if (!csv.is_open()) {

        cerr
            << "Could not open results/experimental_results.csv\n";

        return 1;
    }

    csv << fixed
        << setprecision(4);

    writeCSVHeader(csv);


    // ------------------------------------------
    // Test Case 1
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase1();

        DStarLitePlanner planner;

        PlanningResult result =
            planner.plan(problem);

        printResult(
            "TEST CASE 1 - Basic Reachability",
            result
        );

        writeCSVRow(
            csv,
            1,
            problem,
            result
        );
    }


    // ------------------------------------------
    // Test Case 2
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase2();

        DStarLitePlanner planner;

        PlanningResult result =
            planner.plan(problem);

        printResult(
            "TEST CASE 2 - Bad State Avoidance",
            result
        );

        writeCSVRow(
            csv,
            2,
            problem,
            result
        );
    }


    // ------------------------------------------
    // Test Case 3
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase3();

        DStarLitePlanner planner;

        PlanningResult result =
            planner.plan(problem);

        printResult(
            "TEST CASE 3 - Safety Margin",
            result
        );

        writeCSVRow(
            csv,
            3,
            problem,
            result
        );
    }


    // ------------------------------------------
    // Test Case 4
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase4();

        DStarLitePlanner planner;

        PlanningResult initialResult =
            planner.plan(problem);

        printResult(
            "TEST CASE 4 - Initial Dynamic Path",
            initialResult
        );

        /*
           Transition 1:
           A -> G

           becomes unavailable.
        */

        planner.updateTransition(
            1,
            false
        );

        auto replanningStart =
            chrono::high_resolution_clock::now();

        PlanningResult replannedResult =
            planner.replan();

        auto replanningEnd =
            chrono::high_resolution_clock::now();

        replannedResult.replanningTimeMs =
            chrono::duration<
                double,
                milli
            >(
                replanningEnd -
                replanningStart
            ).count();

        printResult(
            "TEST CASE 4 - After Transition Removal",
            replannedResult
        );

        writeCSVRow(
            csv,
            4,
            problem,
            replannedResult
        );
    }


    // ------------------------------------------
    // Test Case 5
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase5();

        DStarLitePlanner planner;

        PlanningResult initialResult =
            planner.plan(problem);

        printResult(
            "TEST CASE 5 - Initial Goal",
            initialResult
        );

        /*
           Change goal from state 4
           to state 5.
        */

        planner.updateGoal(5);

        auto replanningStart =
            chrono::high_resolution_clock::now();

        PlanningResult replannedResult =
            planner.replan();

        auto replanningEnd =
            chrono::high_resolution_clock::now();

        replannedResult.replanningTimeMs =
            chrono::duration<
                double,
                milli
            >(
                replanningEnd -
                replanningStart
            ).count();

        printResult(
            "TEST CASE 5 - Goal Update",
            replannedResult
        );

        writeCSVRow(
            csv,
            5,
            problem,
            replannedResult
        );
    }


    // ------------------------------------------
    // Test Case 6
    // ------------------------------------------

    {
        PlanningProblem problem =
            testCase6();

        DStarLitePlanner planner;

        PlanningResult initialResult =
            planner.plan(problem);

        printResult(
            "TEST CASE 6 - Before Shortcut",
            initialResult
        );

        /*
           Add a new shortcut:
           S -> G
        */

        planner.addTransition(
            Transition(
                5,
                0,
                4,
                0.5,
                0.98,
                0.99
            )
        );

        auto replanningStart =
            chrono::high_resolution_clock::now();

        PlanningResult improvedResult =
            planner.replan();

        auto replanningEnd =
            chrono::high_resolution_clock::now();

        improvedResult.replanningTimeMs =
            chrono::duration<
                double,
                milli
            >(
                replanningEnd -
                replanningStart
            ).count();

        printResult(
            "TEST CASE 6 - After Transition Addition",
            improvedResult
        );

        writeCSVRow(
            csv,
            6,
            problem,
            improvedResult
        );
    }


    csv.close();

    cout << "\n============================================\n";
    cout << "Experimental results saved to:\n";
    cout << "results/experimental_results.csv\n";
    cout << "============================================\n";

    return 0;
}