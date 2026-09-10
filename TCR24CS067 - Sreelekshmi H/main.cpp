#include <iostream>
#include <iomanip>
#include "LPAStar.h"

using namespace std;

void printResult(const PlanningResult& result) {

    cout << "\n========== PLANNING RESULT ==========\n";

    cout << "Success: "
         << (result.success ? "YES" : "NO")
         << endl;

    if (!result.success) {
        cout << "No safe path found.\n";
        return;
    }

    cout << "State Path: ";

    for (size_t i = 0; i < result.statePath.size(); i++) {
        cout << result.statePath[i];

        if (i + 1 < result.statePath.size())
            cout << " -> ";
    }

    cout << endl;

    cout << "Transition Path: ";

    for (size_t i = 0;
         i < result.transitionPath.size();
         i++) {

        cout << result.transitionPath[i];

        if (i + 1 < result.transitionPath.size())
            cout << " -> ";
    }

    cout << endl;

    cout << fixed << setprecision(2);

    cout << "Total Cost: "
         << result.totalCost << endl;

    cout << "Minimum Safety Distance: "
         << result.safetyScore << endl;
    cout << "Explored States: "
     << result.exploredStates << endl;

    cout << "Planning Time: "
     << result.planningTimeMs
     << " ms" << endl;
    cout << "Bad States Visited: "
     << result.badStatesVisited << endl;
    cout << "Estimated Memory Usage: "
    << result.memoryUsageKB
     << " KB" << endl;
}



int main() {

    cout << "=====================================\n";
    cout << "       SAFE SEMANTIC PLANNER\n";
    cout << "=====================================\n";

    PlanningProblem problem;

    int numberOfStates;

    cout << "\nEnter number of states: ";
    cin >> numberOfStates;

    // -------------------------------
    // INPUT STATES
    // -------------------------------

    cout << "\nEnter state ID and coordinates:\n";
    cout << "Example: 1 0 0\n\n";

    for (int i = 0; i < numberOfStates; i++) {

        uint64_t id;
        double x, y;

        cout << "State " << i + 1 << ": ";
        cin >> id >> x >> y;

        problem.states.push_back(
            State(id, {x, y})
        );
    }

    // -------------------------------
    // INITIAL AND GOAL
    // -------------------------------

    cout << "\nEnter initial state ID: ";
    cin >> problem.initialState;

    cout << "Enter goal state ID: ";
    cin >> problem.goalState;

    // -------------------------------
    // BAD STATES
    // -------------------------------

    int numberOfBadStates;

    cout << "\nEnter number of bad states: ";
    cin >> numberOfBadStates;

    if (numberOfBadStates > 0) {

        cout << "Enter bad state IDs:\n";

        for (int i = 0; i < numberOfBadStates; i++) {

            uint64_t badID;

            cin >> badID;

            problem.badStates.push_back(badID);
        }
    }

    // -------------------------------
    // TRANSITIONS
    // -------------------------------

    int numberOfTransitions;

    cout << "\nEnter number of transitions: ";
    cin >> numberOfTransitions;

    cout << "\nFor each transition enter:\n";
    cout << "ID FROM TO COST SAFETY RELIABILITY AVAILABLE\n";
    cout << "Example: 1 1 2 2 0.9 0.95 1\n\n";

    for (int i = 0; i < numberOfTransitions; i++) {

        uint64_t id;
        uint64_t from;
        uint64_t to;

        double cost;
        double safety;
        double reliability;

        int available;

        cout << "Transition " << i + 1 << ": ";

        cin >> id
            >> from
            >> to
            >> cost
            >> safety
            >> reliability
            >> available;

        problem.transitions.push_back(
            Transition(
                id,
                from,
                to,
                cost,
                safety,
                reliability,
                available == 1
            )
        );
    }

    // -------------------------------
    // DISPLAY INPUT
    // -------------------------------

    cout << "\n========== INPUT PROBLEM ==========\n";

    cout << "Initial State: "
         << problem.initialState << endl;

    cout << "Goal State: "
         << problem.goalState << endl;

    cout << "Bad States: ";

    if (problem.badStates.empty()) {
        cout << "None";
    }
    else {
        for (size_t i = 0;
             i < problem.badStates.size();
             i++) {

            cout << problem.badStates[i];

            if (i + 1 < problem.badStates.size())
                cout << ", ";
        }
    }

    cout << endl;

    // -------------------------------
    // RUN PLANNER
    // -------------------------------

    LPAStar planner;

    PlanningResult result =
        planner.plan(problem);

    // -------------------------------
    // DISPLAY RESULT
    // -------------------------------

    printResult(result);

    cout << "\n=====================================\n";

    return 0;
}