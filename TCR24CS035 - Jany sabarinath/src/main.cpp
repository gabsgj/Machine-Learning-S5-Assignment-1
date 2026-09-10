#include "DStarLite.h"
#include "../tests/TestCases.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

void printLine() {
    std::cout
        << "============================================\n";
}

void printPath(
    const PlanningResult& result
) {

    if (!result.success) {

        std::cout
            << "No valid path found.\n";

        return;
    }

    std::cout << "Path: ";

    for (size_t i = 0;
         i < result.statePath.size();
         i++) {

        std::cout
            << result.statePath[i];

        if (
            i + 1 <
            result.statePath.size()
        ) {
            std::cout << " -> ";
        }
    }

    std::cout << "\n";
}

void printResult(
    const PlanningResult& result
) {

    printPath(result);

    std::cout
        << std::fixed
        << std::setprecision(4);

    std::cout
        << "Success           : "
        << (result.success ? "YES" : "NO")
        << "\n";

    std::cout
        << "Total Cost        : "
        << result.totalCost
        << "\n";

    std::cout
        << "Minimum Safety    : "
        << result.safetyScore
        << "\n";

    std::cout
        << "Reliability       : "
        << result.reliability
        << "\n";

    std::cout
        << "Explored States   : "
        << result.exploredStates
        << "\n";

    std::cout
        << "Planning Time     : "
        << result.planningTimeMs
        << " ms\n";

    if (result.replanningTimeMs > 0) {

        std::cout
            << "Replanning Time   : "
            << result.replanningTimeMs
            << " ms\n";
    }
}

void testBasicReachability() {

    printLine();

    std::cout
        << "TEST CASE 1: BASIC REACHABILITY\n";

    printLine();

    PlanningProblem problem =
        TestCases::basicReachability();

    DStarLite planner(1.0);

    PlanningResult result =
        planner.plan(problem);

    printResult(result);
}

void testBadStateAvoidance() {

    printLine();

    std::cout
        << "TEST CASE 2: BAD STATE AVOIDANCE\n";

    printLine();

    PlanningProblem problem =
        TestCases::badStateAvoidance();

    DStarLite planner(2.0);

    PlanningResult result =
        planner.plan(problem);

    printResult(result);
}

void testSafetyMargin() {

    printLine();

    std::cout
        << "TEST CASE 3: SAFETY MARGIN\n";

    printLine();

    PlanningProblem problem =
        TestCases::safetyMargin();

    std::cout
        << "\nLow safety weight:\n";

    DStarLite plannerLow(0.1);

    PlanningResult resultLow =
        plannerLow.plan(problem);

    printResult(resultLow);

    std::cout
        << "\nHigh safety weight:\n";

    DStarLite plannerHigh(10.0);

    PlanningResult resultHigh =
        plannerHigh.plan(problem);

    printResult(resultHigh);
}

void testDynamicTransition() {

    printLine();

    std::cout
        << "TEST CASE 4: DYNAMIC TRANSITION\n";

    printLine();

    PlanningProblem problem =
        TestCases::dynamicTransition();

    DStarLite planner(1.0);

    std::cout
        << "\nInitial planning:\n";

    PlanningResult initial =
        planner.plan(problem);

    printResult(initial);

    std::cout
        << "\nTransition 1 becomes unavailable...\n";

    planner.updateTransition(
        1,
        false
    );

    PlanningResult replanned =
        planner.replan();

    printResult(replanned);
}

void testGoalUpdate() {

    printLine();

    std::cout
        << "TEST CASE 5: GOAL UPDATE\n";

    printLine();

    PlanningProblem problem =
        TestCases::goalUpdate();

    DStarLite planner(1.0);

    std::cout
        << "\nInitial goal:\n";

    PlanningResult initial =
        planner.plan(problem);

    printResult(initial);

    std::cout
        << "\nGoal changed to state 4...\n";

    planner.updateGoal(4);

    PlanningResult updated =
        planner.replan();

    printResult(updated);
}

void testTransitionAddition() {

    printLine();

    std::cout
        << "TEST CASE 6: TRANSITION ADDITION\n";

    printLine();

    PlanningProblem problem =
        TestCases::transitionAddition();

    DStarLite planner(1.0);

    std::cout
        << "\nBefore shortcut:\n";

    PlanningResult initial =
        planner.plan(problem);

    printResult(initial);

    std::cout
        << "\nAdding shortcut S -> G...\n";

    Transition shortcut(
        10,
        0,
        3,
        2.0,
        0.95,
        0.99,
        true
    );

    planner.addTransition(
        shortcut
    );

    PlanningResult updated =
        planner.replan();

    printResult(updated);
}

void saveSimpleCSV() {

    std::ofstream file(
        "../results/results.csv"
    );

    if (!file.is_open())
        return;

    file
        << "test_case,success,total_cost,"
        << "minimum_safety,explored_states,"
        << "planning_time_ms\n";

    file
        << "Generated during execution\n";

    file.close();
}

int main() {

    printLine();

    std::cout
        << "SAFE SEMANTIC PLANNER\n";

    std::cout
        << "D* Lite Dynamic Path Planning\n";

    printLine();

    testBasicReachability();

    testBadStateAvoidance();

    testSafetyMargin();

    testDynamicTransition();

    testGoalUpdate();

    testTransitionAddition();

    printLine();

    std::cout
        << "ALL TEST CASES COMPLETED\n";

    printLine();

    return 0;
}