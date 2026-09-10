#include "DStarLitePlanner.h"

#include <iostream>
#include <vector>

bool checkPath(
    const PlanningResult& result,
    const std::vector<uint64_t>& expected
) {

    if (!result.success)
        return false;

    return result.statePath == expected;
}

bool checkNoBadStates(
    const PlanningProblem& problem,
    const PlanningResult& result
) {

    for (uint64_t state :
         result.statePath) {

        if (problem.isBadState(state))
            return false;
    }

    return true;
}

PlanningProblem basicReachability() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {
        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {2, 0}),
        State(3, {3, 0})
    };

    problem.transitions = {
        Transition(0, 0, 1, 1, 0.9, 0.95),
        Transition(1, 1, 2, 1, 0.9, 0.95),
        Transition(2, 2, 3, 1, 0.9, 0.95)
    };

    return problem;
}

PlanningProblem badStateAvoidance() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {
        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {2, 0}),
        State(3, {0, 1}),
        State(4, {1, 1}),
        State(5, {2, 1})
    };

    problem.badStates = {2};

    problem.transitions = {
        Transition(0, 0, 1, 1, 0.9, 0.95),
        Transition(1, 1, 2, 1, 0.2, 0.8),
        Transition(2, 2, 5, 1, 0.2, 0.8),

        Transition(3, 0, 3, 1.2, 0.95, 0.98),
        Transition(4, 3, 4, 1, 0.95, 0.98),
        Transition(5, 4, 5, 1, 0.95, 0.98)
    };

    return problem;
}

int main() {

    int passed = 0;
    int total = 2;

    {
        DStarLitePlanner planner;

        PlanningProblem problem =
            basicReachability();

        PlanningResult result =
            planner.plan(problem);

        bool passedTest =
            checkPath(
                result,
                {0, 1, 2, 3}
            );

        std::cout
            << "Test 1 - Basic Reachability: "
            << (passedTest ? "PASS" : "FAIL")
            << "\n";

        if (passedTest)
            ++passed;
    }

    {
        DStarLitePlanner planner;

        PlanningProblem problem =
            badStateAvoidance();

        PlanningResult result =
            planner.plan(problem);

        bool passedTest =
            result.success &&
            checkNoBadStates(
                problem,
                result
            ) &&
            !result.statePath.empty() &&
            result.statePath.back() == 5;

        std::cout
            << "Test 2 - Bad State Avoidance: "
            << (passedTest ? "PASS" : "FAIL")
            << "\n";

        if (passedTest)
            ++passed;
    }

    std::cout
        << "\nPassed: "
        << passed
        << "/"
        << total
        << "\n";

    return passed == total ? 0 : 1;
}