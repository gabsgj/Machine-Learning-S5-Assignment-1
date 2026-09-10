#include <iostream>

#include "State.h"
#include "Transition.h"
#include "PlanningProblem.h"
#include "PlanningResult.h"
#include "LPAStarPlanner.h"


int main()
{
    std::cout << "Safe Semantic Planner\n";
    std::cout << "=====================\n\n";


    // -------------------------------------------------
    // 1. Create states
    // -------------------------------------------------

    State start(
        1,
        {0.0, 0.0}
    );

    State safeA(
        2,
        {2.0, 1.0}
    );

    State badState(
        3,
        {2.0, -1.0}
    );

    State goal(
        4,
        {4.0, 0.0}
    );


    // -------------------------------------------------
    // 2. Create transitions
    // -------------------------------------------------

    Transition startToA(
        1,
        1,
        2,
        3.0,
        0.95,
        0.98
    );

    Transition AToGoal(
        2,
        2,
        4,
        4.0,
        0.90,
        0.95
    );

    Transition startToBad(
        3,
        1,
        3,
        1.0,
        0.20,
        0.80
    );

    Transition badToGoal(
        4,
        3,
        4,
        1.0,
        0.10,
        0.70
    );


    // -------------------------------------------------
    // 3. Create planning problem
    // -------------------------------------------------

    PlanningProblem problem;

    problem.initialState = 1;

    problem.goalState = 4;

    problem.states = {
        start,
        safeA,
        badState,
        goal
    };

    problem.transitions = {
        startToA,
        AToGoal,
        startToBad,
        badToGoal
    };

    // State 3 is unsafe.
    problem.badStates = {
        3
    };


    // -------------------------------------------------
    // 4. Run LPA* planner
    // -------------------------------------------------

    LPAStarPlanner planner;

    PlanningResult result =
        planner.plan(problem);


    // -------------------------------------------------
    // 5. Display result
    // -------------------------------------------------

    std::cout << "Planning Result\n";
    std::cout << "---------------\n";

    if (!result.success)
    {
        std::cout << "No safe path found.\n";
        return 0;
    }

    std::cout << "Safe path found!\n\n";

    std::cout << "State Path: ";

    for (size_t i = 0;
         i < result.statePath.size();
         ++i)
    {
        std::cout
            << result.statePath[i];

        if (i + 1 <
            result.statePath.size())
        {
            std::cout << " -> ";
        }
    }

    std::cout << "\n";

    std::cout
        << "Total Cost: "
        << result.totalCost
        << "\n";

    std::cout
        << "Minimum Safety: "
        << result.safetyScore
        << "\n";

    std::cout
        << "Transitions Used: ";

    for (auto id :
         result.transitionPath)
    {
        std::cout << id << " ";
    }

    std::cout << "\n";

    return 0;
}