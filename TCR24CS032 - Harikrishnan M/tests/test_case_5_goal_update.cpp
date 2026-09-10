#include "planner.hpp"
#include <iostream>

int main()
{
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 1;

    p.states = {
        {0, {0.0, 0.0}},
        {1, {1.0, 0.0}},
        {2, {2.0, 0.0}}};

    p.transitions = {
        {1, 0, 1, 1.0, 1.0, 1.0, true},
        {2, 1, 2, 1.0, 1.0, 1.0, true}};

    LPAPlanner planner;
    planner.plan(p);

    // Update goal from 1 to 2
    planner.env.updateGoal(2);
    PlanningResult res = planner.replan();

    std::cout << "Test 5 (Goal Update): " << (res.success && res.statePath.back() == 2 ? "PASS" : "FAIL") << "\n";
    return 0;
}