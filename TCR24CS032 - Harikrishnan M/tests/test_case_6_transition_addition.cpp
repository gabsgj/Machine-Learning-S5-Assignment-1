#include "planner.hpp"
#include <iostream>

int main()
{
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 2;

    p.states = {
        {0, {0.0, 0.0}},
        {1, {1.0, 0.0}},
        {2, {2.0, 0.0}}};

    p.transitions = {
        {1, 0, 1, 1.0, 1.0, 1.0, true},
        {2, 1, 2, 1.0, 1.0, 1.0, true}};

    LPAPlanner planner;
    PlanningResult old_res = planner.plan(p);

    // Add shortcut 0 -> 2 directly with a lower cost
    planner.env.addTransition({3, 0, 2, 0.5, 1.0, 1.0, true});
    PlanningResult res = planner.replan();

    std::cout << "Test 6 (Add Shortcut): " << (res.totalCost < old_res.totalCost ? "PASS" : "FAIL") << "\n";
    return 0;
}