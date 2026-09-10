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
        {2, {2.0, 0.0}},
        {3, {1.0, 1.0}}};

    p.transitions = {
        {1, 0, 1, 1.0, 1.0, 1.0, true},
        {2, 1, 2, 1.0, 1.0, 1.0, true}, // Primary path
        {3, 0, 3, 2.0, 1.0, 1.0, true},
        {4, 3, 2, 2.0, 1.0, 1.0, true} // Backup path
    };

    LPAPlanner planner;
    planner.plan(p);

    // Transition 1->2 becomes unavailable
    planner.env.removeTransition(1, 2);
    PlanningResult res = planner.replan();

    // It should now take the backup path (cost > 3.0)
    std::cout << "Test 4 (Dynamic Disconnect): " << (res.success && res.totalCost > 3.0 ? "PASS" : "FAIL") << "\n";
    return 0;
}