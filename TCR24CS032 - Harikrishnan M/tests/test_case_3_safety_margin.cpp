#include "planner.hpp"
#include <iostream>

int main()
{
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {4};

    // Bad state is at (1.0, 0.5).
    // Path 1 (Close/Cheap): 0 -> 1 (1.0, 0.0) -> 3. Cost 2. Dist to bad = 0.5
    // Path 2 (Far/Expensive): 0 -> 2 (1.0, -2.0) -> 3. Cost 3. Dist to bad = 2.5
    p.states = {
        {0, {0.0, 0.0}},
        {1, {1.0, 0.0}},
        {2, {1.0, -2.0}},
        {3, {2.0, 0.0}},
        {4, {1.0, 0.5}}};

    p.transitions = {
        {1, 0, 1, 1.0, 1.0, 1.0, true},
        {2, 1, 3, 1.0, 1.0, 1.0, true},
        {3, 0, 2, 1.5, 1.0, 1.0, true},
        {4, 2, 3, 1.5, 1.0, 1.0, true}};

    LPAPlanner planner;
    PlanningResult res = planner.plan(p);

    // The planner should prefer node 2 over node 1 due to the safety penalty
    bool took_safe_path = false;
    for (auto s : res.statePath)
    {
        if (s == 2)
            took_safe_path = true;
    }

    std::cout << "Test 3 (Safety Margin): " << (took_safe_path ? "PASS" : "FAIL") << "\n";
    return 0;
}