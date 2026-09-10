#include "planner.hpp"
#include <iostream>

int main()
{
    PlanningProblem problem;
    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {
        {0, {0.0, 0.0}}, {1, {1.0, 0.0}}, {2, {2.0, 0.0}}, {3, {3.0, 0.0}}}; // S -> A -> B -> G

    problem.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true}};

    LPAPlanner planner;
    PlanningResult result = planner.plan(problem);

    std::cout << "Test 1 Success: " << (result.success ? "PASS" : "FAIL") << "\n";
    return 0;
}