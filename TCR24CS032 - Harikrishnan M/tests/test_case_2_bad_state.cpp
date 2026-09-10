#include "planner.hpp"
#include <iostream>

int main()
{
    PlanningProblem problem;
    problem.initialState = 0;
    problem.goalState = 4;
    problem.badStates = {2}; // State X is bad

    // S(0) -> A(1) -> X(2) -> G(4)
    // S(0) -> C(3) -> G(4)
    problem.states = {
        {0, {0.0, 0.0}}, {1, {1.0, 1.0}}, {2, {2.0, 1.0}}, {3, {1.0, -1.0}}, {4, {3.0, 0.0}}};

    problem.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true}, // S -> A
        {1, 1, 2, 1.0, 0.0, 1.0, true}, // A -> X (Bad)
        {2, 2, 4, 1.0, 1.0, 1.0, true}, // X -> G
        {3, 0, 3, 1.5, 1.0, 1.0, true}, // S -> C
        {4, 3, 4, 1.5, 1.0, 1.0, true}  // C -> G
    };

    LPAPlanner planner;
    PlanningResult result = planner.plan(problem);

    std::cout << "Test 2 Valid Path Found: " << (result.success ? "PASS" : "FAIL") << "\n";
    // Check that path doesn't contain bad state '2'

    return 0;
}