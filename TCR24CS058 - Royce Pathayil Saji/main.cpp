#include "planner.hpp"
#include <iostream>

void printResult(const PlanningResult& res) {
    if (res.success) {
        std::cout << "Success\nCost: " << res.totalCost << "\nSafety: " << res.safetyScore << "\nPath: ";
        for (uint64_t s : res.statePath) {
            std::cout << s << " ";
        }
        std::cout << "\n\n";
    } else {
        std::cout << "Failed to find path\n\n";
    }
}

int main() {
    SafePlanner planner;
    planner.setGamma(10.0);

    PlanningProblem p1;
    p1.initialState = 1;
    p1.goalState = 4;
    p1.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {3.0, 0.0}}
    };
    p1.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true}
    };
    
    std::cout << "Test 1: Basic Reachability\n";
    printResult(planner.plan(p1));

    PlanningProblem p2 = p1;
    p2.states.push_back({5, {1.0, 1.0}});
    p2.states.push_back({6, {2.0, 1.0}});
    p2.badStates = {2};
    p2.transitions.push_back({4, 1, 5, 1.4, 1.0, 1.0, true});
    p2.transitions.push_back({5, 5, 6, 1.0, 1.0, 1.0, true});
    p2.transitions.push_back({6, 6, 4, 1.4, 1.0, 1.0, true});
    
    std::cout << "Test 2: Bad State Avoidance\n";
    printResult(planner.plan(p2));

    PlanningProblem p3 = p1;
    p3.states.push_back({5, {1.0, 2.0}});
    p3.states.push_back({6, {2.0, 2.0}});
    p3.states.push_back({7, {1.0, 1.0}});
    p3.badStates = {7};
    p3.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true},
        {4, 1, 5, 2.0, 1.0, 1.0, true},
        {5, 5, 6, 1.0, 1.0, 1.0, true},
        {6, 6, 4, 2.0, 1.0, 1.0, true}
    };
    
    std::cout << "Test 3: Safety Margin\n";
    printResult(planner.plan(p3));

    PlanningProblem p4 = p1;
    p4.transitions[1].available = false;
    
    std::cout << "Test 4: Dynamic Transition\n";
    printResult(planner.plan(p4));

    PlanningProblem p5 = p1;
    p5.goalState = 3;
    
    std::cout << "Test 5: Goal Update\n";
    printResult(planner.plan(p5));

    PlanningProblem p6 = p1;
    p6.transitions.push_back({4, 1, 4, 1.5, 1.0, 1.0, true});
    
    std::cout << "Test 6: Transition Addition\n";
    printResult(planner.plan(p6));

    return 0;
}
