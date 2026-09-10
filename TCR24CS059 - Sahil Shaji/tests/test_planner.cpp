#include "DStarLite.h"
#include <iostream>
#include <vector>
#include <cassert>

void printPath(const PlanningResult& result) {
    if (!result.success) {
        std::cout << "Path not found.\n";
        return;
    }
    std::cout << "Path: ";
    for (size_t i = 0; i < result.statePath.size(); ++i) {
        std::cout << result.statePath[i] << (i + 1 == result.statePath.size() ? "" : " -> ");
    }
    std::cout << "\nCost: " << result.totalCost << ", Safety Score: " << result.safetyScore << "\n";
}

void testCase1() {
    std::cout << "--- Test Case 1: Basic Reachability ---\n";
    PlanningProblem p;
    p.initialState = 0; // S
    p.goalState = 3;    // G
    p.states = {
        {0, {0, 0}}, // S
        {1, {1, 0}}, // A
        {2, {2, 0}}, // B
        {3, {3, 0}}  // G
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true}
    };
    
    DStarLite planner;
    PlanningResult res = planner.plan(p);
    printPath(res);
    assert(res.success && res.statePath.size() == 4);
}

void testCase2() {
    std::cout << "--- Test Case 2: Bad State Avoidance ---\n";
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 5;
    p.badStates = {2}; // X is bad
    p.states = {
        {0, {0, 0}}, // S
        {1, {1, 1}}, // A
        {2, {2, 1}}, // X (bad)
        {3, {1, -1}},// C
        {4, {2, -1}},// D
        {5, {3, 0}}  // G
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 5, 1.0, 1.0, 1.0, true},
        {3, 0, 3, 1.0, 1.0, 1.0, true},
        {4, 3, 4, 1.0, 1.0, 1.0, true},
        {5, 4, 5, 1.0, 1.0, 1.0, true}
    };

    DStarLite planner;
    PlanningResult res = planner.plan(p);
    printPath(res);
    // Should take S -> C -> D -> G (0->3->4->5)
    assert(res.success && res.statePath[1] == 3);
}

void testCase3() {
    std::cout << "--- Test Case 3: Safety Margin ---\n";
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {4}; // B is bad state at (1.5, 0)
    p.states = {
        {0, {0, 0}},   // S
        {1, {1.5, 0.5}}, // Path 1 (close to bad state, low base cost)
        {2, {1.5, 2.0}}, // Path 2 (far from bad state, high base cost)
        {3, {3, 0}},   // G
        {4, {1.5, 0}}  // Bad State
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 3, 1.0, 1.0, 1.0, true},
        {2, 0, 2, 2.0, 1.0, 1.0, true}, // higher cost
        {3, 2, 3, 2.0, 1.0, 1.0, true}  // higher cost
    };

    DStarLite planner(1.0, 10.0, 0.0); // High safety weight
    PlanningResult res = planner.plan(p);
    printPath(res);
    // Should prefer path 2 because of high safety penalty on path 1
    assert(res.success && res.statePath[1] == 2);
}

void testCase4() {
    std::cout << "--- Test Case 4: Dynamic Transition ---\n";
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 2;
    p.states = {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {1, 1}}
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 0, 3, 1.5, 1.0, 1.0, true},
        {3, 3, 2, 1.5, 1.0, 1.0, true}
    };
    
    DStarLite planner;
    PlanningResult res = planner.plan(p);
    std::cout << "Initial Plan:\n";
    printPath(res);

    std::cout << "Transition A->G becomes unavailable...\n";
    planner.updateTransition(1, 2, false);
    PlanningResult res2 = planner.replan(0);
    printPath(res2);
    assert(res2.success && res2.statePath[1] == 3);
}

void testCase5() {
    std::cout << "--- Test Case 5: Goal Update ---\n";
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 2;
    p.states = {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {2, 2}}
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 1, 3, 1.0, 1.0, 1.0, true}
    };
    
    DStarLite planner;
    PlanningResult res = planner.plan(p);
    std::cout << "Initial Plan:\n";
    printPath(res);

    std::cout << "Goal changes to node 3...\n";
    planner.updateGoal(3);
    PlanningResult res2 = planner.replan(0);
    printPath(res2);
    assert(res2.success && res2.statePath.back() == 3);
}

void testCase6() {
    std::cout << "--- Test Case 6: Transition Addition ---\n";
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 2;
    p.states = {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}}
    };
    p.transitions = {
        {0, 0, 1, 1.0, 1.0, 1.0, true},
        {1, 1, 2, 1.0, 1.0, 1.0, true}
    };
    
    DStarLite planner;
    PlanningResult res = planner.plan(p);
    std::cout << "Initial Plan:\n";
    printPath(res);

    std::cout << "Shortcut 0->2 added...\n";
    planner.addTransition({2, 0, 2, 1.5, 1.0, 1.0, true});
    PlanningResult res2 = planner.replan(0);
    printPath(res2);
    assert(res2.success && res2.totalCost <= res.totalCost);
}

int main() {
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    std::cout << "All tests passed successfully.\n";
    return 0;
}
