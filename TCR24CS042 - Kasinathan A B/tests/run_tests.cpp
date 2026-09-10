#include "../include/planning_problem.h"
#include "../include/lpa_star.h"
#include <iostream>
#include <chrono>

void printResult(const std::string& label, const PlanningResult& r, double timeMs, int explored) {
    std::cout << "--- " << label << " ---\n";
    std::cout << "Success: " << (r.success ? "yes" : "no") << "\n";
    std::cout << "Path: ";
    for (uint64_t s : r.statePath) std::cout << s << " ";
    std::cout << "\n";
    std::cout << "Total cost: " << r.totalCost << "\n";
    std::cout << "Safety score (min distance to bad state): " << r.safetyScore << "\n";
    std::cout << "States explored: " << explored << "\n";
    std::cout << "Planning time: " << timeMs << " ms\n\n";
}

double elapsedMs(std::chrono::steady_clock::time_point start) {
    auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
}

// Test Case 1: Basic Reachability, S -> A -> B -> G
void testCase1() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.states = { {1,{0,0}}, {2,{1,0}}, {3,{2,0}}, {4,{3,0}} };
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r = planner.plan(p);
    printResult("Test Case 1: Basic Reachability", r, elapsedMs(start), planner.statesExpanded);
}

// Test Case 2: Bad State Avoidance, S->A->X->G (bad) vs S->C->G (safe)
void testCase2() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 5;
    p.states = { {1,{0,0}}, {2,{1,0}}, {3,{2,0}}, {4,{0,1}}, {5,{2,1}} };
    p.badStates = {3};
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 5, 1.0, 1.0, 1.0, true},
        {4, 1, 4, 1.0, 1.0, 1.0, true},
        {5, 4, 5, 1.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r = planner.plan(p);
    printResult("Test Case 2: Bad State Avoidance", r, elapsedMs(start), planner.statesExpanded);

    bool hitBad = false;
    for (uint64_t s : r.statePath) if (s == 3) hitBad = true;
    std::cout << "Bad states visited: " << (hitBad ? 1 : 0) << " (expected 0)\n\n";
}

// Test Case 3: Safety Margin, low-cost path close to bad state vs higher-cost path far from it
void testCase3() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    // Path A: 1 -> 2 -> 4, passes near bad state 5
    // Path B: 1 -> 3 -> 4, costs more but stays farther from bad state 5
    p.states = { {1,{0,0}}, {2,{1,0.1}}, {3,{1,3}}, {4,{2,0}}, {5,{1,0}} };
    p.badStates = {5};
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 4, 1.0, 1.0, 1.0, true},
        {3, 1, 3, 3.0, 1.0, 1.0, true},
        {4, 3, 4, 3.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r = planner.plan(p);
    printResult("Test Case 3: Safety Margin", r, elapsedMs(start), planner.statesExpanded);
    std::cout << "Chosen path balances cost against distance from bad state 5.\n\n";
}

// Test Case 4: Dynamic Transition, S->A->G, then A->G becomes unavailable
void testCase4() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 3;
    p.states = { {1,{0,0}}, {2,{1,0}}, {3,{2,0}} };
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 1, 3, 5.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r1 = planner.plan(p);
    printResult("Test Case 4a: Initial plan", r1, elapsedMs(start), planner.statesExpanded);

    auto replanStart = std::chrono::steady_clock::now();
    PlanningResult r2 = planner.onTransitionUnavailable(2, 2, 3);
    double replanTime = elapsedMs(replanStart);
    printResult("Test Case 4b: After A->G unavailable", r2, replanTime, planner.statesExpanded);
    std::cout << "Replanning time: " << replanTime << " ms\n\n";
}

// Test Case 5: Goal Update
void testCase5() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 3;
    p.states = { {1,{0,0}}, {2,{1,0}}, {3,{2,0}}, {4,{3,0}} };
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r1 = planner.plan(p);
    printResult("Test Case 5a: Initial goal = state 3", r1, elapsedMs(start), planner.statesExpanded);

    auto replanStart = std::chrono::steady_clock::now();
    PlanningResult r2 = planner.onGoalChanged(4);
    double replanTime = elapsedMs(replanStart);
    printResult("Test Case 5b: Goal changed to state 4", r2, replanTime, planner.statesExpanded);
    std::cout << "Replanning time: " << replanTime << " ms\n\n";
}

// Test Case 6: Transition Addition, a shortcut appears
void testCase6() {
    PlanningProblem p;
    p.initialState = 1; p.goalState = 4;
    p.states = { {1,{0,0}}, {2,{1,0}}, {3,{2,0}}, {4,{3,0}} };
    p.transitions = {
        {1, 1, 2, 1.0, 1.0, 1.0, true},
        {2, 2, 3, 1.0, 1.0, 1.0, true},
        {3, 3, 4, 1.0, 1.0, 1.0, true}
    };

    LPAStarPlanner planner;
    auto start = std::chrono::steady_clock::now();
    PlanningResult r1 = planner.plan(p);
    printResult("Test Case 6a: Before shortcut", r1, elapsedMs(start), planner.statesExpanded);

    Transition shortcut = {4, 1, 4, 1.5, 1.0, 1.0, true}; // direct S -> G, cheaper overall
    auto replanStart = std::chrono::steady_clock::now();
    PlanningResult r2 = planner.onTransitionAdded(shortcut);
    double replanTime = elapsedMs(replanStart);
    printResult("Test Case 6b: After shortcut added", r2, replanTime, planner.statesExpanded);
    std::cout << "Replanning time: " << replanTime << " ms\n\n";
}

int main() {
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    return 0;
}