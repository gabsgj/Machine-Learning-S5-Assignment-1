#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "planner.hpp"

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAILED] " << msg << " (" << #cond << ") at line " << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

void testBasicReachability() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {3.0, 0.0}}
    };
    problem.transitions = {
        {101, 1, 2, 2.0, 1.0, 1.0, true},
        {102, 2, 3, 2.0, 1.0, 1.0, true},
        {103, 3, 4, 2.0, 1.0, 1.0, true}
    };

    Planner planner;
    PlanningResult res = planner.plan(problem);
    TEST_ASSERT(res.success, "Basic reachability plan should succeed");
    TEST_ASSERT(res.statePath.size() == 4, "Path should have 4 states");
    TEST_ASSERT(res.statePath[0] == 1 && res.statePath[3] == 4, "Path should go from S(1) to G(4)");
    TEST_ASSERT(std::abs(res.totalCost - 6.0) < 1e-6, "Cost should be 6.0");
    TEST_ASSERT(res.badStatesVisited == 0, "No bad states visited");
    std::cout << "[PASSED] Basic Reachability Test\n";
}

void testBadStateAvoidance() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {3}; // X is bad
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}}, // Bad state X
        {4, {3.0, 0.0}},
        {5, {0.0, 1.0}},
        {6, {3.0, 1.0}}
    };
    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true},
        {102, 2, 3, 1.0, 0.0, 1.0, true},
        {103, 3, 4, 1.0, 0.0, 1.0, true},
        {104, 1, 5, 2.0, 1.0, 1.0, true},
        {105, 5, 6, 3.0, 1.0, 1.0, true},
        {106, 6, 4, 2.0, 1.0, 1.0, true}
    };

    Planner planner;
    PlanningResult res = planner.plan(problem);
    TEST_ASSERT(res.success, "Planner should find alternative route avoiding bad state");
    TEST_ASSERT(res.statePath.size() == 4, "Alternative path should have 4 states (S->C->D->G)");
    TEST_ASSERT(res.statePath == std::vector<uint64_t>({1, 5, 6, 4}), "Path must avoid X");
    TEST_ASSERT(std::abs(res.totalCost - 7.0) < 1e-6, "Cost should be 7.0");
    TEST_ASSERT(res.badStatesVisited == 0, "No bad states visited");
    TEST_ASSERT(res.safetyScore > 0.0, "Safety score must be positive");
    std::cout << "[PASSED] Bad State Avoidance Test\n";
}

void testSafetyMarginTradeoff() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {5};
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {2.0, 0.2}}, // Close to B_bad (0.2)
        {3, {2.0, 3.0}}, // Far from B_bad (3.0)
        {4, {4.0, 0.0}},
        {5, {2.0, 0.0}}  // Bad state
    };
    problem.transitions = {
        {101, 1, 2, 2.5, 0.2, 1.0, true},
        {102, 2, 4, 2.5, 0.2, 1.0, true},
        {103, 1, 3, 4.0, 3.0, 1.0, true},
        {104, 3, 4, 4.0, 3.0, 1.0, true}
    };

    // Weight 0.0 -> Cheaper path
    Planner plannerCostOnly;
    plannerCostOnly.setSafetyWeight(0.0);
    PlanningResult res1 = plannerCostOnly.plan(problem);
    TEST_ASSERT(res1.success, "Cost-only plan should succeed");
    TEST_ASSERT(res1.statePath == std::vector<uint64_t>({1, 2, 4}), "Should choose cheap path");
    TEST_ASSERT(std::abs(res1.totalCost - 5.0) < 1e-6, "Cost should be 5.0");

    // Weight 1.0 -> Safer path
    Planner plannerSafe;
    plannerSafe.setSafetyWeight(1.0);
    PlanningResult res2 = plannerSafe.plan(problem);
    TEST_ASSERT(res2.success, "Safety-weighted plan should succeed");
    TEST_ASSERT(res2.statePath == std::vector<uint64_t>({1, 3, 4}), "Should choose safer path");
    TEST_ASSERT(std::abs(res2.totalCost - 8.0) < 1e-6, "Cost should be 8.0");
    TEST_ASSERT(res2.safetyScore > res1.safetyScore, "Safer path must have strictly higher safety score");
    std::cout << "[PASSED] Safety Margin Trade-off Test\n";
}

void testDynamicTransitionReplanning() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {1.0, 1.0}},
        {4, {2.0, 0.0}}
    };
    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true},
        {102, 2, 4, 1.0, 1.0, 1.0, true},
        {103, 1, 3, 2.0, 1.0, 1.0, true},
        {104, 3, 4, 2.0, 1.0, 1.0, true}
    };

    Planner planner;
    PlanningResult res1 = planner.plan(problem);
    TEST_ASSERT(res1.success && res1.statePath == std::vector<uint64_t>({1, 2, 4}), "Initial path S->A->G");

    planner.updateTransition(102, false); // Disable A -> G
    PlanningResult res2 = planner.replan();
    TEST_ASSERT(res2.success, "Replan should succeed");
    TEST_ASSERT(res2.statePath == std::vector<uint64_t>({1, 3, 4}), "Replanned path must be S->B->G");
    TEST_ASSERT(std::abs(res2.totalCost - 4.0) < 1e-6, "Replanned cost should be 4.0");
    std::cout << "[PASSED] Dynamic Transition Replanning Test\n";
}

void testGoalUpdateReplanning() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 3;
    problem.badStates = {};
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {0.0, 1.0}},
        {5, {0.0, 2.0}}
    };
    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true},
        {102, 2, 3, 1.0, 1.0, 1.0, true},
        {103, 1, 4, 1.5, 1.0, 1.0, true},
        {104, 4, 5, 1.5, 1.0, 1.0, true}
    };

    Planner planner;
    PlanningResult res1 = planner.plan(problem);
    TEST_ASSERT(res1.success && res1.statePath == std::vector<uint64_t>({1, 2, 3}), "Initial path to G1");

    planner.updateGoal(5); // Switch goal to G2
    PlanningResult res2 = planner.replan();
    TEST_ASSERT(res2.success, "Goal update replan should succeed");
    TEST_ASSERT(res2.statePath == std::vector<uint64_t>({1, 4, 5}), "Replanned path must be to G2");
    std::cout << "[PASSED] Goal Update Replanning Test\n";
}

void testTransitionAdditionReplanning() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};
    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {2.5, 0.0}}
    };
    problem.transitions = {
        {101, 1, 2, 2.0, 1.0, 1.0, true},
        {102, 2, 3, 2.0, 1.0, 1.0, true},
        {103, 3, 4, 2.0, 1.0, 1.0, true}
    };

    Planner planner;
    PlanningResult res1 = planner.plan(problem);
    TEST_ASSERT(res1.success && std::abs(res1.totalCost - 6.0) < 1e-6, "Initial cost 6.0");

    Transition shortcut = {104, 1, 4, 2.5, 1.0, 1.0, true};
    planner.addTransition(shortcut);
    PlanningResult res2 = planner.replan();
    TEST_ASSERT(res2.success, "Shortcut replan should succeed");
    TEST_ASSERT(res2.statePath == std::vector<uint64_t>({1, 4}), "Path should take direct shortcut S->G");
    TEST_ASSERT(std::abs(res2.totalCost - 2.5) < 1e-6, "New cost should be 2.5");
    std::cout << "[PASSED] Transition Addition Replanning Test\n";
}

void testArbitraryDimensionDistance() {
    // 1D
    State s1{1, {3.0}};
    State s2{2, {7.0}};
    TEST_ASSERT(std::abs(Planner::heuristic(s1, s2) - 4.0) < 1e-6, "1D Euclidean distance");

    // 3D
    State s3D_1{3, {1.0, 2.0, 3.0}};
    State s3D_2{4, {4.0, 6.0, 3.0}}; // sqrt(3^2 + 4^2 + 0) = 5.0
    TEST_ASSERT(std::abs(Planner::heuristic(s3D_1, s3D_2) - 5.0) < 1e-6, "3D Euclidean distance");

    // 5D
    State s5D_1{5, {1.0, 2.0, 3.0, 4.0, 5.0}};
    State s5D_2{6, {1.0, 2.0, 3.0, 4.0, 5.0}};
    TEST_ASSERT(std::abs(Planner::heuristic(s5D_1, s5D_2) - 0.0) < 1e-6, "5D zero distance");

    std::cout << "[PASSED] Arbitrary Dimension Distance Test\n";
}

void testEdgeCases() {
    // Case A: Disconnected graph / No path exists
    {
        PlanningProblem problem;
        problem.initialState = 1;
        problem.goalState = 2;
        problem.badStates = {};
        problem.states = {{1, {0.0}}, {2, {10.0}}};
        problem.transitions = {}; // No transitions

        Planner planner;
        PlanningResult res = planner.plan(problem);
        TEST_ASSERT(!res.success, "Disconnected graph should return failure");
    }

    // Case B: Start == Goal
    {
        PlanningProblem problem;
        problem.initialState = 1;
        problem.goalState = 1;
        problem.badStates = {};
        problem.states = {{1, {0.0}}};
        problem.transitions = {};

        Planner planner;
        PlanningResult res = planner.plan(problem);
        TEST_ASSERT(res.success, "Start == Goal should return success");
        TEST_ASSERT(res.statePath == std::vector<uint64_t>({1}), "Path should contain just start state");
        TEST_ASSERT(std::abs(res.totalCost - 0.0) < 1e-6, "Cost should be 0");
    }

    // Case C: Start or Goal in Bad States
    {
        PlanningProblem problem;
        problem.initialState = 1;
        problem.goalState = 2;
        problem.badStates = {1}; // Start is bad
        problem.states = {{1, {0.0}}, {2, {1.0}}};
        problem.transitions = {{101, 1, 2, 1.0, 1.0, 1.0, true}};

        Planner planner;
        PlanningResult res = planner.plan(problem);
        TEST_ASSERT(!res.success, "Start state in bad states must fail");
    }

    std::cout << "[PASSED] Edge Cases Test (Disconnected, Start==Goal, Bad Start/Goal)\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "     RUNNING PLANNER UNIT TESTS         \n";
    std::cout << "========================================\n";

    testBasicReachability();
    testBadStateAvoidance();
    testSafetyMarginTradeoff();
    testDynamicTransitionReplanning();
    testGoalUpdateReplanning();
    testTransitionAdditionReplanning();
    testArbitraryDimensionDistance();
    testEdgeCases();

    std::cout << "\n>>> ALL UNIT TESTS PASSED SUCCESSFULLY! <<<\n";
    return 0;
}
