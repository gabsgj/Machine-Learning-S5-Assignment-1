#include "safe_semantic_planner/core_types.hpp"
#include "safe_semantic_planner/problem_loader.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace safe_semantic_planner;

// Simple test assertions helper
#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "Assertion failed: " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

void testGraphWithNoBadStates() {
    std::cout << "[RUNNING] testGraphWithNoBadStates..." << std::endl;

    PlanningProblem prob;
    prob.initialState = "s_start";
    prob.goalState = "s_goal";
    prob.badStates = {}; // No bad states

    prob.states.push_back(State("s_start", {0.0, 0.0}));
    prob.states.push_back(State("s_mid",   {1.0, 0.0}));
    prob.states.push_back(State("s_goal",  {2.0, 0.0}));

    prob.transitions.push_back(Transition("t1", "s_start", "s_mid", 2.0, 0.9));
    prob.transitions.push_back(Transition("t2", "s_mid", "s_goal", 3.0, 0.8));

    WeightParams params(1.0, 1.0, 1.0, 0.5, 1.0); // beta=1.0, delta=0.5, r=1.0
    // w_min = 1.0 * min(2.0, 3.0) - 0.5 * max(0.9, 0.8) = 1.0 * 2.0 - 0.5 * 0.9 = 2.0 - 0.45 = 1.55 >= 0

    ProblemLoader loader(prob, params);

    TEST_ASSERT(loader.getActiveStates().size() == 3, "All 3 states should be active when no bad states exist");
    TEST_ASSERT(loader.getExcludedStates().empty(), "No states should be excluded");
    TEST_ASSERT(loader.getActiveTransitions().size() == 2, "All 2 transitions should be active");
    TEST_ASSERT(!loader.isInitialStateExcluded(), "Initial state must not be excluded");
    TEST_ASSERT(!loader.isGoalStateExcluded(), "Goal state must not be excluded");
    TEST_ASSERT(std::abs(loader.getWMin() - 1.55) < 1e-6, "w_min should match 1.55");
    // c_min: min((1.0*2.0 - 0.5*0.9)/1.0, (1.0*3.0 - 0.5*0.8)/1.0) = 1.55
    TEST_ASSERT(std::abs(loader.getCMin() - 1.55) < 1e-6, "c_min should match 1.55");

    for (const auto* s : loader.getActiveStates()) {
        TEST_ASSERT(std::isinf(s->distanceToNearestBadState), "Distance to nearest bad state should be infinity");
        TEST_ASSERT(!s->isExcluded, "State should not be excluded");
        TEST_ASSERT(!s->isBadState, "State should not be marked bad");
    }

    std::cout << "[PASSED] testGraphWithNoBadStates" << std::endl;
}

void testGoalWithinExclusionRadius() {
    std::cout << "[RUNNING] testGoalWithinExclusionRadius..." << std::endl;

    PlanningProblem prob;
    prob.initialState = "s0";
    prob.goalState = "s_goal";
    prob.badStates = {"s_bad"};

    prob.states.push_back(State("s0",    {0.0, 0.0}));
    prob.states.push_back(State("s1",    {1.0, 0.0}));
    prob.states.push_back(State("s_goal",{3.0, 0.0}));
    prob.states.push_back(State("s_bad", {3.2, 0.0})); // Goal is distance 0.2 from bad state

    prob.transitions.push_back(Transition("t1", "s0", "s1", 1.0, 0.95));
    prob.transitions.push_back(Transition("t2", "s1", "s_goal", 2.0, 0.90));
    prob.transitions.push_back(Transition("t3", "s1", "s_bad", 1.5, 0.50));

    // Safety exclusion radius r = 0.5
    // Distance(s_goal, s_bad) = 0.2 <= 0.5 -> s_goal is EXCLUDED!
    // Distance(s_bad, s_bad) = 0.0 <= 0.5 -> s_bad is EXCLUDED!
    // Distance(s1, s_bad) = 2.2 > 0.5 -> s1 is ACTIVE
    // Distance(s0, s_bad) = 3.2 > 0.5 -> s0 is ACTIVE
    WeightParams params(1.0, 1.0, 1.0, 0.1, 0.5);

    ProblemLoader loader(prob, params);

    TEST_ASSERT(loader.isGoalStateExcluded(), "Goal state must be detected as excluded");
    TEST_ASSERT(!loader.isInitialStateExcluded(), "Initial state should remain active");
    
    const State* goalState = loader.getState("s_goal");
    TEST_ASSERT(goalState != nullptr, "Goal state exists in lookup");
    TEST_ASSERT(goalState->isExcluded, "Goal state is marked excluded");
    TEST_ASSERT(std::abs(goalState->distanceToNearestBadState - 0.2) < 1e-6, "Goal distance to bad state should be 0.2");

    const State* badState = loader.getState("s_bad");
    TEST_ASSERT(badState != nullptr && badState->isBadState && badState->isExcluded, "Bad state is bad and excluded");

    TEST_ASSERT(loader.getActiveStates().size() == 2, "Only s0 and s1 should be active");
    TEST_ASSERT(loader.getExcludedStates().size() == 2, "s_goal and s_bad should be excluded");

    // Only t1 connects active states (s0 -> s1)
    TEST_ASSERT(loader.getActiveTransitions().size() == 1, "Only t1 should be active");
    TEST_ASSERT(loader.getActiveTransitions()[0]->id == "t1", "Active transition is t1");

    std::cout << "[PASSED] testGoalWithinExclusionRadius" << std::endl;
}

void testWeightValidationNegativeWMin() {
    std::cout << "[RUNNING] testWeightValidationNegativeWMin..." << std::endl;

    PlanningProblem prob;
    prob.initialState = "s0";
    prob.goalState = "s1";
    prob.badStates = {};

    prob.states.push_back(State("s0", {0.0, 0.0}));
    prob.states.push_back(State("s1", {1.0, 0.0}));

    // Transition with cost = 1.0, reliability = 0.95
    prob.transitions.push_back(Transition("t1", "s0", "s1", 1.0, 0.95));

    // If beta = 0.5, delta = 1.0:
    // w_min = 0.5 * 1.0 - 1.0 * 0.95 = 0.5 - 0.95 = -0.45 < 0
    WeightParams params(1.0, 0.5, 1.0, 1.0, 0.0);

    bool caught = false;
    try {
        ProblemLoader loader(prob, params);
    } catch (const ValidationException& ex) {
        caught = true;
        std::string err = ex.what();
        std::cout << "Caught expected ValidationException: " << err << std::endl;
        TEST_ASSERT(err.find("w_min") != std::string::npos, "Error should name w_min");
        TEST_ASSERT(err.find("min(cost)") != std::string::npos || err.find("min_cost") != std::string::npos, "Error should name min cost");
        TEST_ASSERT(err.find("max(reliability)") != std::string::npos || err.find("max_reliability") != std::string::npos, "Error should name max reliability");
    }

    TEST_ASSERT(caught, "ValidationException must be thrown when w_min < 0");
    std::cout << "[PASSED] testWeightValidationNegativeWMin" << std::endl;
}

void testIntermediateStateSafetyExclusion() {
    std::cout << "[RUNNING] testIntermediateStateSafetyExclusion..." << std::endl;

    PlanningProblem prob;
    prob.initialState = "A";
    prob.goalState = "D";
    prob.badStates = {"HAZARD"};

    // Layout:
    // A (0,0) -> B (2,0) -> D (4,0)
    // A (0,0) -> C (2,2) -> D (4,0)
    // HAZARD at (2, 0.2)
    prob.states.push_back(State("A", {0.0, 0.0}));
    prob.states.push_back(State("B", {2.0, 0.0}));
    prob.states.push_back(State("C", {2.0, 2.0}));
    prob.states.push_back(State("D", {4.0, 0.0}));
    prob.states.push_back(State("HAZARD", {2.0, 0.2}));

    prob.transitions.push_back(Transition("tAB", "A", "B", 2.0, 0.99));
    prob.transitions.push_back(Transition("tBD", "B", "D", 2.0, 0.99));
    prob.transitions.push_back(Transition("tAC", "A", "C", 2.828, 0.95));
    prob.transitions.push_back(Transition("tCD", "C", "D", 2.828, 0.95));

    // Radius r = 0.5
    // Distance(B, HAZARD) = 0.2 <= 0.5 -> B excluded!
    // Distance(C, HAZARD) = 1.8 > 0.5 -> C active
    WeightParams params(1.0, 1.0, 1.0, 0.2, 0.5);

    ProblemLoader loader(prob, params);

    TEST_ASSERT(loader.getState("B")->isExcluded, "B should be excluded because it's within r=0.5 of HAZARD");
    TEST_ASSERT(!loader.getState("C")->isExcluded, "C should remain active");
    TEST_ASSERT(!loader.isInitialStateExcluded(), "A is active");
    TEST_ASSERT(!loader.isGoalStateExcluded(), "D is active");

    // Only tAC and tCD should be active; tAB and tBD must be pruned because B is excluded
    TEST_ASSERT(loader.getActiveTransitions().size() == 2, "Only 2 transitions via C should be active");
    for (const auto* t : loader.getActiveTransitions()) {
        TEST_ASSERT(t->id == "tAC" || t->id == "tCD", "Active transitions must be tAC and tCD");
    }

    std::cout << "[PASSED] testIntermediateStateSafetyExclusion" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Stage 1 Core Model & ProblemLoader Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    testGraphWithNoBadStates();
    testGoalWithinExclusionRadius();
    testWeightValidationNegativeWMin();
    testIntermediateStateSafetyExclusion();

    std::cout << "========================================" << std::endl;
    std::cout << "ALL STAGE 1 TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
