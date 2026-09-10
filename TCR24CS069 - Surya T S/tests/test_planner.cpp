/// @file test_planner.cpp
/// @brief Automated test suite for the Safe Semantic Planner.
///
/// Uses assertion-based testing to verify correctness of all 6 test cases
/// plus edge cases. Returns exit code 0 on success, 1 on failure.

#include "d_star_lite_planner.h"
#include "planning_problem.h"
#include "planning_result.h"
#include "safety_utils.h"

#include <cassert>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

static int totalTests  = 0;
static int passedTests = 0;

#define TEST(name) \
    do { \
        totalTests++; \
        std::cout << "  [TEST] " << name << "... "; \
    } while (0)

#define PASS() \
    do { \
        passedTests++; \
        std::cout << "PASS ✓\n"; \
    } while (0)

#define FAIL(msg) \
    do { \
        std::cout << "FAIL ✗ (" << msg << ")\n"; \
    } while (0)

#define ASSERT_TRUE(cond, msg) \
    do { \
        if (!(cond)) { FAIL(msg); return; } \
    } while (0)

#define ASSERT_EQ(a, b, msg) \
    do { \
        if ((a) != (b)) { FAIL(msg); return; } \
    } while (0)

#define ASSERT_NEAR(a, b, eps, msg) \
    do { \
        if (std::abs((a) - (b)) > (eps)) { FAIL(msg); return; } \
    } while (0)

// ═════════════════════════════════════════════════════════════════════════
// Safety Utils Tests
// ═════════════════════════════════════════════════════════════════════════

static void testEuclideanDistance() {
    TEST("Euclidean distance 2D");
    double d = safety::euclideanDistance({0.0, 0.0}, {3.0, 4.0});
    ASSERT_NEAR(d, 5.0, 1e-9, "Expected distance 5.0");
    PASS();
}

static void testEuclideanDistanceHighDim() {
    TEST("Euclidean distance 4D");
    double d = safety::euclideanDistance({1.0, 2.0, 3.0, 4.0}, {5.0, 6.0, 7.0, 8.0});
    // sqrt((4^2)*4) = sqrt(64) = 8
    ASSERT_NEAR(d, 8.0, 1e-9, "Expected distance 8.0");
    PASS();
}

static void testSafetyMapNoBadStates() {
    TEST("Safety map with no bad states");
    std::vector<State> states = {State(0, {0.0, 0.0}), State(1, {1.0, 0.0})};
    std::vector<uint64_t> bad = {};
    auto safetyMap = safety::computeSafetyMap(states, bad);
    // With no bad states, all distances should be infinity
    ASSERT_TRUE(std::isinf(safetyMap[0]), "Expected infinity for state 0");
    ASSERT_TRUE(std::isinf(safetyMap[1]), "Expected infinity for state 1");
    PASS();
}

static void testSafetyMapOneBadState() {
    TEST("Safety map with one bad state");
    std::vector<State> states = {
        State(0, {0.0, 0.0}),
        State(1, {3.0, 4.0}),
        State(2, {1.0, 0.0}),  // bad
    };
    std::vector<uint64_t> bad = {2};
    auto safetyMap = safety::computeSafetyMap(states, bad);
    ASSERT_NEAR(safetyMap[0], 1.0, 1e-9, "State 0 should be 1.0 from bad state 2");
    // State 1: dist to (1,0) = sqrt(4+16) = sqrt(20)
    ASSERT_NEAR(safetyMap[1], std::sqrt(20.0), 1e-9, "State 1 distance mismatch");
    ASSERT_NEAR(safetyMap[2], 0.0, 1e-9, "Bad state distance to itself should be 0");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC1: Basic Reachability
// ═════════════════════════════════════════════════════════════════════════

static void testBasicReachability() {
    TEST("TC1: Basic Reachability (S→A→B→G)");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 3;
    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}),
        State(2, {2.0, 0.0}), State(3, {3.0, 0.0}),
    };
    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),
        Transition(2, 2, 3, 1.0, 1.0, 1.0, true),
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(result.success, "Should find a path");
    ASSERT_EQ(result.statePath.size(), 4u, "Path should have 4 states");
    ASSERT_EQ(result.statePath[0], 0u, "Start should be 0");
    ASSERT_EQ(result.statePath[3], 3u, "End should be 3");
    ASSERT_NEAR(result.totalCost, 3.0, 1e-6, "Total cost should be 3.0");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC2: Bad State Avoidance
// ═════════════════════════════════════════════════════════════════════════

static void testBadStateAvoidance() {
    TEST("TC2: Bad State Avoidance");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 4;
    prob.badStates    = {2};

    prob.states = {
        State(0, {0.0, 0.0}),   State(1, {1.0, 1.0}),
        State(2, {2.0, 1.0}),   State(3, {1.0, -1.0}),
        State(4, {3.0, 0.0}),   State(5, {2.0, -1.0}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),
        Transition(2, 2, 4, 1.0, 1.0, 1.0, true),
        Transition(3, 0, 3, 1.0, 1.0, 1.0, true),
        Transition(4, 3, 5, 1.0, 1.0, 1.0, true),
        Transition(5, 5, 4, 1.0, 1.0, 1.0, true),
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(result.success, "Should find a path");
    // Verify no bad state in path
    for (uint64_t sid : result.statePath) {
        ASSERT_TRUE(sid != 2, "Path should not contain bad state 2");
    }
    // Expected path: 0 → 3 → 5 → 4
    ASSERT_EQ(result.statePath[0], 0u, "Start should be 0");
    ASSERT_EQ(result.statePath.back(), 4u, "End should be 4");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC3: Safety Margin
// ═════════════════════════════════════════════════════════════════════════

static void testSafetyMargin() {
    TEST("TC3: Safety Margin tradeoff");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 4;
    prob.badStates    = {5};

    prob.states = {
        State(0, {0.0, 0.0}),   State(1, {1.0, 0.0}),
        State(2, {0.0, -3.0}),  State(3, {2.0, -3.0}),
        State(4, {3.0, 0.0}),   State(5, {1.5, 0.5}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 0.9, true),
        Transition(1, 1, 4, 1.0, 1.0, 0.9, true),
        Transition(2, 0, 2, 2.0, 1.0, 0.9, true),
        Transition(3, 2, 3, 2.0, 1.0, 0.9, true),
        Transition(4, 3, 4, 2.0, 1.0, 0.9, true),
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(result.success, "Should find a path");
    // Path should not visit bad state
    for (uint64_t sid : result.statePath) {
        ASSERT_TRUE(sid != 5, "Path should not contain bad state 5");
    }
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC4: Dynamic Transition
// ═════════════════════════════════════════════════════════════════════════

static void testDynamicTransition() {
    TEST("TC4: Dynamic Transition Unavailability");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 2;

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}),
        State(2, {3.0, 0.0}), State(3, {1.0, -1.0}),
        State(4, {2.0, -1.0}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),   // Will become unavailable
        Transition(2, 0, 3, 1.5, 1.0, 0.9, true),
        Transition(3, 3, 4, 1.5, 1.0, 0.9, true),
        Transition(4, 4, 2, 1.5, 1.0, 0.9, true),
    };

    DStarLitePlanner planner;
    PlanningResult result1 = planner.plan(prob);

    ASSERT_TRUE(result1.success, "Initial plan should succeed");
    // Should use short path S→A→G
    ASSERT_EQ(result1.statePath.size(), 3u, "Initial path: 3 states (S→A→G)");

    // Disable A→G
    planner.setTransitionAvailability(1, false);
    PlanningResult result2 = planner.replan();

    ASSERT_TRUE(result2.success, "Replan should succeed");
    // Should find alternative path
    ASSERT_EQ(result2.statePath.back(), 2u, "Should still reach goal 2");
    ASSERT_TRUE(result2.statePath.size() > 3, "Should use longer path");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC5: Goal Update
// ═════════════════════════════════════════════════════════════════════════

static void testGoalUpdate() {
    TEST("TC5: Goal Update");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 3;

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}),
        State(2, {2.0, 0.0}), State(3, {3.0, 0.0}),
        State(4, {2.0, 2.0}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),
        Transition(2, 2, 3, 1.0, 1.0, 1.0, true),
        Transition(3, 1, 4, 1.5, 1.0, 1.0, true),
        Transition(4, 2, 4, 1.0, 1.0, 1.0, true),
    };

    DStarLitePlanner planner;
    PlanningResult result1 = planner.plan(prob);

    ASSERT_TRUE(result1.success, "Initial plan to G1 should succeed");
    ASSERT_EQ(result1.statePath.back(), 3u, "Should reach G1=3");

    // Change goal to G2=4
    planner.updateGoal(4);
    PlanningResult result2 = planner.replan();

    ASSERT_TRUE(result2.success, "Plan to G2 should succeed");
    ASSERT_EQ(result2.statePath.back(), 4u, "Should reach G2=4");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// TC6: Transition Addition (Shortcut)
// ═════════════════════════════════════════════════════════════════════════

static void testShortcutAddition() {
    TEST("TC6: Shortcut Transition Addition");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 4;

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}),
        State(2, {2.0, 0.0}), State(3, {3.0, 0.0}),
        State(4, {4.0, 0.0}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),
        Transition(2, 2, 3, 1.0, 1.0, 1.0, true),
        Transition(3, 3, 4, 1.0, 1.0, 1.0, true),
    };

    DStarLitePlanner planner;
    PlanningResult result1 = planner.plan(prob);

    ASSERT_TRUE(result1.success, "Initial plan should succeed");
    ASSERT_NEAR(result1.totalCost, 4.0, 1e-6, "Initial cost should be 4.0");

    // Add shortcut A→G (cost 1.0)
    Transition shortcut(10, 1, 4, 1.0, 1.0, 1.0, true);
    planner.addTransition(shortcut);
    PlanningResult result2 = planner.replan();

    ASSERT_TRUE(result2.success, "Should still find a path");
    ASSERT_TRUE(result2.totalCost <= result1.totalCost, "Should find cheaper or equal path");
    ASSERT_NEAR(result2.totalCost, 2.0, 1e-6, "Shortcut path cost should be 2.0");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// Edge Cases
// ═════════════════════════════════════════════════════════════════════════

static void testNoPathExists() {
    TEST("Edge: No path exists");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 2;

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}), State(2, {2.0, 0.0}),
    };

    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        // No transition from 1 to 2
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(!result.success, "Should report no path");
    PASS();
}

static void testStartIsGoal() {
    TEST("Edge: Start equals goal");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 0;

    prob.states = {State(0, {0.0, 0.0})};
    prob.transitions = {};

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(result.success, "Start == goal should succeed");
    ASSERT_EQ(result.statePath.size(), 1u, "Path should be just the start");
    ASSERT_NEAR(result.totalCost, 0.0, 1e-9, "Cost should be 0");
    PASS();
}

static void testAllTransitionsUnavailable() {
    TEST("Edge: All transitions unavailable");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 1;

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}),
    };
    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, false),  // Unavailable!
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(!result.success, "Should fail: only transition is unavailable");
    PASS();
}

static void testBadStateBlocksOnlyPath() {
    TEST("Edge: Bad state blocks only path");

    PlanningProblem prob;
    prob.initialState = 0;
    prob.goalState    = 2;
    prob.badStates    = {1};

    prob.states = {
        State(0, {0.0, 0.0}), State(1, {1.0, 0.0}), State(2, {2.0, 0.0}),
    };
    prob.transitions = {
        Transition(0, 0, 1, 1.0, 1.0, 1.0, true),
        Transition(1, 1, 2, 1.0, 1.0, 1.0, true),
    };

    DStarLitePlanner planner;
    PlanningResult result = planner.plan(prob);

    ASSERT_TRUE(!result.success, "Should fail: bad state blocks only path");
    PASS();
}

// ═════════════════════════════════════════════════════════════════════════
// Main
// ═════════════════════════════════════════════════════════════════════════

int main() {
    std::cout << "\n══════════════════════════════════════════════════════\n";
    std::cout << "  Safe Semantic Planner — Automated Test Suite\n";
    std::cout << "══════════════════════════════════════════════════════\n\n";

    // Safety utils
    testEuclideanDistance();
    testEuclideanDistanceHighDim();
    testSafetyMapNoBadStates();
    testSafetyMapOneBadState();

    std::cout << "\n";

    // Core test cases
    testBasicReachability();
    testBadStateAvoidance();
    testSafetyMargin();
    testDynamicTransition();
    testGoalUpdate();
    testShortcutAddition();

    std::cout << "\n";

    // Edge cases
    testNoPathExists();
    testStartIsGoal();
    testAllTransitionsUnavailable();
    testBadStateBlocksOnlyPath();

    std::cout << "\n══════════════════════════════════════════════════════\n";
    std::cout << "  Results: " << passedTests << " / " << totalTests << " tests passed";
    if (passedTests == totalTests) {
        std::cout << "  ✓ ALL PASS\n";
    } else {
        std::cout << "  ✗ SOME FAILURES\n";
    }
    std::cout << "══════════════════════════════════════════════════════\n\n";

    return (passedTests == totalTests) ? 0 : 1;
}
