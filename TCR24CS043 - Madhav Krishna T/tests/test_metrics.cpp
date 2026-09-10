#include "safe_semantic_planner/core_types.hpp"
#include "safe_semantic_planner/dstar_lite.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

using namespace safe_semantic_planner;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Assertion failed: " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

PlanningProblem createTestGrid(int width, int height) {
    PlanningProblem p;
    p.initialState = "s_0_0";
    p.goalState = "s_" + std::to_string(width - 1) + "_" + std::to_string(height - 1);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::string id = "s_" + std::to_string(x) + "_" + std::to_string(y);
            p.states.emplace_back(id, std::vector<double>{static_cast<double>(x), static_cast<double>(y)});
        }
    }

    int tCount = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::string u = "s_" + std::to_string(x) + "_" + std::to_string(y);
            if (x + 1 < width) {
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, "s_" + std::to_string(x + 1) + "_" + std::to_string(y), 1.0, 0.95);
            }
            if (y + 1 < height) {
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, "s_" + std::to_string(x) + "_" + std::to_string(y + 1), 1.0, 0.95);
            }
            if (x + 1 < width && y + 1 < height) {
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, "s_" + std::to_string(x + 1) + "_" + std::to_string(y + 1), 1.414, 0.90);
            }
        }
    }
    return p;
}

void testInstrumentationAndMetrics() {
    std::cout << "[RUNNING] testInstrumentationAndMetrics..." << std::endl;

    PlanningProblem prob = createTestGrid(5, 5);
    prob.badStates = {"s_2_2"};
    WeightParams params(1.0, 1.0, 1.0, 0.1, 0.5);

    DStarLitePlanner planner;
    planner.initialize(prob, params);

    // Initial Solve
    PlanningResult res1 = planner.computeShortestPath();
    TEST_ASSERT(res1.success, "Initial solve must succeed");

    PlannerMetrics m1 = planner.getMetrics();
    std::cout << "Initial solve metrics: "
              << "statesExplored=" << m1.statesExplored
              << ", planningTimeMs=" << m1.planningTimeMs
              << ", memoryBytes=" << m1.memoryBytes
              << ", totalCost=" << m1.totalCost
              << ", minClearance=" << m1.minClearance
              << ", badStatesVisited=" << m1.badStatesVisited
              << ", goalSuccessCount=" << m1.goalSuccessCount
              << ", attemptCount=" << m1.attemptCount
              << ", successRate=" << m1.getGoalSuccessRate() << std::endl;

    TEST_ASSERT(m1.statesExplored > 0, "Initial statesExplored must be > 0");
    TEST_ASSERT(m1.planningTimeMs >= 0.0, "planningTimeMs must be non-negative");
    TEST_ASSERT(m1.badStatesVisited == 0, "Zero bad states must be visited");
    TEST_ASSERT(m1.minClearance > 0.5 - 1e-6, "minClearance must exceed safety radius r=0.5");
    TEST_ASSERT(m1.attemptCount == 1, "attemptCount should be 1");
    TEST_ASSERT(m1.goalSuccessCount == 1, "goalSuccessCount should be 1");
    TEST_ASSERT(std::abs(m1.getGoalSuccessRate() - 1.0) < 1e-6, "Initial success rate must be 100%");

    // Replan with Goal Shift
    planner.notifyGoalChanged("s_4_3");
    PlanningResult res2 = planner.computeShortestPath();
    TEST_ASSERT(res2.success, "Replan with goal shift must succeed");

    PlannerMetrics m2 = planner.getMetrics();
    std::cout << "Replan metrics: "
              << "statesExplored=" << m2.statesExplored
              << ", replanningTimeMs=" << m2.replanningTimeMs
              << ", goalSuccessCount=" << m2.goalSuccessCount
              << ", attemptCount=" << m2.attemptCount
              << ", successRate=" << m2.getGoalSuccessRate() << std::endl;

    TEST_ASSERT(m2.statesExplored >= 0, "Replan statesExplored must be recorded for this call");
    TEST_ASSERT(m2.replanningTimeMs >= 0.0, "replanningTimeMs must be non-negative");
    TEST_ASSERT(m2.attemptCount == 2, "attemptCount should be 2");
    TEST_ASSERT(m2.goalSuccessCount == 2, "goalSuccessCount should be 2");
    TEST_ASSERT(std::abs(m2.getGoalSuccessRate() - 1.0) < 1e-6, "Success rate should still be 1.0");

    // Block with impossible safety radius to test failure metrics & running success rate
    planner.notifySafetyRadiusChanged(10.0); // Excludes all states
    PlanningResult res3 = planner.computeShortestPath();
    TEST_ASSERT(!res3.success, "Should fail when all states excluded");

    PlannerMetrics m3 = planner.getMetrics();
    std::cout << "Failed attempt metrics: "
              << "goalSuccessCount=" << m3.goalSuccessCount
              << ", attemptCount=" << m3.attemptCount
              << ", successRate=" << m3.getGoalSuccessRate() << std::endl;

    TEST_ASSERT(m3.attemptCount == 3, "attemptCount must now be 3");
    TEST_ASSERT(m3.goalSuccessCount == 2, "goalSuccessCount must still be 2");
    TEST_ASSERT(std::abs(m3.getGoalSuccessRate() - (2.0 / 3.0)) < 1e-6, "Success rate must be 2/3 = 66.67%");

    std::cout << "[PASSED] testInstrumentationAndMetrics" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "Planner Metrics & Instrumentation Tests" << std::endl;
    std::cout << "========================================" << std::endl;

    testInstrumentationAndMetrics();

    std::cout << "========================================" << std::endl;
    std::cout << "ALL METRICS TESTS PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
