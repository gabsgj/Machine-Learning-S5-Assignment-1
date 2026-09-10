#include "dstar_lite.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

State state(uint64_t id, double x, double y = 0.0) {
    return {id, {x, y}};
}

Transition edge(uint64_t id, uint64_t from, uint64_t to, double cost,
                double reliability = 1.0, bool available = true) {
    return {id, from, to, cost, 1.0, reliability, available};
}

void requirePath(const PlanningResult& result, const std::vector<uint64_t>& expected) {
    assert(result.success);
    assert(result.statePath == expected);
    assert(result.exploredStates > 0);
    assert(result.planningTimeMs >= 0.0);
}

void printMetrics(const char* label, const PlanningResult& result) {
    std::cout << label
              << ": explored=" << result.exploredStates
              << ", planning_ms=" << result.planningTimeMs
              << ", replanning_ms=" << result.replanningTimeMs
              << ", peak_memory_kb=" << result.peakMemoryKB << '\n';
}

// Test case 1: Basic reachability.
void test1() {
    DStarLitePlanner planner;
    const PlanningProblem problem{1, 4, {},
        {state(1, 0), state(2, 1), state(3, 2), state(4, 3)},
        {edge(1, 1, 2, 1), edge(2, 2, 3, 1), edge(3, 3, 4, 1)}};
    const PlanningResult result = planner.plan(problem);
    requirePath(result, {1, 2, 3, 4});
    assert(std::abs(result.totalCost - 3.0) < 1e-9);
    printMetrics("test1", result);
}

// Test case 2: Bad-state avoidance.
void test2() {
    DStarLitePlanner planner;
    const PlanningProblem problem{1, 6, {3},
        {state(1, 0), state(2, 1), state(3, 2), state(4, 1), state(5, 2), state(6, 3)},
        {edge(1, 1, 2, 1), edge(2, 2, 3, 1), edge(3, 3, 6, 1),
         edge(4, 1, 4, 1), edge(5, 4, 5, 1), edge(6, 5, 6, 1)}};
    const PlanningResult result = planner.plan(problem);
    requirePath(result, {1, 4, 5, 6});
    assert(std::abs(result.totalCost - 3.0) < 1e-9);
    printMetrics("test2", result);
}

// Test case 3: Safety margin.
void test3() {
    const PlanningProblem problem{1, 5, {9},
        {state(1, 0, 0), state(2, 1, 0), state(3, 1, 10), state(4, 2, 10),
         state(5, 3, 0), state(9, 1, 0.1)},
        {edge(1, 1, 2, 1), edge(2, 2, 5, 1), edge(3, 1, 3, 2),
         edge(4, 3, 4, 2), edge(5, 4, 5, 2)}};
    DStarLitePlanner safetyPlanner(0.0, 1.0);
    const PlanningResult safeResult = safetyPlanner.plan(problem);
    requirePath(safeResult, {1, 3, 4, 5});
    assert(std::abs(safeResult.totalCost - 6.0) < 1e-9);

    DStarLitePlanner costPlanner(1.0, 0.0);
    const PlanningResult lowCostResult = costPlanner.plan(problem);
    requirePath(lowCostResult, {1, 2, 5});
    assert(std::abs(lowCostResult.totalCost - 2.0) < 1e-9);
    assert(safeResult.safetyScore > lowCostResult.safetyScore);
    printMetrics("test3_safety", safeResult);
    printMetrics("test3_cost", lowCostResult);
}

// Test case 4: Dynamic transition.
void test4() {
    DStarLitePlanner planner;
    PlanningProblem problem{1, 4, {},
        {state(1, 0), state(2, 1), state(3, 1), state(4, 2)},
        {edge(1, 1, 2, 1), edge(2, 2, 4, 1), edge(3, 1, 3, 2), edge(4, 3, 4, 2)}};
    requirePath(planner.plan(problem), {1, 2, 4});
    problem.transitions[1].available = false;
    const PlanningResult replanned = planner.plan(problem);
    requirePath(replanned, {1, 3, 4});
    assert(std::abs(replanned.totalCost - 4.0) < 1e-9);
    printMetrics("test4", replanned);
}

// Test case 5: Goal update.
void test5() {
    DStarLitePlanner planner;
    PlanningProblem problem{1, 4, {},
        {state(1, 0), state(2, 1), state(3, 1), state(4, 2)},
        {edge(1, 1, 2, 1), edge(2, 2, 4, 1), edge(3, 1, 3, 2), edge(4, 3, 4, 2)}};
    requirePath(planner.plan(problem), {1, 2, 4});
    problem.goalState = 3;
    const PlanningResult replanned = planner.plan(problem);
    requirePath(replanned, {1, 3});
    assert(std::abs(replanned.totalCost - 2.0) < 1e-9);
    printMetrics("test5", replanned);
}

// Test case 6: Transition addition.
void test6() {
    DStarLitePlanner planner;
    PlanningProblem problem{1, 4, {},
        {state(1, 0), state(2, 1), state(3, 1), state(4, 2)},
        {edge(1, 1, 2, 1), edge(2, 2, 4, 1), edge(3, 1, 3, 2), edge(4, 3, 4, 2)}};
    requirePath(planner.plan(problem), {1, 2, 4});
    problem.transitions.push_back(edge(5, 1, 4, 0.25));
    const PlanningResult replanned = planner.plan(problem);
    requirePath(replanned, {1, 4});
    assert(std::abs(replanned.totalCost - 0.25) < 1e-9);
    printMetrics("test6", replanned);
}

}  // namespace

int main() {
    test1();
    test2();
    test3();
    test4();
    test5();
    test6();
    std::cout << "All illustrative test cases passed\n";
}
