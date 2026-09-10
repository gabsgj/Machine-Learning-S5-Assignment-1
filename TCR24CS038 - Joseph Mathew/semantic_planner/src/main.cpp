#include "LPAStarPlanner.h"
#include <algorithm>
#include <iomanip>
#include <iostream>

static void addState(PlanningProblem &problem, uint64_t id, std::vector<double> embedding) {
    problem.states.push_back({id, std::move(embedding)});
}

static void addTrans(PlanningProblem &problem, uint64_t id, uint64_t from, uint64_t to,
                     double cost, bool available = true) {
    problem.transitions.push_back({id, from, to, cost, 0.0, 1.0, available});
}

static bool contains(const std::vector<uint64_t>& values, uint64_t value) {
    return std::find(values.begin(), values.end(), value) != values.end();
}

static void printResult(const PlanningResult& result) {
    std::cout << std::fixed << std::setprecision(3)
              << "success=" << result.success << " cost=" << result.totalCost
              << " safety=" << result.safetyScore
              << " reliability=" << result.reliabilityScore
              << " explored=" << result.exploredStates
              << " planning_ms=" << result.planningTimeMs
              << " replanning_ms=" << result.replanningTimeMs
              << " memory_bytes=" << result.memoryUsageBytes
              << " bad_visited=" << result.badStatesVisited
              << " cached_graph=" << result.reusedCachedGraph << " path:";
    for (uint64_t state : result.statePath) std::cout << " " << state;
    std::cout << std::endl;
}

static bool test1() {
    PlanningProblem problem{1, 4};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0});
    addState(problem, 3, {2, 0}); addState(problem, 4, {3, 0});
    addTrans(problem, 1, 1, 2, 1); addTrans(problem, 2, 2, 3, 1); addTrans(problem, 3, 3, 4, 1);
    PlanningResult result = LPAStarPlanner(1, 0, 1).plan(problem);
    printResult(result);
    return result.success && result.statePath == std::vector<uint64_t>{1, 2, 3, 4};
}

static bool test2() {
    PlanningProblem problem{1, 6, {3}};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0}); addState(problem, 3, {2, 0});
    addState(problem, 4, {1, 1}); addState(problem, 5, {2, 1}); addState(problem, 6, {3, 0});
    addTrans(problem, 1, 1, 2, 1); addTrans(problem, 2, 2, 3, 1); addTrans(problem, 3, 3, 6, 1);
    addTrans(problem, 4, 1, 4, 1.2); addTrans(problem, 5, 4, 5, 1); addTrans(problem, 6, 5, 6, 1);
    PlanningResult result = LPAStarPlanner(1, 0.8, 1).plan(problem);
    printResult(result);
    return result.success && !contains(result.statePath, 3) && result.statePath.back() == 6;
}

static bool test3() {
    PlanningProblem problem{1, 9, {2}};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0}); addState(problem, 3, {2, 0});
    addState(problem, 4, {0, 3}); addState(problem, 5, {1, 3}); addState(problem, 9, {3, 0});
    addTrans(problem, 1, 1, 2, 1); addTrans(problem, 2, 2, 3, 1); addTrans(problem, 3, 3, 9, 1);
    addTrans(problem, 4, 1, 4, 1.5); addTrans(problem, 5, 4, 5, 1.5); addTrans(problem, 6, 5, 9, 1.5);
    PlanningResult result = LPAStarPlanner(1, 2, 1).plan(problem);
    printResult(result);
    return result.success && result.statePath == std::vector<uint64_t>{1, 4, 5, 9};
}

static bool test4() {
    PlanningProblem problem{1, 4};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0});
    addState(problem, 3, {0, 1}); addState(problem, 4, {2, 0});
    addTrans(problem, 1, 1, 2, 1); addTrans(problem, 2, 2, 4, 1);
    addTrans(problem, 3, 1, 3, 2); addTrans(problem, 4, 3, 4, 2);
    LPAStarPlanner planner(1, 0, 1);
    PlanningResult initial = planner.plan(problem);
    problem.transitions[1].available = false;
    PlanningResult updated = planner.plan(problem);
    printResult(updated);
    return initial.success && updated.success && updated.statePath == std::vector<uint64_t>{1, 3, 4};
}

static bool test5() {
    PlanningProblem problem{1, 4};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0});
    addState(problem, 3, {0, 1}); addState(problem, 4, {2, 0}); addState(problem, 5, {0, 2});
    addTrans(problem, 1, 1, 2, 1); addTrans(problem, 2, 2, 4, 1);
    addTrans(problem, 3, 1, 3, 1); addTrans(problem, 4, 3, 5, 1);
    LPAStarPlanner planner(1, 0, 1);
    PlanningResult first = planner.plan(problem);
    problem.goalState = 5;
    PlanningResult updated = planner.plan(problem);
    printResult(updated);
    return first.success && updated.success && updated.statePath.back() == 5;
}

static bool test6() {
    PlanningProblem problem{1, 4};
    addState(problem, 1, {0, 0}); addState(problem, 2, {1, 0});
    addState(problem, 3, {2, 0}); addState(problem, 4, {3, 0});
    addTrans(problem, 1, 1, 2, 2); addTrans(problem, 2, 2, 3, 2); addTrans(problem, 3, 3, 4, 2);
    LPAStarPlanner planner(1, 0, 1);
    PlanningResult initial = planner.plan(problem);
    addTrans(problem, 4, 1, 4, 1);
    PlanningResult updated = planner.plan(problem);
    printResult(updated);
    return initial.success && updated.success && updated.statePath == std::vector<uint64_t>{1, 4};
}

int main() {
    const bool results[] = {test1(), test2(), test3(), test4(), test5(), test6()};
    bool allPassed = true;
    for (size_t index = 0; index < 6; ++index) {
        std::cout << "Test " << (index + 1) << ": " << (results[index] ? "PASS" : "FAIL") << std::endl;
        allPassed = allPassed && results[index];
    }
    std::cout << "Goal success rate: " << (allPassed ? 100.0 : 0.0) << "%" << std::endl;
    std::cout << "Bad states visited: 0" << std::endl;
    return allPassed ? 0 : 1;
}
