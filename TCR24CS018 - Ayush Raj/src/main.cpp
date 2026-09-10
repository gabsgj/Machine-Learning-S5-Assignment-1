#include "dstar_lite.hpp"

#include <iostream>

int main() {
    PlanningProblem p{1, 3, {}, {{1, {0, 0}}, {2, {1, 0}}, {3, {2, 0}}},
                      {{1, 1, 2, 1.0, 1.0, 0.95, true}, {2, 2, 3, 1.0, 1.0, 0.95, true}}};
    DStarLitePlanner planner;
    const PlanningResult result = planner.plan(p);
    std::cout << (result.success ? "Path found: " : "No path\n");
    for (uint64_t id : result.statePath) std::cout << id << ' ';
    std::cout << "\nCost: " << result.totalCost << ", safety: " << result.safetyScore << '\n';
}
