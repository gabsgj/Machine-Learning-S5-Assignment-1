#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <cstdint>
#include <vector>

struct PlanningResult {
    bool success = false;

    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;

    double totalCost = 0.0;
    double safetyScore = 0.0;
    double reliability = 0.0;

    int exploredStates = 0;

    double planningTimeMs = 0.0;
    double replanningTimeMs = 0.0;
};

#endif