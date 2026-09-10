#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <cstdint>
#include <vector>
#include <cstddef>

class PlanningResult {
public:
    bool success;

    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;

    double totalCost;
    double safetyScore;
    double minimumSafetyDistance;

    size_t exploredStates;

    double planningTimeMs;
    double replanningTimeMs;

    size_t memoryUsageBytes;

    PlanningResult();
};

#endif