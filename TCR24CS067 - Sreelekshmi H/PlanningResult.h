#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <cstdint>
#include <vector>

class PlanningResult {
public:
    bool success;

    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;

    double totalCost;
    double safetyScore;

    // Experimental metrics
    int exploredStates;
    double planningTimeMs;
    double memoryUsageKB;
    int badStatesVisited;

    PlanningResult() {
        success = false;
        totalCost = 0.0;
        safetyScore = 0.0;

        exploredStates = 0;
        planningTimeMs = 0.0;
        memoryUsageKB = 0.0;
        badStatesVisited = 0;
    }
};

#endif