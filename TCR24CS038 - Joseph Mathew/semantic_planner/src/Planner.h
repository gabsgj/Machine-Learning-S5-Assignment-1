#pragma once
#include "PlanningProblem.h"
#include <vector>

struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;
    double reliabilityScore = 0.0;
    std::size_t exploredStates = 0;
    double planningTimeMs = 0.0;
    double replanningTimeMs = 0.0;
    std::size_t memoryUsageBytes = 0;
    std::size_t badStatesVisited = 0;
    bool reusedCachedGraph = false;
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};
