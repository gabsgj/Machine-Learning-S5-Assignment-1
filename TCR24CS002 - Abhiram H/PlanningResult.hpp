#pragma once
#include <cstdint>
#include <vector>

class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;   // minimum distance to a bad state along the path

    // Extra experimental/telemetry fields (additive; do not break the
    // interface given in the brief, just extend it for reporting).
    uint64_t exploredStates = 0;
    double planningTimeMs = 0.0;
    double cumulativeReliability = 0.0;
};
