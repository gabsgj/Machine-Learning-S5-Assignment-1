#pragma once
#include <cstdint>
#include <vector>

struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;        // sum of raw transition.cost along the path
    double safetyScore = 0.0;      // min Euclidean distance to any bad state, over the path

    // Extra diagnostics (not in the original spec, but required by the
    // "students should evaluate" section at the end of the assignment).
    int badStatesVisited = 0;
    int statesExpanded = 0;        // states popped from the open queue this call
    double planningTimeMs = 0.0;
    size_t peakOpenSetSize = 0;    // rough memory-footprint proxy
};
