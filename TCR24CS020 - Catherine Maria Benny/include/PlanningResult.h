#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Outcome of a planning (or replanning) call, plus the metrics required by
// the assignment. `success == false` always means statePath/transitionPath
// are empty and no valid path to the goal exists under current constraints.
class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;      // == minimumSafetyDistance (Euclidean, see Metrics.h)
    double reliability = 1.0;      // cumulative product of transition reliabilities
    double objectiveScore = 0.0;   // Score(P) = alpha*G - beta*C + gamma*D + delta*R
    int badStatesVisited = 0;      // must always be 0 for a successful plan
    int exploredStates = 0;        // number of states popped/expanded from the OPEN queue
    double planningTimeMs = 0.0;
    size_t memoryBytesEstimate = 0;
    std::string errorMessage;      // set when success == false, explains why
};
