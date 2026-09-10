#ifndef SAFE_PLANNER_PLANNING_RESULT_H
#define SAFE_PLANNER_PLANNING_RESULT_H

#include <cstdint>
#include <vector>

namespace planner {

class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;      // minimum distance to any bad state along the path
    double reliabilityScore = 0.0; // cumulative (product) reliability along the path

    // Experimental / diagnostic metrics (not part of the required interface
    // in the assignment, but needed for the evaluation section).
    std::size_t statesExplored = 0;
    double planningTimeMs = 0.0;
};

} // namespace planner

#endif // SAFE_PLANNER_PLANNING_RESULT_H
