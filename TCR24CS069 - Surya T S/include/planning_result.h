#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <cstdint>
#include <vector>

/// @brief Stores the result of a planning query.
class PlanningResult {
public:
    bool success;                           ///< Whether a valid path was found
    std::vector<uint64_t> statePath;        ///< Ordered sequence of state IDs
    std::vector<uint64_t> transitionPath;   ///< Ordered sequence of transition IDs
    double totalCost;                       ///< Cumulative transition cost
    double safetyScore;                     ///< Minimum Euclidean distance to nearest bad state

    PlanningResult()
        : success(false), totalCost(0.0), safetyScore(0.0) {}
};

#endif // PLANNING_RESULT_H
