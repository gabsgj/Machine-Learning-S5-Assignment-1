#ifndef PLANNING_RESULT_H
#define PLANNING_RESULT_H

#include <cstdint>
#include <vector>

class PlanningResult
{
public:

    bool success;

    std::vector<uint64_t> statePath;

    std::vector<uint64_t> transitionPath;

    double totalCost;

    double safetyScore;

    PlanningResult()
        : success(false),
          totalCost(0.0),
          safetyScore(0.0)
    {
    }
};

#endif