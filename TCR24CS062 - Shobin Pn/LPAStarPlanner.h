#ifndef LPA_STAR_PLANNER_H
#define LPA_STAR_PLANNER_H

#include <unordered_map>
#include <vector>
#include <queue>
#include <cstdint>

#include "Planner.h"

class LPAStarPlanner : public Planner
{
private:

    // Distance/value used by LPA*
    std::unordered_map<uint64_t, double> g;

    std::unordered_map<uint64_t, double> rhs;

    // Find a state using its ID
    const State* findState(
        const PlanningProblem& problem,
        uint64_t id
    ) const;

    // Calculate heuristic distance
    double heuristic(
        const State& a,
        const State& b
    ) const;

    // Check whether a state is a bad state
    bool isBadState(
        const PlanningProblem& problem,
        uint64_t stateId
    ) const;

public:

    PlanningResult plan(
        const PlanningProblem& problem
    ) override;
};

#endif