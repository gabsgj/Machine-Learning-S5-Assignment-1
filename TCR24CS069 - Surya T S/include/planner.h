#ifndef PLANNER_H
#define PLANNER_H

#include "planning_problem.h"
#include "planning_result.h"

/// @brief Abstract planner interface.
/// All concrete planners must implement the plan() method.
class Planner {
public:
    /// @brief Compute a plan for the given problem.
    /// @param problem The complete planning problem specification.
    /// @return A PlanningResult containing the path and metrics.
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;

    virtual ~Planner() = default;
};

#endif // PLANNER_H
