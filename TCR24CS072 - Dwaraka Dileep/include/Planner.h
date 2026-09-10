#ifndef SAFE_PLANNER_PLANNER_H
#define SAFE_PLANNER_PLANNER_H

#include "PlanningProblem.h"
#include "PlanningResult.h"

namespace planner {

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

} // namespace planner

#endif // SAFE_PLANNER_PLANNER_H
