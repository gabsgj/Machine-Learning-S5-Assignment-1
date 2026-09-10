#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

#endif // PLANNER_HPP
