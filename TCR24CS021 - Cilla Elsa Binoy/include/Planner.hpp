#pragma once
#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};
