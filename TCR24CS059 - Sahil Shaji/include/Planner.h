#pragma once

#include "Types.h"

// Planner Interface as specified in assignment
class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};
