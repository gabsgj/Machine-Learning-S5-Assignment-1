#ifndef PLANNER_H
#define PLANNER_H

#include "PlanningProblem.h"
#include "PlanningResult.h"

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;

    virtual ~Planner() {}
};

#endif