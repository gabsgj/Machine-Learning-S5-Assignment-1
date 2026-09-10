#ifndef SAFETY_H
#define SAFETY_H

#include "PlanningProblem.h"
#include <vector>
#include <cstdint>

double euclideanDistance(
    const State& a,
    const State& b
);

double calculateSafetyDistance(
    const std::vector<uint64_t>& path,
    const PlanningProblem& problem
);

#endif