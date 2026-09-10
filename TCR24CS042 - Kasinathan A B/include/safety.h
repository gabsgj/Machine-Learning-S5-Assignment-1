#ifndef SAFETY_H
#define SAFETY_H

#include <cstdint>
#include <vector>
#include "state.h"

double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b);

double distanceToNearestBadState(
    const State& s,
    const std::vector<uint64_t>& badStateIds,
    const std::vector<State>& allStates
);

#endif