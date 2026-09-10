#pragma once
#include <cstdint>
#include "PlanningProblem.h"

struct GraphGenConfig {
    int numStates = 100;
    int dimensions = 2;
    double areaMin = 0.0;
    double areaMax = 100.0;
    double edgeProbability = 0.03; // expected density of directed edges
    int numBadStates = 5;
    unsigned int seed = 42;
    // Edge cost = EuclideanDistance(u,v) * uniform(1.0, costSlack). Keeping
    // cost >= distance preserves heuristic admissibility (see GraphGenerator.cpp).
    double costSlack = 1.4;
    double minReliability = 0.85;
    double maxReliability = 1.0;
    double minSafety = 0.5;
    double maxSafety = 1.0;
    double unavailableProbability = 0.02;
};

namespace graphgen {
// Generates a reproducible random Cartesian graph. State 0 is always the
// initial state and the last generated state is always the goal state,
// with a guaranteed path between them threaded through the random graph
// (so experiments don't waste runs on trivially-unreachable instances).
PlanningProblem generate(const GraphGenConfig& cfg);
}
