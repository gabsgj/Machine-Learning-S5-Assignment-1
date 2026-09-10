#pragma once
#include <cstdint>
#include <vector>

// A single state in the finite Cartesian state space S subset of R^d.
// The embedding is what lets us compute Euclidean distances for the
// heuristic and for the "distance to nearest bad state" safety metric.
struct State {
    uint64_t id;
    std::vector<double> embedding; // (x1, x2, ..., xd)
};
