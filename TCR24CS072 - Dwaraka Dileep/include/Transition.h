#ifndef SAFE_PLANNER_TRANSITION_H
#define SAFE_PLANNER_TRANSITION_H

#include <cstdint>

namespace planner {

// A directed edge (s_i -> s_j) with the four attributes required by the
// assignment: cost, reliability, safety score and an availability flag
// (used to model transitions disappearing/reappearing in the dynamic
// environment scenarios).
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // per-edge safety score in [0, 1], higher = safer
    double reliability;  // per-edge reliability in [0, 1], higher = more reliable
    bool available;

    Transition()
        : id(0), from(0), to(0), cost(0.0), safety(1.0),
          reliability(1.0), available(true) {}

    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_ = 1.0, double reliability_ = 1.0,
               bool available_ = true)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}
};

} // namespace planner

#endif // SAFE_PLANNER_TRANSITION_H
