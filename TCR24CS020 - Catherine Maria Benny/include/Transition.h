#pragma once
#include <cstdint>

// A directed transition (edge) between two states.
// cost:        non-negative traversal cost, used by the planner to minimize path cost.
// safety:      a raw safety score in [0,1] describing the edge itself (not the same as
//              the *minimum safety distance* metric, which is computed geometrically
//              from Cartesian embeddings against bad states — see Metrics.h).
// reliability: probability in [0,1] that this transition succeeds. Path reliability is
//              the product of transition reliabilities along the path.
// available:   whether the transition can currently be used. Unavailable transitions are
//              treated by the planner exactly as if they did not exist in the graph.
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;

    Transition()
        : id(0), from(0), to(0), cost(0.0), safety(1.0),
          reliability(1.0), available(true) {}

    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_, double reliability_, bool available_)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}
};
