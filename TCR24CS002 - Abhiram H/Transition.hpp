#pragma once
#include <cstdint>

// A directed edge between two states, as specified in the assignment brief.
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;         // >= 0, raw traversal cost
    double safety;       // in [0,1], 1 = perfectly safe
    double reliability;  // in [0,1], 1 = perfectly reliable
    bool available;      // can be flipped at runtime for dynamic edges

    Transition() = default;
    Transition(uint64_t id_, uint64_t from_, uint64_t to_,
               double cost_, double safety_, double reliability_, bool available_ = true)
        : id(id_), from(from_), to(to_),
          cost(cost_), safety(safety_), reliability(reliability_), available(available_) {}
};
