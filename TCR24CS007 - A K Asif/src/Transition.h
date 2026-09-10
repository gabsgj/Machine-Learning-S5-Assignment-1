#ifndef TRANSITION_H
#define TRANSITION_H

#include <cstdint>

struct Transition {
    uint64_t id{};
    uint64_t from{};
    uint64_t to{};
    double cost{};
    double safety{};
    double reliability{1.0};
    bool available{true};

    Transition() = default;

    Transition(uint64_t id_, uint64_t from_, uint64_t to_,
               double cost_, double safety_, double reliability_,
               bool available_ = true)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}
};

#endif
