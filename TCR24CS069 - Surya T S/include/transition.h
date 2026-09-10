#ifndef TRANSITION_H
#define TRANSITION_H

#include <cstdint>

/// @brief Represents a directed transition between two states.
/// Each transition carries cost, safety, reliability, and availability metadata.
class Transition {
public:
    uint64_t id;          ///< Unique transition identifier
    uint64_t from;        ///< Source state ID
    uint64_t to;          ///< Destination state ID
    double   cost;        ///< Transition cost (>= 0)
    double   safety;      ///< Safety score of this transition
    double   reliability; ///< Reliability score [0, 1]
    bool     available;   ///< Whether this transition is currently traversable

    Transition()
        : id(0), from(0), to(0), cost(0.0), safety(0.0),
          reliability(1.0), available(true) {}

    Transition(uint64_t id, uint64_t from, uint64_t to,
               double cost, double safety, double reliability, bool available)
        : id(id), from(from), to(to), cost(cost), safety(safety),
          reliability(reliability), available(available) {}
};

#endif // TRANSITION_H
