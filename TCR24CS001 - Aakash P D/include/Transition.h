#ifndef TRANSITION_H
#define TRANSITION_H

#include <cstdint>

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
        : id(0),
          from(0),
          to(0),
          cost(0.0),
          safety(0.0),
          reliability(0.0),
          available(true) {}

    Transition(
        uint64_t transitionId,
        uint64_t fromState,
        uint64_t toState,
        double transitionCost,
        double safetyScore,
        double reliabilityScore,
        bool isAvailable = true
    )
        : id(transitionId),
          from(fromState),
          to(toState),
          cost(transitionCost),
          safety(safetyScore),
          reliability(reliabilityScore),
          available(isAvailable) {}
};

#endif