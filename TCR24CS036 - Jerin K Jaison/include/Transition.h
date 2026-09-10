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

    Transition();

    Transition(
        uint64_t id,
        uint64_t from,
        uint64_t to,
        double cost,
        double safety,
        double reliability,
        bool available = true
    );
};

#endif