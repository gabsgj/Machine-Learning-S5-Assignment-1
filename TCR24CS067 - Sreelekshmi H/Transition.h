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

    Transition() {}

    Transition(
        uint64_t id,
        uint64_t from,
        uint64_t to,
        double cost,
        double safety,
        double reliability,
        bool available = true
    ) {
        this->id = id;
        this->from = from;
        this->to = to;
        this->cost = cost;
        this->safety = safety;
        this->reliability = reliability;
        this->available = available;
    }
};

#endif