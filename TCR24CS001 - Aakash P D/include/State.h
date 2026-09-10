#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <vector>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}

    State(uint64_t stateId, const std::vector<double>& coordinates)
        : id(stateId), embedding(coordinates) {}
};

#endif