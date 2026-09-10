#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <vector>

struct State {
    uint64_t id;
    std::vector<double> embedding;

    State() = default;

    State(uint64_t id, const std::vector<double>& embedding)
        : id(id), embedding(embedding) {}
};

#endif