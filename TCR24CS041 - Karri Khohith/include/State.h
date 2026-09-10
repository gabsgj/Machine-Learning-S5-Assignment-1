#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <vector>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State();

    State(
        uint64_t id,
        const std::vector<double>& embedding
    );
};

#endif