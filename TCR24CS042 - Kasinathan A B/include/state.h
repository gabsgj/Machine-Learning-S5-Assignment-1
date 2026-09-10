#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <vector>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;
};

#endif