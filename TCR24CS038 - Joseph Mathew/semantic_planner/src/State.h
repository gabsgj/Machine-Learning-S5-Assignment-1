#pragma once
#include <cstdint>
#include <vector>

struct State {
    uint64_t tid;
    std::vector<double> embedding;
};
