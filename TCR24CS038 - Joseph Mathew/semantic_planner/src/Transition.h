#pragma once
#include <cstdint>

struct Transition {
    uint64_t tid;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety; // safety score (higher is safer)
    double reliability;
    bool available;
};
