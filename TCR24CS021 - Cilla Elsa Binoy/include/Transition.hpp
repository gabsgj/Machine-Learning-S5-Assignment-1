#pragma once
#include <cstdint>

// A directed edge (si -> sj) with the four attributes the assignment asks for.
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;        // raw traversal cost, e.g. time/energy/distance
    double safety;       // 0 (unsafe) .. 1 (very safe) intrinsic edge safety
    double reliability;   // 0 (never succeeds) .. 1 (always succeeds)
    bool available;      // can this transition currently be taken?
};
