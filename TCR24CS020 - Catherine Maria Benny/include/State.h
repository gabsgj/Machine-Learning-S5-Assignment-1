#pragma once
#include <cstdint>
#include <vector>

// A state embedded in a finite Cartesian space R^d.
// id: unique identifier for the state.
// embedding: the Cartesian coordinates (x1, x2, ..., xd) of the state.
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}
    State(uint64_t id_, std::vector<double> embedding_)
        : id(id_), embedding(std::move(embedding_)) {}
};
