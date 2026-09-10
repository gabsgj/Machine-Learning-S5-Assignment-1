#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <vector>

/// @brief Represents a state in the Cartesian state space R^d.
/// Each state has a unique identifier and a d-dimensional embedding vector.
class State {
public:
    uint64_t id;                       ///< Unique state identifier
    std::vector<double> embedding;     ///< Coordinate vector (x_1, x_2, ..., x_d)

    State() : id(0) {}

    State(uint64_t id, std::vector<double> embedding)
        : id(id), embedding(std::move(embedding)) {}
};

#endif // STATE_H
