#ifndef STATE_H
#define STATE_H

#include <cstdint>
#include <utility>
#include <vector>

struct State {
    uint64_t id{};
    std::vector<double> embedding;

    State() = default;
    State(uint64_t id_, std::vector<double> embedding_)
        : id(id_), embedding(std::move(embedding_)) {}
};

#endif
