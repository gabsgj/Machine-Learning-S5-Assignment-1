#ifndef SAFE_PLANNER_STATE_H
#define SAFE_PLANNER_STATE_H

#include <cstdint>
#include <vector>
#include <cmath>
#include <stdexcept>

namespace planner {

// A single point s_i = (x_1, ..., x_d) in the finite Cartesian state space R^d.
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0) {}
    State(uint64_t id_, std::vector<double> embedding_)
        : id(id_), embedding(std::move(embedding_)) {}

    size_t dimension() const { return embedding.size(); }

    // Euclidean distance between this state and another. Used both as the
    // admissible heuristic (straight-line lower bound on transition cost)
    // and as the basis for the safety-margin computation (distance to the
    // nearest bad state).
    double distanceTo(const State& other) const {
        if (embedding.size() != other.embedding.size()) {
            throw std::invalid_argument("State dimensionality mismatch");
        }
        double sumSq = 0.0;
        for (size_t i = 0; i < embedding.size(); ++i) {
            double d = embedding[i] - other.embedding[i];
            sumSq += d * d;
        }
        return std::sqrt(sumSq);
    }
};

} // namespace planner

#endif // SAFE_PLANNER_STATE_H
