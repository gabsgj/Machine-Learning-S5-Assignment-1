#ifndef STATE_HPP
#define STATE_HPP

#include <cstdint>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <string>
#include <sstream>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0), embedding({}) {}
    State(uint64_t id_, const std::vector<double>& emb) : id(id_), embedding(emb) {}

    // Euclidean distance between this state and another state in R^d
    double euclideanDistance(const State& other) const {
        if (embedding.size() != other.embedding.size()) {
            size_t minDim = std::min(embedding.size(), other.embedding.size());
            double sumSq = 0.0;
            for (size_t i = 0; i < minDim; ++i) {
                double diff = embedding[i] - other.embedding[i];
                sumSq += diff * diff;
            }
            for (size_t i = minDim; i < embedding.size(); ++i) {
                sumSq += embedding[i] * embedding[i];
            }
            for (size_t i = minDim; i < other.embedding.size(); ++i) {
                sumSq += other.embedding[i] * other.embedding[i];
            }
            return std::sqrt(sumSq);
        }

        double sumSq = 0.0;
        for (size_t i = 0; i < embedding.size(); ++i) {
            double diff = embedding[i] - other.embedding[i];
            sumSq += diff * diff;
        }
        return std::sqrt(sumSq);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "State(" << id << ", [";
        for (size_t i = 0; i < embedding.size(); ++i) {
            oss << embedding[i] << (i + 1 < embedding.size() ? ", " : "");
        }
        oss << "])";
        return oss.str();
    }
};

#endif // STATE_HPP
