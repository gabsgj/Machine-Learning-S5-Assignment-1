#ifndef STATE_HPP
#define STATE_HPP

#include <cstdint>
#include <vector>
#include <cmath>
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>

/**
 * @brief Represents a state embedded in a finite d-dimensional Cartesian space R^d.
 */
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0), embedding({}) {}
    State(uint64_t stateId, const std::vector<double>& emb) : id(stateId), embedding(emb) {}
    State(uint64_t stateId, std::initializer_list<double> emb) : id(stateId), embedding(emb) {}

    /**
     * @brief Computes Euclidean distance to another state.
     */
    double euclideanDistance(const State& other) const {
        if (embedding.size() != other.embedding.size()) {
            // If dimensions differ or one is empty, fallback to 0.0 or handle gracefully
            if (embedding.empty() || other.embedding.empty()) return 0.0;
        }
        double sum = 0.0;
        size_t n = std::min(embedding.size(), other.embedding.size());
        for (size_t i = 0; i < n; ++i) {
            double diff = embedding[i] - other.embedding[i];
            sum += diff * diff;
        }
        return std::sqrt(sum);
    }

    /**
     * @brief Computes Manhattan distance to another state.
     */
    double manhattanDistance(const State& other) const {
        double sum = 0.0;
        size_t n = std::min(embedding.size(), other.embedding.size());
        for (size_t i = 0; i < n; ++i) {
            sum += std::abs(embedding[i] - other.embedding[i]);
        }
        return sum;
    }

    /**
     * @brief Computes Cosine similarity to another state (useful for semantic embeddings).
     */
    double cosineSimilarity(const State& other) const {
        if (embedding.empty() || other.embedding.empty()) return 0.0;
        double dot = 0.0, normA = 0.0, normB = 0.0;
        size_t n = std::min(embedding.size(), other.embedding.size());
        for (size_t i = 0; i < n; ++i) {
            dot += embedding[i] * other.embedding[i];
            normA += embedding[i] * embedding[i];
            normB += other.embedding[i] * other.embedding[i];
        }
        if (normA <= 1e-9 || normB <= 1e-9) return 0.0;
        return dot / (std::sqrt(normA) * std::sqrt(normB));
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << "State(id=" << id << ", [";
        for (size_t i = 0; i < embedding.size(); ++i) {
            oss << embedding[i] << (i + 1 < embedding.size() ? ", " : "");
        }
        oss << "])";
        return oss.str();
    }
};

#endif // STATE_HPP
