#include "safety_utils.h"

#include <cmath>
#include <limits>
#include <unordered_map>

namespace safety {

// ═══════════════════════════════════════════════════════════════════════
// Euclidean Distance in R^d
// ═══════════════════════════════════════════════════════════════════════
double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.size() != b.size()) return std::numeric_limits<double>::infinity();
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

// ═══════════════════════════════════════════════════════════════════════
// Minimum Distance to Bad States
// ═══════════════════════════════════════════════════════════════════════
double minDistanceToBadStates(const std::vector<double>& stateEmbedding,
                             const std::vector<std::vector<double>>& badEmbeddings) {
    if (badEmbeddings.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double minDist = std::numeric_limits<double>::infinity();
    for (const auto& badEmb : badEmbeddings) {
        double d = euclideanDistance(stateEmbedding, badEmb);
        if (d < minDist) {
            minDist = d;
        }
    }
    return minDist;
}

// ═══════════════════════════════════════════════════════════════════════
// Compute Safety Map (state_id → min distance to nearest bad state)
// ═══════════════════════════════════════════════════════════════════════
std::unordered_map<uint64_t, double> computeSafetyMap(
    const std::vector<State>& states,
    const std::vector<uint64_t>& badStates) {

    std::unordered_map<uint64_t, double> safetyMap;

    // Collect bad state embeddings
    std::unordered_map<uint64_t, const std::vector<double>*> stateEmbeddings;
    for (const auto& s : states) {
        stateEmbeddings[s.id] = &s.embedding;
    }

    std::vector<std::vector<double>> badEmbeddings;
    badEmbeddings.reserve(badStates.size());
    for (uint64_t bId : badStates) {
        auto it = stateEmbeddings.find(bId);
        if (it != stateEmbeddings.end()) {
            badEmbeddings.push_back(*(it->second));
        }
    }

    // Compute min distance for every state
    for (const auto& s : states) {
        safetyMap[s.id] = minDistanceToBadStates(s.embedding, badEmbeddings);
    }

    return safetyMap;
}

// ═══════════════════════════════════════════════════════════════════════
// Update Safety Map (recomputes from scratch with new bad state list)
// ═══════════════════════════════════════════════════════════════════════
void updateSafetyMap(std::unordered_map<uint64_t, double>& safetyMap,
                     const std::vector<State>& states,
                     const std::vector<uint64_t>& newBadStates) {
    safetyMap = computeSafetyMap(states, newBadStates);
}

} // namespace safety
