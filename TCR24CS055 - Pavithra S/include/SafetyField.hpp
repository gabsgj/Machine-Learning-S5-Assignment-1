#ifndef SAFETY_FIELD_HPP
#define SAFETY_FIELD_HPP

#include "Types.hpp"
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

namespace Safety {

/**
 * Computes Euclidean distance between two embedding vectors in R^d.
 */
inline double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return 0.0;
    size_t dim = std::min(a.size(), b.size());
    double sum = 0.0;
    for (size_t i = 0; i < dim; ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

/**
 * Weights for the multi-objective evaluation score:
 * Score(P) = alpha * G - beta * C + gamma * D + delta * R
 */
struct ObjectiveWeights {
    double alpha = 100.0; // Goal completion weight
    double beta = 1.0;    // Cost minimization weight
    double gamma = 2.0;   // Safety distance maximization weight
    double delta = 5.0;   // Reliability weight
};

/**
 * Parameters controlling the safety barrier repulsive field.
 */
struct SafetyFieldConfig {
    double safetyInfluenceRadius = 5.0; // Distance beyond which obstacle penalty is negligible
    double safetyWeight = 10.0;          // Multiplier for safety repulsive potential
    double barrierEpsilon = 0.1;        // Prevents division by zero
    double reliabilityWeight = 2.0;     // Penalty for unreliable edges
};

/**
 * Calculates the minimum Euclidean distance from state 's' to the nearest bad state.
 */
inline double minDistanceToBadStates(
    const State& state,
    const std::unordered_set<uint64_t>& badSet,
    const std::unordered_map<uint64_t, State>& stateMap
) {
    if (badSet.empty()) return 1000.0; // Safe default when no bad states exist
    if (badSet.find(state.id) != badSet.end()) return 0.0;

    double minDist = std::numeric_limits<double>::infinity();
    for (uint64_t badId : badSet) {
        auto it = stateMap.find(badId);
        if (it != stateMap.end()) {
            double d = euclideanDistance(state.embedding, it->second.embedding);
            if (d < minDist) {
                minDist = d;
            }
        }
    }
    return minDist;
}

/**
 * Computes safety repulsive potential at a given state.
 * States closer to bad states receive a higher repulsive penalty.
 */
inline double computeSafetyPenalty(
    const State& targetState,
    const std::unordered_set<uint64_t>& badSet,
    const std::unordered_map<uint64_t, State>& stateMap,
    const SafetyFieldConfig& config
) {
    if (badSet.find(targetState.id) != badSet.end()) {
        return 1e9; // Forbidden / effectively infinite penalty
    }
    if (badSet.empty()) return 0.0;

    double d = minDistanceToBadStates(targetState, badSet, stateMap);
    if (d >= config.safetyInfluenceRadius) {
        return 0.0;
    }
    // Smooth inverse distance repulsive potential
    return config.safetyWeight * std::pow((config.safetyInfluenceRadius - d) / (d + config.barrierEpsilon), 2.0);
}

/**
 * Computes the combined effective cost for traversing a transition.
 * c_eff(u, v) = cost + safety_penalty(v) + reliability_penalty(transition)
 */
inline double computeEffectiveTransitionCost(
    const Transition& trans,
    const State& targetState,
    const std::unordered_set<uint64_t>& badSet,
    const std::unordered_map<uint64_t, State>& stateMap,
    const SafetyFieldConfig& config
) {
    if (!trans.available || badSet.find(trans.to) != badSet.end()) {
        return std::numeric_limits<double>::infinity();
    }

    double safetyPenalty = computeSafetyPenalty(targetState, badSet, stateMap, config);
    
    // Reliability penalty: -ln(reliability)
    double r = std::max(0.0001, std::min(1.0, trans.reliability));
    double relPenalty = -config.reliabilityWeight * std::log(r);

    return trans.cost + safetyPenalty + relPenalty;
}

/**
 * Computes the composite Multi-Objective Score of a path.
 */
inline double evaluateMultiObjectiveScore(
    bool goalReached,
    double totalCost,
    double minSafetyDist,
    double cumulativeReliability,
    const ObjectiveWeights& weights
) {
    double G = goalReached ? 1.0 : 0.0;
    return (weights.alpha * G) - (weights.beta * totalCost) + 
           (weights.gamma * minSafetyDist) + (weights.delta * cumulativeReliability);
}

} // namespace Safety

#endif // SAFETY_FIELD_HPP
