#ifndef SAFETY_UTILS_H
#define SAFETY_UTILS_H

#include <cstdint>
#include <unordered_map>
#include <vector>
#include "state.h"

namespace safety {

/// @brief Compute Euclidean distance between two d-dimensional embedding vectors.
/// @param a First embedding vector.
/// @param b Second embedding vector.
/// @return Euclidean distance ||a - b||_2.
double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b);

/// @brief Compute the minimum Euclidean distance from a state's embedding
///        to the nearest bad state embedding.
/// @param stateEmbedding The embedding of the query state.
/// @param badEmbeddings  Embeddings of all bad states.
/// @return Minimum distance, or +infinity if badEmbeddings is empty.
double minDistanceToBadStates(const std::vector<double>& stateEmbedding,
                             const std::vector<std::vector<double>>& badEmbeddings);

/// @brief Precompute a safety map: state_id → min Euclidean distance to nearest bad state.
/// @param states     All states in the problem.
/// @param badStates  IDs of bad states.
/// @return Map from state ID to its minimum distance to the closest bad state.
std::unordered_map<uint64_t, double> computeSafetyMap(
    const std::vector<State>& states,
    const std::vector<uint64_t>& badStates);

/// @brief Update the safety map incrementally when bad states change.
///        Recomputes distances only for states whose safety may have changed.
/// @param safetyMap     Existing safety map to update in-place.
/// @param states        All states.
/// @param newBadStates  The updated bad state list.
void updateSafetyMap(std::unordered_map<uint64_t, double>& safetyMap,
                     const std::vector<State>& states,
                     const std::vector<uint64_t>& newBadStates);

} // namespace safety

#endif // SAFETY_UTILS_H
