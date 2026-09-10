#include "../include/safety.h"
#include <cmath>
#include <limits>

double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size(); i++) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double distanceToNearestBadState(
    const State& s,
    const std::vector<uint64_t>& badStateIds,
    const std::vector<State>& allStates
) {
    double minDistance = std::numeric_limits<double>::infinity();

    for (uint64_t badId : badStateIds) {
        for (const State& candidate : allStates) {
            if (candidate.id == badId) {
                double d = euclideanDistance(s.embedding, candidate.embedding);
                if (d < minDistance) {
                    minDistance = d;
                }
                break;
            }
        }
    }

    return minDistance;
}