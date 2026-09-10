#pragma once

#include "types.hpp"
#include <cmath>
#include <vector>
#include <unordered_map>
#include <algorithm>

namespace SafePlanner {

/**
 * @brief Euclidean geometry and Cartesian distance calculations in R^d.
 */
class Geometry {
public:
    /**
     * @brief Computes Euclidean distance between two d-dimensional vectors.
     */
    static double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
        if (a.empty() || b.empty()) return 0.0;
        size_t dim = std::min(a.size(), b.size());
        double sumSq = 0.0;
        for (size_t i = 0; i < dim; ++i) {
            double diff = a[i] - b[i];
            sumSq += diff * diff;
        }
        return std::sqrt(sumSq);
    }

    /**
     * @brief Calculates the minimum Euclidean distance from a state to any bad state in B.
     * If badStates is empty, returns +infinity.
     */
    static double distanceToBadStates(
        const std::vector<double>& stateEmb,
        const std::vector<uint64_t>& badStateIds,
        const std::unordered_map<uint64_t, std::vector<double>>& stateEmbeddings) 
    {
        if (badStateIds.empty()) return INF;
        
        double minD = INF;
        for (uint64_t badId : badStateIds) {
            auto it = stateEmbeddings.find(badId);
            if (it != stateEmbeddings.end()) {
                double dist = euclideanDistance(stateEmb, it->second);
                if (dist < minD) {
                    minD = dist;
                }
            }
        }
        return minD;
    }

    /**
     * @brief Computes the minimum Euclidean distance from all visited states on a path
     * to the nearest bad state (Objective 4).
     */
    static double computePathSafetyScore(
        const std::vector<uint64_t>& path,
        const std::vector<uint64_t>& badStateIds,
        const std::unordered_map<uint64_t, std::vector<double>>& stateEmbeddings)
    {
        if (path.empty() || badStateIds.empty()) return INF;

        double minSafety = INF;
        for (uint64_t stateId : path) {
            auto it = stateEmbeddings.find(stateId);
            if (it != stateEmbeddings.end()) {
                double d = distanceToBadStates(it->second, badStateIds, stateEmbeddings);
                if (d < minSafety) {
                    minSafety = d;
                }
            }
        }
        return minSafety;
    }

    /**
     * @brief Computes the average safety distance along a path.
     */
    static double computeAverageSafetyDistance(
        const std::vector<uint64_t>& path,
        const std::vector<uint64_t>& badStateIds,
        const std::unordered_map<uint64_t, std::vector<double>>& stateEmbeddings)
    {
        if (path.empty() || badStateIds.empty()) return INF;

        double sumDist = 0.0;
        size_t count = 0;
        for (uint64_t stateId : path) {
            auto it = stateEmbeddings.find(stateId);
            if (it != stateEmbeddings.end()) {
                double d = distanceToBadStates(it->second, badStateIds, stateEmbeddings);
                if (d < INF) {
                    sumDist += d;
                    count++;
                }
            }
        }
        return (count > 0) ? (sumDist / count) : INF;
    }
};

} // namespace SafePlanner
