#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include <unordered_map>
#include <vector>
#include <algorithm>
#include <cmath>

namespace SafePlanner {

enum class HeuristicType {
    ADMISSIBLE_EUCLIDEAN, // Scaled Euclidean distance guaranteed to never overestimate
    EUCLIDEAN_RAW,        // Raw Euclidean distance
    MANHATTAN,            // L1 norm scaled
    ZERO_DIJKSTRA,        // Uninformed Dijkstra search
    SAFETY_BIASED         // Potential field heuristic pushing away from bad states
};

class Heuristic {
private:
    HeuristicType type_;
    double minCostRatio_; // c_min_ratio = min(cost / distance) over all valid edges
    std::unordered_map<uint64_t, std::vector<double>> embeddings_;
    std::vector<uint64_t> badStates_;
    double safetyWeight_;

public:
    Heuristic(HeuristicType type = HeuristicType::ADMISSIBLE_EUCLIDEAN, double safetyWeight = 0.0)
        : type_(type), minCostRatio_(1.0), safetyWeight_(safetyWeight) {}

    /**
     * @brief Calibrates the heuristic scale factor c_min_ratio from the problem's transitions
     * to strictly preserve admissibility and consistency.
     */
    void calibrate(const std::vector<State>& states,
                   const std::vector<Transition>& transitions,
                   const std::vector<uint64_t>& badStates) 
    {
        embeddings_.clear();
        for (const auto& s : states) {
            embeddings_[s.id] = s.embedding;
        }
        badStates_ = badStates;

        if (type_ == HeuristicType::ZERO_DIJKSTRA) {
            minCostRatio_ = 0.0;
            return;
        }

        double minRatio = INF;
        for (const auto& t : transitions) {
            if (!t.available || t.cost <= 0.0) continue;
            auto itFrom = embeddings_.find(t.from);
            auto itTo = embeddings_.find(t.to);
            if (itFrom != embeddings_.end() && itTo != embeddings_.end()) {
                double dist = Geometry::euclideanDistance(itFrom->second, itTo->second);
                if (dist > EPSILON) {
                    double ratio = t.cost / dist;
                    if (ratio < minRatio) {
                        minRatio = ratio;
                    }
                }
            }
        }

        if (minRatio < INF && minRatio > 0.0) {
            minCostRatio_ = minRatio;
        } else {
            // Fallback: If no valid transitions or all zero-distance, use conservative 1.0 or 0.0
            minCostRatio_ = 0.0;
        }
    }

    /**
     * @brief Computes h(u, goal) from state u to the goal state.
     */
    double compute(uint64_t u, uint64_t goal) const {
        if (u == goal) return 0.0;
        if (type_ == HeuristicType::ZERO_DIJKSTRA || minCostRatio_ <= 0.0) {
            return 0.0;
        }

        auto itU = embeddings_.find(u);
        auto itG = embeddings_.find(goal);
        if (itU == embeddings_.end() || itG == embeddings_.end()) {
            return 0.0;
        }

        double baseDist = 0.0;
        if (type_ == HeuristicType::MANHATTAN) {
            const auto& a = itU->second;
            const auto& b = itG->second;
            size_t dim = std::min(a.size(), b.size());
            for (size_t i = 0; i < dim; ++i) {
                baseDist += std::abs(a[i] - b[i]);
            }
        } else {
            baseDist = Geometry::euclideanDistance(itU->second, itG->second);
        }

        double hValue = (type_ == HeuristicType::EUCLIDEAN_RAW) ? baseDist : (minCostRatio_ * baseDist);

        // Safety-biased potential field bonus/penalty (if enabled)
        if (type_ == HeuristicType::SAFETY_BIASED && safetyWeight_ > 0.0 && !badStates_.empty()) {
            double badDist = Geometry::distanceToBadStates(itU->second, badStates_, embeddings_);
            if (badDist < INF && badDist > EPSILON) {
                // Artificial potential field repulsive cost: 1 / badDist
                hValue += safetyWeight_ / badDist;
            }
        }

        return hValue;
    }

    void setType(HeuristicType t) { type_ = t; }
    HeuristicType getType() const { return type_; }
    double getMinCostRatio() const { return minCostRatio_; }
    void setMinCostRatio(double r) { minCostRatio_ = r; }
};

} // namespace SafePlanner
