#pragma once

#include "core_types.hpp"
#include "problem_loader.hpp"
#include "kd_tree.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <string>
#include <vector>
#include <memory>

namespace safe_semantic_planner {

struct DStarKey {
    double k1 = std::numeric_limits<double>::infinity();
    double k2 = std::numeric_limits<double>::infinity();

    bool operator<(const DStarKey& other) const {
        if (k1 != other.k1 && std::abs(k1 - other.k1) > 1e-9) {
            return k1 < other.k1;
        }
        if (k2 != other.k2 && std::abs(k2 - other.k2) > 1e-9) {
            return k2 < other.k2;
        }
        return false;
    }

    bool operator<=(const DStarKey& other) const {
        if (k1 != other.k1 && std::abs(k1 - other.k1) > 1e-9) {
            return k1 < other.k1;
        }
        if (k2 != other.k2 && std::abs(k2 - other.k2) > 1e-9) {
            return k2 <= other.k2;
        }
        return true;
    }
};

struct QueueEntry {
    std::string stateId;
    DStarKey key;

    // For std::priority_queue (min-heap behavior)
    bool operator>(const QueueEntry& other) const {
        if (key.k1 != other.key.k1 && std::abs(key.k1 - other.key.k1) > 1e-9) {
            return key.k1 > other.key.k1;
        }
        if (key.k2 != other.key.k2 && std::abs(key.k2 - other.key.k2) > 1e-9) {
            return key.k2 > other.key.k2;
        }
        return false;
    }
};

struct MemoryAccounting {
    size_t gRhsTableBytes = 0;
    size_t priorityQueueBytes = 0;
    size_t graphAdjacencyBytes = 0;
    size_t kdTreeBytes = 0;
    size_t totalBytes = 0;
};

class DStarLitePlanner {
public:
    DStarLitePlanner() = default;

    /**
     * @brief Initialize planner with problem and weight parameters.
     */
    void initialize(const PlanningProblem& problem, const WeightParams& params);

    /**
     * @brief Main planning routine. Computes shortest path from initialState to goalState.
     */
    PlanningResult computeShortestPath();

    /**
     * @brief Dynamic update hook: notifies that an edge changed cost or availability.
     */
    void notifyEdgeChanged(const std::string& transitionId, double newCost, bool newAvailability);

    /**
     * @brief Dynamic update hook: notifies that the destination goal state moved.
     */
    void notifyGoalChanged(const std::string& newGoalStateId);

    /**
     * @brief Dynamic update hook: notifies that bad states changed.
     */
    void notifyBadStatesChanged(const std::vector<std::string>& newBadStates);

    /**
     * @brief Dynamic update hook: modifies the safety exclusion radius r dynamically.
     */
    void notifySafetyRadiusChanged(double newRadius);

    /**
     * @brief Extracts current optimal path based on g and rhs values.
     */
    PlanningResult extractPath() const;

    /**
     * @brief Get comprehensive runtime and memory metrics.
     */
    PlannerMetrics getMetrics() const;

    // Instrumentation metrics
    size_t getExpansionCount() const { return currentCallExpansions_; }
    void resetExpansionCount() { currentCallExpansions_ = 0; }

    MemoryAccounting getAnalyticalMemoryAccounting() const;

    const State* getState(const std::string& id) const;
    const Transition* getTransition(const std::string& id) const;
    double getG(const std::string& id) const;
    double getRHS(const std::string& id) const;

    // Lifetime counters reset if needed
    void resetLifetimeStats() {
        goalSuccessCount_ = 0;
        attemptCount_ = 0;
    }

private:
    struct InternalEdge {
        std::string transitionId;
        std::string from;
        std::string to;
        double cost = 1.0;
        double reliability = 1.0;
        double safetyScore = 1.0;
        bool available = true;
        double weight = 1.0;
    };

    DStarKey calculateKey(const std::string& stateId) const;
    double computeHeuristic(const std::string& stateId, const std::string& targetGoalId) const;
    double computeEdgeWeight(double cost, double reliability) const;

    void updateVertex(const std::string& u);
    void updateSafetyExclusions();
    void rebuildGraphStructures();

    PlanningProblem problem_;
    WeightParams params_;
    KdTree kdTree_;

    std::string sInit_; // Fixed D* Lite Goal (search tree root)
    std::string sGoal_; // Movable D* Lite Start (search destination)
    std::string sLastGoal_;

    double km_ = 0.0;
    double cMin_ = 1.0;
    size_t currentCallExpansions_ = 0;

    // Metrics and timers
    bool isInitialSolve_ = true;
    double lastPlanningTimeMs_ = 0.0;
    double lastReplanningTimeMs_ = 0.0;
    size_t goalSuccessCount_ = 0;
    size_t attemptCount_ = 0;

    std::unordered_map<std::string, State> states_;
    std::unordered_map<std::string, InternalEdge> transitions_;

    // Adjacency lists (original graph directions)
    std::unordered_map<std::string, std::vector<std::string>> successors_;   // u -> [v]
    std::unordered_map<std::string, std::vector<std::string>> predecessors_; // v -> [u]
    std::unordered_map<std::string, std::string> edgeLookup_; // "from->to" -> transitionId

    std::unordered_map<std::string, double> g_;
    std::unordered_map<std::string, double> rhs_;
    std::unordered_map<std::string, DStarKey> inOpenList_;

    // Priority queue with min-heap ordering
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> openList_;
};

} // namespace safe_semantic_planner
