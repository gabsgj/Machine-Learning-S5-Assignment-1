#ifndef LPASTAR_PLANNER_HPP
#define LPASTAR_PLANNER_HPP

#include "Planner.hpp"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <chrono>
#include <algorithm>
#include <cmath>

/**
 * @brief 2-tuple key for LPA* priority queue.
 */
struct LPAKey {
    double k1;
    double k2;

    bool operator<(const LPAKey& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) {
            return k1 < other.k1;
        }
        return k2 < other.k2;
    }

    bool operator>(const LPAKey& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) {
            return k1 > other.k1;
        }
        return k2 > other.k2;
    }

    bool operator<=(const LPAKey& other) const {
        return (*this < other) || (std::abs(k1 - other.k1) <= 1e-9 && std::abs(k2 - other.k2) <= 1e-9);
    }
};

/**
 * @brief Lifelong Planning A* (LPA*) Planner implementation with Safe Semantic Optimization.
 */
class LPAStarPlanner : public Planner {
public:
    // Multi-objective weighting parameters: Score = alpha*G - beta*C + gamma*D + delta*R
    double alpha; // Goal completion weight
    double beta;  // Transition cost weight
    double gamma; // Safety distance weight
    double delta; // Reliability weight
    double safetyRadius; // Desired clearance radius around bad states

    LPAStarPlanner(double a = 100.0, double b = 1.0, double g = 5.0, double d = 2.0, double sRadius = 3.0);
    virtual ~LPAStarPlanner() override = default;

    virtual PlanningResult plan(const PlanningProblem& problem) override;

    // Incremental dynamic environment update methods
    void initProblem(const PlanningProblem& problem);
    PlanningResult computeShortestPath();
    void updateTransition(uint64_t transitionId, bool available, double newCost = -1.0);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void updateBadStates(const std::vector<uint64_t>& bads);
    void updateGoal(uint64_t newGoal);

    // Diagnostics & state accessors
    double getGValue(uint64_t stateId) const;
    double getRhsValue(uint64_t stateId) const;
    LPAKey calculateKey(uint64_t stateId) const;
    uint64_t getExpandedCount() const { return exploredStatesCount; }

private:
    PlanningProblem currentProblem;
    std::unordered_map<uint64_t, State> stateMap;
    std::unordered_map<uint64_t, Transition> transitionMap;
    std::unordered_set<uint64_t> badStateSet;

    // Graph adjacency: from -> list of transition IDs, to -> list of incoming transition IDs
    std::unordered_map<uint64_t, std::vector<uint64_t>> outEdges;
    std::unordered_map<uint64_t, std::vector<uint64_t>> inEdges;

    // LPA* vertex estimates
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;

    // Priority queue representation
    struct QueueNode {
        uint64_t id;
        LPAKey key;
        bool operator>(const QueueNode& other) const {
            return key > other.key;
        }
    };
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> openQueue;
    std::unordered_map<uint64_t, LPAKey> openSetKeys; // Tracks active keys in openQueue

    uint64_t exploredStatesCount;
    bool isInitialized;

    // Internal helper routines
    double heuristic(uint64_t fromId, uint64_t toId) const;
    double effectiveTransitionCost(const Transition& t) const;
    void updateVertex(uint64_t u);
    void cleanupQueue();
    PlanningResult extractPath();
};

#endif // LPASTAR_PLANNER_HPP
