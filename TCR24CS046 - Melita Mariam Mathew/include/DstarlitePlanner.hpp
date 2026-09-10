#ifndef DSTARLITE_PLANNER_HPP
#define DSTARLITE_PLANNER_HPP

#include "Planner.hpp"
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <chrono>
#include <algorithm>
#include <cmath>

struct DStarKey {
    double k1;
    double k2;

    bool operator<(const DStarKey& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) return k1 < other.k1;
        return k2 < other.k2;
    }

    bool operator>(const DStarKey& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) return k1 > other.k1;
        return k2 > other.k2;
    }

    bool operator<=(const DStarKey& other) const {
        return (*this < other) || (std::abs(k1 - other.k1) <= 1e-9 && std::abs(k2 - other.k2) <= 1e-9);
    }
};

/**
 * @brief D* Lite (Dynamic Heuristic Search) Planner with Safe Multi-Objective Evaluation.
 */
class DstarlitePlanner : public Planner {
public:
    double alpha;
    double beta;
    double gamma;
    double delta;
    double safetyRadius;

    DstarlitePlanner(double a = 100.0, double b = 1.0, double gVal = 5.0, double d = 2.0, double sRadius = 3.0);
    virtual ~DstarlitePlanner() override = default;

    virtual PlanningResult plan(const PlanningProblem& problem) override;

    void initProblem(const PlanningProblem& problem);
    PlanningResult computeShortestPath();
    void updateTransition(uint64_t transitionId, bool available, double newCost = -1.0);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void updateBadStates(const std::vector<uint64_t>& bads);
    void updateGoal(uint64_t newGoal);
    void updateStart(uint64_t newStart);

    uint64_t getExpandedCount() const { return exploredStatesCount; }

private:
    PlanningProblem currentProblem;
    std::unordered_map<uint64_t, State> stateMap;
    std::unordered_map<uint64_t, Transition> transitionMap;
    std::unordered_set<uint64_t> badStateSet;

    std::unordered_map<uint64_t, std::vector<uint64_t>> outEdges;
    std::unordered_map<uint64_t, std::vector<uint64_t>> inEdges;

    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;

    struct QueueNode {
        uint64_t id;
        DStarKey key;
        bool operator>(const QueueNode& other) const {
            return key > other.key;
        }
    };
    std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> openQueue;
    std::unordered_map<uint64_t, DStarKey> openSetKeys;

    double km;
    uint64_t lastStart;
    uint64_t exploredStatesCount;
    bool isInitialized;

    double heuristic(uint64_t aId, uint64_t bId) const;
    double effectiveCost(const Transition& t) const;
    DStarKey calculateKey(uint64_t stateId) const;
    double getG(uint64_t stateId) const;
    double getRhs(uint64_t stateId) const;
    void updateVertex(uint64_t u);
    PlanningResult extractPath();
};

#endif // DSTARLITE_PLANNER_HPP
