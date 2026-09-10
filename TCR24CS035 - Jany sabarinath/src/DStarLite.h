#ifndef DSTAR_LITE_H
#define DSTAR_LITE_H

#include "Planner.h"

#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class DStarLite : public Planner {
private:

    struct Key {
        double first;
        double second;

        bool operator<(const Key& other) const {
            if (first != other.first)
                return first < other.first;

            return second < other.second;
        }
    };

    struct QueueNode {
        uint64_t state;
        Key key;

        bool operator>(const QueueNode& other) const {
            if (key.first != other.key.first)
                return key.first > other.key.first;

            return key.second > other.key.second;
        }
    };

    using PriorityQueue =
        std::priority_queue<
            QueueNode,
            std::vector<QueueNode>,
            std::greater<QueueNode>
        >;

    PlanningProblem problem;

    uint64_t start;
    uint64_t goal;

    double km;

    double safetyWeight;

    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;

    std::unordered_map<uint64_t, std::vector<Transition>> outgoing;
    std::unordered_map<uint64_t, std::vector<Transition>> incoming;

    std::unordered_set<uint64_t> badStateSet;

    PriorityQueue openList;

private:

    double INF() const;

    double heuristic(
        uint64_t a,
        uint64_t b
    ) const;

    double distanceToNearestBadState(
        uint64_t stateId
    ) const;

    double edgeCost(
        const Transition& transition
    ) const;

    bool isBadState(
        uint64_t stateId
    ) const;

    bool stateExists(
        uint64_t stateId
    ) const;

    Key calculateKey(
        uint64_t stateId
    ) const;

    double getG(
        uint64_t stateId
    ) const;

    double getRHS(
        uint64_t stateId
    ) const;

    void setG(
        uint64_t stateId,
        double value
    );

    void setRHS(
        uint64_t stateId,
        double value
    );

    void initialize();

    void updateVertex(
        uint64_t stateId
    );

    void computeShortestPath();

    std::vector<uint64_t> reconstructPath(
        int& exploredStates
    );

    uint64_t getBestSuccessor(
        uint64_t stateId
    ) const;

public:

    explicit DStarLite(
        double safetyWeight = 2.0
    );

    PlanningResult plan(
        const PlanningProblem& problem
    ) override;

    void updateTransition(
        uint64_t transitionId,
        bool available
    );

    void addTransition(
        const Transition& transition
    );

    void removeTransition(
        uint64_t transitionId
    );

    void updateGoal(
        uint64_t newGoal
    );

    void updateBadStates(
        const std::vector<uint64_t>& newBadStates
    );

    PlanningResult replan();
};

#endif