#ifndef DSTAR_LITE_PLANNER_H
#define DSTAR_LITE_PLANNER_H

#include "PlanningProblem.h"
#include "PlanningResult.h"

#include <cstdint>
#include <vector>
#include <queue>
#include <unordered_map>
#include <limits>
#include <functional>

class DStarLitePlanner {

private:

    struct QueueNode {
        double k1;
        double k2;
        uint64_t state;

        bool operator>(
            const QueueNode& other
        ) const {

            if (k1 != other.k1)
                return k1 > other.k1;

            if (k2 != other.k2)
                return k2 > other.k2;

            return state > other.state;
        }
    };

    using PriorityQueue =
        std::priority_queue<
            QueueNode,
            std::vector<QueueNode>,
            std::greater<QueueNode>
        >;

    PlanningProblem problem;

    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;

    std::unordered_map<
        uint64_t,
        std::vector<const Transition*>
    > outgoing;

    std::unordered_map<
        uint64_t,
        std::vector<const Transition*>
    > incoming;

    PriorityQueue open;

    double km;

    uint64_t start;
    uint64_t goal;

    size_t exploredStates;

    static constexpr double INF =
        std::numeric_limits<double>::infinity();

    double heuristic(
        uint64_t stateA,
        uint64_t stateB
    ) const;

    double getG(
        uint64_t state
    ) const;

    double getRHS(
        uint64_t state
    ) const;

    std::pair<double, double> calculateKey(
        uint64_t state
    ) const;

    double transitionCost(
        const Transition& transition
    ) const;

    double safetyPenalty(
        uint64_t state
    ) const;

    void buildGraph();

    void initialize();

    void updateVertex(
        uint64_t state
    );

    void computeShortestPath();

    uint64_t chooseNextState(
        uint64_t current
    ) const;

    PlanningResult constructResult(
        double elapsedTimeMs
    ) const;

    size_t approximateMemoryUsage() const;

public:

    DStarLitePlanner();

    PlanningResult plan(
        const PlanningProblem& problem
    );

    PlanningResult replan();

    void updateGoal(
        uint64_t newGoal
    );

    void updateTransition(
        uint64_t transitionId,
        bool available
    );

    void updateBadStates(
        const std::vector<uint64_t>& badStates
    );

    void addTransition(
        const Transition& transition
    );

    void removeTransition(
        uint64_t transitionId
    );
};

#endif