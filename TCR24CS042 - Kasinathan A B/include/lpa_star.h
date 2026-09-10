#ifndef LPA_STAR_H
#define LPA_STAR_H

#include <unordered_map>
#include <vector>
#include <utility>
#include "planner.h"
#include "graph.h"

class LPAStarPlanner : public Planner {
public:
    PlanningResult plan(const PlanningProblem& problem) override;

    int statesExpanded = 0;

    // Replanning entry points (Module 6)
    PlanningResult onTransitionUnavailable(uint64_t transitionId, uint64_t fromState, uint64_t toState);
    PlanningResult onTransitionAdded(const Transition& t);
    PlanningResult onBadStatesChanged(const std::vector<uint64_t>& newBadStates);
    PlanningResult onGoalChanged(uint64_t newGoal);

private:
    struct QueueEntry {
        double k1, k2;
        uint64_t stateId;
    };

    PlanningProblem problem;
    Graph* graphPtr = nullptr;
    std::unordered_map<uint64_t, State> stateLookup;
    std::unordered_map<uint64_t, std::vector<Transition>> incoming;
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;
    std::vector<QueueEntry> queue;

    void initialize();
    std::pair<double,double> calculateKey(uint64_t stateId);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    bool isBadState(uint64_t stateId);
    double effectiveCost(const Transition& t);
    void pushQueue(uint64_t stateId);
    void removeFromQueue(uint64_t stateId);
    PlanningResult extractResult();
};

#endif