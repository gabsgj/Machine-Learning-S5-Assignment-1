#ifndef D_STAR_LITE_PLANNER_HPP
#define D_STAR_LITE_PLANNER_HPP

#include "Planner.hpp"
#include "LPAStarPlanner.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <limits>
#include <chrono>
#include <cmath>

class DStarLitePlanner : public Planner {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    DStarLitePlanner(PlannerWeights weights = PlannerWeights());
    ~DStarLitePlanner() override = default;

    PlanningResult plan(const PlanningProblem& problem) override;

    // Dynamic replanning methods (D* Lite backwards search)
    void updateStart(uint64_t newStartId);
    void updateTransition(uint64_t transitionId, double newCost, double newSafety, double newReliability, bool newAvailable);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    PlanningResult replan();

    size_t getExpandedCount() const { return expandedCount_; }

private:
    PlannerWeights weights_;
    PlanningProblem currentProblem_;
    uint64_t startId_;
    uint64_t goalId_;
    uint64_t lastStartId_;
    double km_;

    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitions_;
    std::unordered_set<uint64_t> badStatesSet_;

    std::unordered_map<uint64_t, std::vector<uint64_t>> succTransitions_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> predTransitions_;

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::unordered_map<uint64_t, double> badDistanceCache_;

    struct QueueElement {
        Key key;
        uint64_t stateId;

        bool operator>(const QueueElement& other) const {
            return key > other.key;
        }
    };

    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> openQueue_;
    std::unordered_map<uint64_t, Key> openSetKeys_;

    size_t expandedCount_;

    void initialize();
    void computeBadDistances();
    double computeTransitionWeight(const Transition& t) const;
    double heuristic(uint64_t u, uint64_t v) const;
    Key calculateKey(uint64_t stateId) const;
    void updateVertex(uint64_t stateId);
    void computeShortestPath();
    PlanningResult extractPath();

    double getG(uint64_t u) const {
        auto it = g_.find(u);
        return it != g_.end() ? it->second : INF;
    }

    double getRhs(uint64_t u) const {
        auto it = rhs_.find(u);
        return it != rhs_.end() ? it->second : INF;
    }

    double getDistanceToBad(uint64_t stateId) const;
};

#endif // D_STAR_LITE_PLANNER_HPP
