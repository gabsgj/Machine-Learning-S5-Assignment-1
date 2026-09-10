#ifndef LPA_STAR_PLANNER_HPP
#define LPA_STAR_PLANNER_HPP

#include "Planner.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <limits>
#include <chrono>
#include <cmath>

struct Key {
    double k1;
    double k2;

    bool operator<(const Key& other) const {
        if (std::abs(k1 - other.k1) > 1e-9) {
            return k1 < other.k1;
        }
        return k2 < other.k2;
    }

    bool operator<=(const Key& other) const {
        return (*this < other) || (!(*this < other) && !(other < *this));
    }

    bool operator>(const Key& other) const {
        return other < *this;
    }

    bool operator>=(const Key& other) const {
        return !(*this < other);
    }
};

struct PlannerWeights {
    double alpha; // Goal completion weight
    double beta;  // Cumulative cost weight
    double gamma; // Safety distance penalty weight
    double delta; // Reliability weight
    double safeDistanceThreshold; // Strict avoidance threshold
    double heuristicScale; // Heuristic scale factor

    PlannerWeights()
        : alpha(100.0), beta(1.0), gamma(5.0), delta(2.0),
          safeDistanceThreshold(0.05), heuristicScale(1.0) {}
};

class LPAStarPlanner : public Planner {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    LPAStarPlanner(PlannerWeights weights = PlannerWeights());
    ~LPAStarPlanner() override = default;

    // Standard Planner Interface
    PlanningResult plan(const PlanningProblem& problem) override;

    // Dynamic replanning methods
    void updateTransition(uint64_t transitionId, double newCost, double newSafety, double newReliability, bool newAvailable);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void updateBadStates(const std::vector<uint64_t>& newBadStates);
    void updateGoal(uint64_t newGoalId);
    void updateStart(uint64_t newStartId);
    PlanningResult replan();

    // Configuration & Inspection
    void setWeights(const PlannerWeights& weights) { weights_ = weights; }
    const PlannerWeights& getWeights() const { return weights_; }
    double getDistanceToBad(uint64_t stateId) const;
    size_t getExpandedCount() const { return expandedCount_; }
    void resetExpandedCount() { expandedCount_ = 0; }

private:
    PlannerWeights weights_;
    PlanningProblem currentProblem_;
    uint64_t startId_;
    uint64_t goalId_;

    // Graph storage
    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitions_;
    std::unordered_set<uint64_t> badStatesSet_;

    // Adjacency: stateId -> list of outgoing transition IDs
    std::unordered_map<uint64_t, std::vector<uint64_t>> succTransitions_;
    // Predecessors: stateId -> list of incoming transition IDs
    std::unordered_map<uint64_t, std::vector<uint64_t>> predTransitions_;

    // LPA* vertex properties
    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::unordered_map<uint64_t, double> badDistanceCache_;

    // Priority Queue for Open Set
    // We use a custom indexed min-heap or priority queue with lazy deletion / locator
    struct QueueElement {
        Key key;
        uint64_t stateId;

        bool operator>(const QueueElement& other) const {
            return key > other.key;
        }
    };

    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> openQueue_;
    std::unordered_map<uint64_t, Key> openSetKeys_; // Tracks presence and current key in Open Set

    size_t expandedCount_;

    // Helper functions
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
};

#endif // LPA_STAR_PLANNER_HPP
