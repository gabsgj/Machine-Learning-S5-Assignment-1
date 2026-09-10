#pragma once
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <limits>
#include <set>
#include "Planner.hpp"

// D* Lite planner extended with a safety- and reliability-aware edge cost,
// and support for incremental replanning after:
//   - transition availability changes (edges appearing/disappearing)
//   - new bad states being added
//   - the start state moving
//   - the goal state changing (handled via a bounded reinitialization)
//
// Search direction follows the classical D* Lite formulation: g/rhs values
// are anchored at the goal and the search fans out towards the start, which
// is what makes replanning after the *start* moves (or edges near the start
// change) cheap. See docs/DesignReport.md for the full derivation.
class DStarLitePlanner : public Planner {
public:
    struct Weights {
        double safety = 4.0;       // penalty weight for (1 - transition.safety)
        double reliability = 2.0;  // penalty weight for (1 - transition.reliability)
        double proximity = 3.0;    // penalty weight for closeness to bad states
        double proximityEps = 0.5; // avoids division blow-up right at a bad state
    };

    DStarLitePlanner();
    explicit DStarLitePlanner(Weights weights);

    // One-shot solve matching the Planner interface required by the brief.
    PlanningResult plan(const PlanningProblem& problem) override;

    // ---- Incremental / dynamic-environment API (bonus scope) ----
    // Load a problem once, then call these as the environment changes and
    // call replan() to get an updated PlanningResult without rebuilding the
    // whole search tree from scratch.
    void initialize(const PlanningProblem& problem);
    void setEdgeAvailability(uint64_t transitionId, bool available);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void addBadState(uint64_t stateId);
    void removeBadState(uint64_t stateId);
    void updateStart(uint64_t newStartId);
    void updateGoal(uint64_t newGoalId); // bounded reinitialization, see report
    PlanningResult replan();

    uint64_t lastExploredCount() const { return exploredCount_; }

private:
    struct Key {
        double k1, k2;
        bool operator<(const Key& other) const {
            if (k1 != other.k1) return k1 < other.k1;
            return k2 < other.k2;
        }
    };

    static constexpr double INF = std::numeric_limits<double>::infinity();

    Weights weights_;

    // Graph storage
    std::unordered_map<uint64_t, State> stateOf_;
    std::unordered_map<uint64_t, Transition> transitionOf_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> outEdges_; // stateId -> transition ids
    std::unordered_map<uint64_t, std::vector<uint64_t>> inEdges_;  // stateId -> transition ids
    std::unordered_set<uint64_t> badStates_;

    uint64_t start_ = 0, goal_ = 0;
    double km_ = 0.0;
    double minCostRate_ = 0.0; // lower bound on cost per unit Euclidean distance

    std::unordered_map<uint64_t, double> g_, rhs_;
    std::set<std::pair<Key, uint64_t>> openSet_;
    std::unordered_map<uint64_t, Key> openKeyOf_;

    uint64_t exploredCount_ = 0;
    uint64_t lastStartForHeuristic_ = 0;

    double edgeCost(const Transition& t) const;
    double heuristic(uint64_t s) const;
    Key calculateKey(uint64_t s) const;
    void updateVertex(uint64_t u);
    void computeShortestPath();
    void insertOrUpdateOpen(uint64_t s, const Key& k);
    void removeFromOpen(uint64_t s);
    double gOf(uint64_t s) const;
    double rhsOf(uint64_t s) const;
    void recomputeMinCostRate();
    PlanningResult extractPath();
};
