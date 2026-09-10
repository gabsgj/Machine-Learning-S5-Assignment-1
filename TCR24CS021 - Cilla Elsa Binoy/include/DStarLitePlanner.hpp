#pragma once
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <set>
#include <limits>
#include "Planner.hpp"

// ---------------------------------------------------------------------------
// D* Lite (Koenig & Likhachev, 2002) specialised to this assignment's
// PlanningProblem. See REPORT.md for the full design write-up; comments here
// explain *why* each piece exists so the code doubles as a teaching tool.
//
// KEY IDEA: instead of computing a fresh shortest path every time the
// environment changes (a new bad state appears, an edge goes down, a
// shortcut is added), D* Lite keeps two numbers per state:
//    g(s)   = current best known cost from s to the goal
//    rhs(s) = one-step-lookahead estimate of g(s), i.e.
//             min over edges (s -> s') of [edgeCost(s,s') + g(s')]
// A state is "consistent" when g(s) == rhs(s). Search only ever touches
// states that became inconsistent because of a local change, which is what
// makes replanning fast instead of starting from scratch.
// ---------------------------------------------------------------------------

class DStarLitePlanner : public Planner {
public:
    // costWeight/safetyWeight/marginWeight/heuristicWeight let you reproduce
    // the assignment's Score(P) = aG - bC + gD + dR tradeoff at search time
    // (see edgeCost() in the .cpp for the exact formula).
    DStarLitePlanner(double costWeight = 1.0,
                      double safetyWeight = 1.0,
                      double marginWeight = 0.5,
                      double heuristicWeight = 0.0);

    // One-shot use: load a full problem from scratch and solve it.
    PlanningResult plan(const PlanningProblem& problem) override;

    // ---- Incremental / dynamic-environment API -----------------------
    // These are the operations Test Cases 4-6 and the "Dynamic Environment"
    // section of the assignment ask for. Each does the minimal amount of
    // re-computation needed and returns the freshly extracted path.
    PlanningResult updateGoal(uint64_t newGoal);
    PlanningResult setTransitionAvailability(uint64_t transitionId, bool available);
    PlanningResult addTransition(const Transition& t);
    PlanningResult removeTransition(uint64_t transitionId);
    PlanningResult addBadState(uint64_t stateId);

    // Diagnostics accessors used by main.cpp for the experimental section.
    int totalStatesExpanded() const { return statesExpandedTotal_; }

private:
    struct Key {
        double k1, k2;
        bool operator<(const Key& o) const {
            const double eps = 1e-9;
            if (k1 + eps < o.k1) return true;
            if (o.k1 + eps < k1) return false;
            return k2 + eps < o.k2;
        }
    };

    void loadProblem(const PlanningProblem& problem);
    void initialize();
    Key calculateKey(uint64_t s) const;
    void updateVertex(uint64_t u);
    void computeShortestPath(int& expandedThisCall, size_t& peakQueueThisCall);
    double heuristic(uint64_t s) const;
    double euclidean(uint64_t a, uint64_t b) const;
    double edgeCost(const Transition& t) const;         // infinity if blocked/unavailable
    double distanceToNearestBadState(uint64_t s) const;
    PlanningResult extractPath();
    void rebuildAdjacency();

    static constexpr double INF = std::numeric_limits<double>::infinity();

    // Static-ish problem data
    std::unordered_map<uint64_t, State> states_;
    std::vector<Transition> transitions_;
    std::unordered_map<uint64_t, size_t> transitionIndexById_;
    std::unordered_map<uint64_t, std::vector<size_t>> outEdges_; // from -> transition indices
    std::unordered_map<uint64_t, std::vector<size_t>> inEdges_;  // to   -> transition indices
    std::unordered_set<uint64_t> badStates_;
    uint64_t start_ = 0, goal_ = 0;

    // Search state (persists across incremental updates -- this is the point)
    std::unordered_map<uint64_t, double> g_, rhs_;
    std::set<std::pair<Key, uint64_t>> openSet_;
    std::unordered_map<uint64_t, Key> keyInOpenSet_;

    // Weights for the scalarized edge cost (see edgeCost() in .cpp)
    double costWeight_, safetyWeight_, marginWeight_, heuristicWeight_;

    // Cumulative diagnostics
    int statesExpandedTotal_ = 0;
    bool loaded_ = false;
};
