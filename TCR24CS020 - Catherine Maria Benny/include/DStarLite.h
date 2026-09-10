#pragma once
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Metrics.h"
#include "Planner.h"
#include "PlanningProblem.h"
#include "PlanningResult.h"
#include "State.h"
#include "Transition.h"

// A genuine D* Lite implementation (Koenig & Likhachev, 2002) specialized for
// the "safe semantic planner" problem: a directed, weighted Cartesian graph
// in which bad states are a HARD constraint (never appear in a returned
// path) rather than a soft penalty.
//
// Conventions (matching the standard D* Lite formulation):
//   - The search propagates BACKWARD from the goal. g(s) and rhs(s)
//     approximate the cost of the shortest path from s to the goal.
//   - rhs(goal) = 0 by definition; rhs(u) = min over successors v of
//     c(u,v) + g(v) for all u != goal.
//   - A vertex is LOCALLY CONSISTENT when g(u) == rhs(u). The open queue
//     only ever holds locally INconsistent vertices.
//   - CalculateKey(s) = [ min(g,rhs) + h(s, start) + km ; min(g,rhs) ].
//     h(s, start) focuses the search around the (fixed, in this project)
//     start state. km accumulates h(old_start, new_start) if the start
//     ever moves (supported via moveStart(), unused by the six required
//     test cases since the agent's start does not move between events).
//   - Edge cost c(u,v) is +inf whenever the transition is unavailable OR v
//     is a bad state (unless v is the goal, which is validated to never be
//     bad). This is what makes bad-state avoidance a hard constraint: no
//     amount of UpdateVertex propagation can ever make it "cheap enough" to
//     route through a bad state, because its incoming edge cost is
//     literally infinite, not merely large.
//
// Dynamic updates (setTransitionAvailability, addTransition, removeTransition,
// addBadState, removeBadState, updateGoal) only call UpdateVertex on the
// vertices whose rhs is directly affected, then rely on ComputeShortestPath's
// incremental propagation to fix up exactly the affected region of the
// search tree -- the whole planner is NOT rebuilt from scratch.
class DStarLite : public Planner {
public:
    static constexpr double INF = std::numeric_limits<double>::infinity();

    DStarLite();

    // Planner interface: (re)initializes the planner from scratch for a new
    // problem instance and computes an initial plan.
    PlanningResult plan(const PlanningProblem& problem) override;

    // --- Dynamic update API -------------------------------------------------
    // Each of these performs the minimal incremental UpdateVertex work and
    // then calls ComputeShortestPath() + ReconstructPath() to produce a new
    // PlanningResult, WITHOUT reinitializing g/rhs for the rest of the graph.
    PlanningResult setTransitionAvailability(uint64_t transitionId, bool available);
    PlanningResult addTransition(const Transition& t);
    PlanningResult removeTransition(uint64_t transitionId); // marks unavailable
    PlanningResult addBadState(uint64_t stateId);
    PlanningResult removeBadState(uint64_t stateId);
    PlanningResult updateGoal(uint64_t newGoal);
    // Optional/bonus: move the agent's start state, correctly bumping km the
    // way the original D* Lite algorithm does when the robot moves.
    PlanningResult moveStart(uint64_t newStart);

    // Re-runs ComputeShortestPath + ReconstructPath without any graph edit;
    // useful after several raw updates() are batched via the setters above.
    PlanningResult replan();

    int lastExploredStates() const { return exploredStatesThisCall_; }
    ObjectiveWeights weights;

private:
    struct Key {
        double k1;
        double k2;
        bool operator<(const Key& o) const {
            if (k1 != o.k1) return k1 < o.k1;
            return k2 < o.k2;
        }
        bool operator==(const Key& o) const { return k1 == o.k1 && k2 == o.k2; }
    };
    struct QueueEntry {
        Key key;
        uint64_t id;
        bool operator>(const QueueEntry& o) const { return key.k1 == o.key.k1 ? key.k2 > o.key.k2 : key.k1 > o.key.k1; }
    };

    // Problem data
    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitions_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> outTransitionIds_; // from -> [transitionId]
    std::unordered_map<uint64_t, std::vector<uint64_t>> inTransitionIds_;  // to   -> [transitionId]
    std::unordered_set<uint64_t> badStates_;
    uint64_t startState_ = 0;
    uint64_t goalState_ = 0;
    bool initialized_ = false;

    // Search state
    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    double km_ = 0.0;
    uint64_t lastStartForKm_ = 0;

    // Lazy-deletion priority queue
    std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open_;
    std::unordered_map<uint64_t, Key> openKey_; // node -> currently-valid key (absence == not open)

    int exploredStatesThisCall_ = 0;

    // state id -> Euclidean distance (in Cartesian embedding space) from that
    // state to the nearest bad state; +INF if there are no bad states.
    // Recomputed (O(|S|*|B|)) whenever the bad-state set changes, so that
    // edgeCost() can look it up in O(1) instead of rescanning all bad states
    // on every single edge relaxation.
    std::unordered_map<uint64_t, double> nearestBadDist_;
    void recomputeNearestBadDistances();

    double gOf(uint64_t s) const;
    double rhsOf(uint64_t s) const;
    double heuristic(uint64_t a, uint64_t b) const; // beta * Euclidean distance between embeddings
    // Composite edge weight actually minimized by the search:
    //   w(u,v) = beta*cost(u,v) + gamma*[ (1-safety(u,v)) + 1/(eps+distToNearestBad(v)) ]
    //            + delta*(-ln(reliability(u,v)))
    // beta/gamma/delta come from `weights` (ObjectiveWeights), so changing
    // them changes which path D* Lite actually finds -- not just how the
    // final path is *scored*. Returns +INF if the transition doesn't exist,
    // is unavailable, or leads into a bad state (hard constraint). See
    // report.md section 11 for the full derivation and justification.
    double edgeCost(uint64_t u, uint64_t v, uint64_t* usedTransitionId = nullptr) const;

    Key calculateKey(uint64_t s) const;
    void queueInsertOrUpdate(uint64_t s, const Key& k);
    void queueRemove(uint64_t s);
    void cleanStaleTop();
    bool queueEmpty();
    Key queueTopKey();
    uint64_t queuePeekTopId();

    void updateVertex(uint64_t u);
    void computeShortestPath();

    // Builds a PlanningResult from the current g/rhs solution by walking
    // greedily from startState_ to goalState_ following minimum edge-cost +
    // g(successor); independently validates the result before returning it.
    PlanningResult reconstructAndValidate();

    void rebuildAdjacency();
    void initializeSearch(); // Initialize() from the D* Lite paper
};
