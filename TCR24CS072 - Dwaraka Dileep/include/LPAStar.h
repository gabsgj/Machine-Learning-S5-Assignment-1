#ifndef SAFE_PLANNER_LPA_STAR_H
#define SAFE_PLANNER_LPA_STAR_H

#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>
#include <set>
#include <optional>

#include "Planner.h"
#include "State.h"
#include "Transition.h"
#include "PlanningProblem.h"
#include "PlanningResult.h"

namespace planner {

// LPA* (Lifelong Planning A*, Koenig & Likhachev 2004) computes a shortest
// path from a fixed start to a fixed goal and can *incrementally* repair
// that path when edge costs, availability, or the graph topology change,
// without recomputing everything from scratch. This is exactly the
// "dynamic environment" requirement in the assignment: transitions may be
// added, removed, or flip availability between calls, and we want to reuse
// as much of the previous search effort as possible.
//
// Design notes (see report/Design_Report.md for the full write-up):
//  - Bad states are removed from the search graph entirely: no vertex for
//    a bad state is ever created, so the planner can structurally never
//    step onto one (objective #2).
//  - A hard `safetyRadius` additionally excludes any transition landing in
//    a state closer than that radius to *any* bad state.
//  - Edge weight combines the raw transition cost with soft penalties for
//    low per-edge safety/reliability and for low geometric clearance from
//    bad states, so that among structurally-safe paths the search still
//    prefers cheaper *and* safer ones (objectives #3 and #4). All penalty
//    terms are non-negative, which preserves Dijkstra/A*-style correctness
//    (LPA* requires non-negative edge weights).
//  - Goal changes require re-deriving the heuristic for every vertex (h is
//    defined relative to the goal), so those force a full reinitialization
//    of g/rhs/priority-queue state. Edge/availability changes do not: they
//    only touch UpdateVertex for the endpoints of the changed edge, which
//    is the incremental fast path.
class LPAStarPlanner : public Planner {
public:
    LPAStarPlanner() = default;

    // One-shot planning: (re)initializes all internal state for `problem`
    // and computes a shortest safe path from scratch. Use this the first
    // time a problem is solved, or whenever the goal / bad-state set /
    // state set changes.
    PlanningResult plan(const PlanningProblem& problem) override;

    // --- Incremental replanning API -------------------------------------
    // These assume `plan()` has already been called once to establish the
    // internal graph and search state. They mutate that state in place and
    // only touch the vertices whose shortest-path value could possibly be
    // affected, then resume ComputeShortestPath. Returns the updated
    // result. Must not be called before plan().

    // Change the cost/safety/reliability/availability of an existing
    // transition (looked up by id). Returns false if no such transition
    // exists.
    bool updateTransition(uint64_t transitionId, double newCost,
                           bool newAvailable, double newSafety = -1.0,
                           double newReliability = -1.0);

    // Insert a brand-new transition into the live graph.
    void addTransition(const Transition& t);

    // Remove a transition (equivalent to marking it permanently
    // unavailable and dropping it from the adjacency lists).
    bool removeTransition(uint64_t transitionId);

    // Re-run ComputeShortestPath and extract the result after one or more
    // of the incremental mutators above have been called. This is the
    // "fast path" replan: it reuses all previously computed g/rhs values.
    PlanningResult replan();

    // Changing the goal (or the bad-state set, which changes the legal
    // vertex set) is NOT incrementally supported by classic LPA* -- see
    // the class-level comment. This convenience method performs a full
    // reinitialization against a new problem definition, which is the
    // "rebuild only what you must" fallback described in the report.
    PlanningResult replanWithNewGoal(const PlanningProblem& updatedProblem);

    std::size_t lastStatesExplored() const { return statesExplored_; }

private:
    struct Key {
        double k1;
        double k2;
        bool operator<(const Key& other) const {
            if (k1 != other.k1) return k1 < other.k1;
            return k2 < other.k2;
        }
        bool operator==(const Key& other) const {
            return k1 == other.k1 && k2 == other.k2;
        }
    };

    static constexpr double INF = std::numeric_limits<double>::infinity();

    // Graph / problem data
    PlanningProblem problem_;
    std::unordered_map<uint64_t, State> stateById_;
    std::unordered_map<uint64_t, std::vector<Transition>> outEdges_;
    std::unordered_map<uint64_t, std::vector<Transition>> inEdges_;
    std::unordered_map<uint64_t, double> clearance_; // distance to nearest bad state
    bool safetyRadiusUsesClearance_ = true;

    // Search state
    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::set<std::pair<Key, uint64_t>> openSet_;
    std::unordered_map<uint64_t, Key> openKeyOf_;
    uint64_t start_ = 0;
    uint64_t goal_ = 0;
    bool initialized_ = false;
    std::size_t statesExplored_ = 0;

    double heuristic(uint64_t s) const;
    double edgeWeight(const Transition& t) const;
    Key calculateKey(uint64_t s) const;
    void updateVertex(uint64_t s);
    void computeShortestPath();
    void buildGraph(const PlanningProblem& problem);
    PlanningResult extractResult();
};

} // namespace planner

#endif // SAFE_PLANNER_LPA_STAR_H
