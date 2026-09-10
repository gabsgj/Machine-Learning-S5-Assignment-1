#pragma once
#include "Planner.h"
#include <unordered_map>
#include <vector>
#include <limits>
#include <set>

// ---------------------------------------------------------------------------
// LPA* (Lifelong Planning A*) based safe semantic planner.
//
// Design summary (see design report for full justification):
//   * The multi-objective score  Score = aG - bC + gD + dR  is turned into a
//     single additive edge weight so that a standard shortest-path search
//     (LPA*) can be reused:
//         w(u,v) = W_COST * cost(u,v)
//                + W_SAFETY * safetyPenalty(v)
//                + W_RELIABILITY * (1 - reliability(u,v))
//     safetyPenalty(v) is a soft "potential field" term: it is 0 once v is
//     farther than SAFETY_RADIUS from every bad state, and grows linearly as
//     v approaches a bad state. This keeps the search additive/shortest-path
//     friendly while still steering the planner away from danger.
//   * Objective 2 ("never visit a bad state") is enforced as a HARD
//     constraint, not a soft penalty: any transition whose destination is a
//     bad state is treated as unavailable (infinite weight). The soft
//     safetyPenalty above only shapes the path *among* the states that are
//     already safe to visit.
//   * The true minimum clearance to a bad state along the final path is
//     reported separately in PlanningResult.safetyScore, since that quantity
//     (a min over the path) is not itself additive.
//   * The heuristic defaults to 0 (i.e. LPA* degrades gracefully to
//     incremental Dijkstra). This guarantees correctness regardless of
//     whether edge weights behave like a metric, and — importantly — it
//     makes goal changes (Test Case 5) nearly free: because the search
//     order no longer depends on the goal at all, previously computed g()
//     values close to the start remain valid and reusable when the goal
//     moves. A distance-based heuristic can optionally be enabled for extra
//     speed when edge cost is known to correlate with Euclidean distance.
// ---------------------------------------------------------------------------
class LPAStarPlanner : public Planner {
public:
    LPAStarPlanner();

    // One-shot planning entry point required by the Planner interface.
    // Internally this calls initialize() (if needed) + computeShortestPath()
    // + extractPath().
    PlanningResult plan(const PlanningProblem& problem) override;

    // --- Dynamic environment API -------------------------------------------------
    // These are the operations described in the "Dynamic Environment" section
    // of the assignment. Each performs the minimal incremental update LPA*
    // needs (UpdateVertex on affected states) instead of rebuilding from
    // scratch, then the next plan()/replan() call only re-expands the
    // inconsistent frontier.

    void setGoal(uint64_t newGoalState);
    void setTransitionAvailability(uint64_t transitionId, bool available);
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId);
    void setBadStates(const std::vector<uint64_t>& badStates);

    // --- Bonus: time-dependent transition availability ---------------------
    // Advances the planner's notion of "now". Any transition whose
    // [availableFrom, availableUntil) window no longer contains newTime (or
    // now does) has its destination's rhs re-evaluated, exactly like an
    // availability toggle. Call replan() afterwards to get the updated path.
    void setCurrentTime(double newTime);

    // Re-run the incremental search after any of the above and extract the
    // new path. Reuses all g()/rhs() values that are still consistent.
    PlanningResult replan();

    // --- Bonus: multi-goal planning -----------------------------------------
    // Plans to whichever state in goalCandidates is cheapest to reach, without
    // requiring the caller to know which one that is in advance. Implemented
    // by attaching a single virtual "meta-goal" state to the problem with a
    // free (zero-cost, always-available) incoming edge from every candidate,
    // then running the ordinary single-goal search to that meta-goal — so all
    // of LPA*'s incremental-replanning guarantees carry over unchanged. The
    // meta-goal is stripped back out of the returned path before it is
    // handed to the caller.
    PlanningResult planMultiGoal(const PlanningProblem& problem, const std::vector<uint64_t>& goalCandidates);

    // Tuning weights, exposed for the experimental section of the report.
    double W_COST = 1.0;
    double W_SAFETY = 4.0;
    double W_RELIABILITY = 2.0;
    double SAFETY_RADIUS = 2.0;   // states farther than this from every bad state get zero penalty
    double HEURISTIC_WEIGHT = 0.0; // 0 = pure Dijkstra-style LPA*; >0 enables Euclidean-distance heuristic

private:
    struct Key {
        double k1, k2;
        bool operator<(const Key& o) const {
            if (k1 != o.k1) return k1 < o.k1;
            return k2 < o.k2;
        }
        bool operator<=(const Key& o) const { return !(o < *this); }
    };

    struct QueueEntry {
        Key key;
        uint64_t id;
        bool operator<(const QueueEntry& o) const {
            // We want a MIN-heap on key but std::set/priority_queue in C++
            // are max-oriented by default for priority_queue; we use
            // std::set here, ordered ascending by (key, id) for determinism.
            if (!(key < o.key) && !(o.key < key)) return id < o.id;
            return key < o.key;
        }
    };

    static constexpr double INF = std::numeric_limits<double>::infinity();
    static constexpr uint64_t META_GOAL_ID = std::numeric_limits<uint64_t>::max(); // reserved id for planMultiGoal's virtual node

    double currentTime_ = 0.0;

    uint64_t startState_;
    uint64_t goalState_;
    bool initialized_ = false;

    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitionsById_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> outgoing_; // stateId -> transition ids
    std::unordered_map<uint64_t, std::vector<uint64_t>> incoming_; // stateId -> transition ids
    std::set<uint64_t> badStates_;
    std::unordered_map<uint64_t, double> clearance_; // stateId -> min Euclidean distance to nearest bad state

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    std::set<QueueEntry> openSet_;
    std::unordered_map<uint64_t, Key> inQueueKey_; // current key of each queued vertex, for lazy removal

    int statesExplored_ = 0;

    void initialize(const PlanningProblem& problem);
    void recomputeClearances();
    double euclidean(uint64_t a, uint64_t b) const;
    double heuristic(uint64_t s) const;
    double edgeWeight(const Transition& t) const;
    Key calculateKey(uint64_t s) const;
    void updateVertex(uint64_t u);
    void computeShortestPath();
    PlanningResult extractPath(double elapsedMs, bool isReplan);
    double gOf(uint64_t s) const;
    double rhsOf(uint64_t s) const;
};
