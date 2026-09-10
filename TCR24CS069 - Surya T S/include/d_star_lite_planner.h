#ifndef D_STAR_LITE_PLANNER_H
#define D_STAR_LITE_PLANNER_H

#include "planner.h"
#include "safety_utils.h"

#include <cstdint>
#include <functional>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

/// @brief A pair of doubles used as D* Lite priority key [k1, k2].
///        Compared lexicographically.
struct KeyPair {
    double k1;
    double k2;

    bool operator<(const KeyPair& other) const {
        if (k1 != other.k1) return k1 < other.k1;
        return k2 < other.k2;
    }
    bool operator>(const KeyPair& other) const { return other < *this; }
    bool operator<=(const KeyPair& other) const { return !(other < *this); }
    bool operator>=(const KeyPair& other) const { return !(*this < other); }
    bool operator==(const KeyPair& other) const {
        return k1 == other.k1 && k2 == other.k2;
    }
    bool operator!=(const KeyPair& other) const { return !(*this == other); }
};

/// @brief D* Lite incremental planner with multi-objective cost augmentation.
///
/// Searches backward from the goal to the start, maintaining g-values and
/// rhs-values for each state. Supports efficient replanning when the goal
/// moves, bad states change, or transitions are added/removed/toggled.
class DStarLitePlanner : public Planner {
public:
    // ── Construction ────────────────────────────────────────────────
    DStarLitePlanner();
    ~DStarLitePlanner() override = default;

    // ── Core interface ──────────────────────────────────────────────
    /// @brief Perform initial planning from scratch.
    PlanningResult plan(const PlanningProblem& problem) override;

    // ── Incremental replanning API ──────────────────────────────────
    /// @brief Change the goal state and replan incrementally.
    void updateGoal(uint64_t newGoal);

    /// @brief Add a new bad state and replan.
    void addBadState(uint64_t stateId);

    /// @brief Remove a bad state and replan.
    void removeBadState(uint64_t stateId);

    /// @brief Toggle availability of a transition and replan.
    void setTransitionAvailability(uint64_t transId, bool available);

    /// @brief Insert a new transition (shortcut) and replan.
    void addTransition(const Transition& t);

    /// @brief Replan after incremental updates have been applied.
    PlanningResult replan();

    // ── Weight configuration ────────────────────────────────────────
    /// @brief Set the objective function weights.
    /// @param beta   Weight on transition cost   (higher = penalize cost more)
    /// @param gamma  Weight on safety distance   (higher = reward distance more)
    /// @param delta  Weight on reliability        (higher = reward reliability more)
    void setWeights(double beta, double gamma, double delta);

    // ── Statistics ──────────────────────────────────────────────────
    /// @brief Return the number of states expanded during the last search.
    size_t getExploredCount() const { return exploredCount_; }

private:
    // ── D* Lite core ────────────────────────────────────────────────
    static constexpr double INF = std::numeric_limits<double>::infinity();

    KeyPair calculateKey(uint64_t s) const;
    void    initialize();
    void    updateVertex(uint64_t u);
    void    computeShortestPath();

    // ── Composite cost ──────────────────────────────────────────────
    /// @brief Compute the effective edge weight for a transition,
    ///        combining cost, safety distance, and reliability.
    double effectiveCost(const Transition& t) const;

    // ── Heuristic ───────────────────────────────────────────────────
    /// @brief Euclidean distance in R^d between two states (used as heuristic).
    double heuristic(uint64_t a, uint64_t b) const;

    // ── Path extraction ─────────────────────────────────────────────
    PlanningResult extractPath() const;

    // ── Safety map ──────────────────────────────────────────────────
    void recomputeSafetyMap();

    // ── Internal helpers ────────────────────────────────────────────
    double getG(uint64_t s) const;
    double getRhs(uint64_t s) const;
    void   setG(uint64_t s, double val);
    void   setRhs(uint64_t s, double val);

    /// @brief Get all predecessors of a state (states that have transitions TO s).
    std::vector<uint64_t> getPredecessors(uint64_t s) const;

    /// @brief Get all successors of a state (states reachable FROM s).
    std::vector<uint64_t> getSuccessors(uint64_t s) const;

    /// @brief Find the transition from u to v, or nullptr if none available.
    const Transition* findTransition(uint64_t from, uint64_t to) const;

    // ── Data members ────────────────────────────────────────────────

    // Problem data (stored copy for replanning)
    PlanningProblem problem_;

    // D* Lite state
    std::unordered_map<uint64_t, double> g_;     ///< g-values (cost-to-goal)
    std::unordered_map<uint64_t, double> rhs_;   ///< rhs-values (one-step lookahead)
    double km_;                                   ///< Key modifier for start movement

    // Priority queue: ordered set of (key, state_id)
    std::set<std::pair<KeyPair, uint64_t>> openSet_;
    std::unordered_map<uint64_t, KeyPair>  openKeys_;  ///< Current key for each state in openSet_

    // Graph adjacency
    std::unordered_map<uint64_t, std::vector<size_t>> adjForward_;   ///< state → outgoing transition indices
    std::unordered_map<uint64_t, std::vector<size_t>> adjReverse_;   ///< state → incoming transition indices

    // State lookup
    std::unordered_map<uint64_t, const State*> stateLookup_;

    // Safety
    std::unordered_map<uint64_t, double> safetyMap_;       ///< state_id → min distance to bad
    std::unordered_set<uint64_t>         badStateSet_;     ///< O(1) bad state membership

    // Weights
    double beta_  = 1.0;   ///< Cost weight
    double gamma_ = 0.5;   ///< Safety distance weight
    double delta_ = 0.3;   ///< Reliability weight

    // Statistics
    mutable size_t exploredCount_ = 0;

    // ── Graph building ──────────────────────────────────────────────
    void buildAdjacencyLists();
    void buildStateLookup();

    // ── Open-set helpers ────────────────────────────────────────────
    void   insertOpen(uint64_t s, KeyPair key);
    void   removeOpen(uint64_t s);
    bool   isInOpen(uint64_t s) const;
    KeyPair topKey() const;
    uint64_t topState() const;
    void   popTop();
};

#endif // D_STAR_LITE_PLANNER_H
