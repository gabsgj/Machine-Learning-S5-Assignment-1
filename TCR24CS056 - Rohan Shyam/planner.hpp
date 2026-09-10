#pragma once

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <string>
#include <limits>
#include <cmath>

/**
 * @brief Represents a state in the finite Cartesian state space.
 */
struct State {
    uint64_t id;
    std::vector<double> embedding;
};

/**
 * @brief Represents a directed transition between two states.
 */
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

/**
 * @brief Definition of the planning problem.
 */
struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

/**
 * @brief Output result of the path planning procedure.
 */
struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;

    // Evaluation metrics
    double planningTimeUs = 0.0;   // Execution time in microseconds
    size_t nodesExplored = 0;      // Number of node expansions
    size_t badStatesVisited = 0;   // Count of bad states in path (must always be 0)
};

/**
 * @brief Priority key for D* Lite nodes: [k1, k2].
 */
struct DStarKey {
    double k1 = 0.0;
    double k2 = 0.0;

    bool operator<(const DStarKey& other) const {
        constexpr double EPS = 1e-9;
        if (k1 + EPS < other.k1) return true;
        if (k1 - EPS > other.k1) return false;
        return k2 + EPS < other.k2;
    }

    bool operator<=(const DStarKey& other) const {
        return (*this < other) || (!(*this < other) && !(other < *this));
    }
};

/**
 * @brief Safe Semantic Planner using D* Lite.
 *
 * Implements incremental backward-search D* Lite with Euclidean heuristic in
 * Cartesian embedding space, strict bad-state avoidance, and geometric safety calculation.
 */
class Planner {
public:
    Planner();

    /**
     * @brief Plans a path for the given problem from scratch.
     */
    PlanningResult plan(const PlanningProblem& problem);

    /**
     * @brief Updates the goal state and prepares for replanning.
     */
    void updateGoal(uint64_t goal);

    /**
     * @brief Updates the start state (e.g. when agent moves).
     */
    void updateStart(uint64_t start);

    /**
     * @brief Updates availability of a transition (enable/disable).
     */
    void updateTransition(uint64_t transitionId, bool available);

    /**
     * @brief Dynamically adds a new transition to the graph.
     */
    void addTransition(const Transition& transition);

    /**
     * @brief Sets the safety trade-off weight lambda >= 0.
     * When weight > 0, edge cost includes penalty inversely proportional to distance to bad states.
     */
    void setSafetyWeight(double weight);
    double getSafetyWeight() const;

    /**
     * @brief Incrementally replans using D* Lite after dynamic changes.
     */
    PlanningResult replan();

    // Geometric and heuristic utilities
    static double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b);
    static double heuristic(const State& a, const State& b);
    double stateSafetyDistance(uint64_t stateId) const;

    // Lookup helpers
    const PlanningProblem& getProblem() const { return problem_; }
    bool hasState(uint64_t id) const { return stateMap_.find(id) != stateMap_.end(); }
    const State& getState(uint64_t id) const { return stateMap_.at(id); }
    const Transition& getTransition(uint64_t id) const { return transitionMap_.at(id); }

private:
    // Core D* Lite internal routines
    DStarKey calculateKey(uint64_t u);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    double getEffectiveCost(const Transition& t) const;
    PlanningResult extractPath();

    // D* Lite state variables
    static constexpr double INF = std::numeric_limits<double>::infinity();

    PlanningProblem problem_;
    uint64_t startState_ = 0;
    uint64_t goalState_ = 0;
    uint64_t lastStartState_ = 0;
    double km_ = 0.0;
    double safetyWeight_ = 0.0;

    std::unordered_set<uint64_t> badStateSet_;
    std::unordered_map<uint64_t, State> stateMap_;
    std::unordered_map<uint64_t, Transition> transitionMap_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> succTransitions_; // u -> out transition IDs
    std::unordered_map<uint64_t, std::vector<uint64_t>> predTransitions_; // u -> in transition IDs

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;

    // Open list (priority queue with efficient key updates and removals)
    std::set<std::pair<DStarKey, uint64_t>> openSet_;
    std::unordered_map<uint64_t, DStarKey> openMap_;

    // Performance tracking
    size_t nodesExplored_ = 0;
};
