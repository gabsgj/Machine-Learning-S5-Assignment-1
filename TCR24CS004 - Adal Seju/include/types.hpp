#pragma once

#include <cstdint>
#include <vector>
#include <string>
#include <limits>
#include <chrono>

// ============================================================================
// PCCST503 - Machine Learning: Assignment 1
// "Design of a Safe Semantic Planner in a Finite Cartesian State Space"
//
// Core Data Structures & Interfaces conforming strictly to Assignment Specifications
// ============================================================================

namespace SafePlanner {

constexpr double INF = std::numeric_limits<double>::infinity();
constexpr double EPSILON = 1e-9;

/**
 * @brief State embedded in a finite d-dimensional Cartesian space R^d.
 */
struct State {
    uint64_t id;
    std::vector<double> embedding;
    std::string label; // Optional semantic label (e.g., "Start", "Intersection_A", "Safe_Hub")

    State() : id(0), embedding({}), label("") {}
    State(uint64_t id_, const std::vector<double>& emb, const std::string& lbl = "")
        : id(id_), embedding(emb), label(lbl.empty() ? ("S" + std::to_string(id_)) : lbl) {}
};

/**
 * @brief Directed transition between two states with cost, safety, reliability, and availability.
 */
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // Per-transition safety rating [0.0, 1.0]
    double reliability;  // Per-transition operational reliability [0.0, 1.0]
    bool available;      // Availability flag (can change dynamically)
    
    // Optional temporal window for time-dependent availability (Bonus feature)
    double timeStart = 0.0;
    double timeEnd = INF;

    Transition()
        : id(0), from(0), to(0), cost(1.0), safety(1.0), reliability(1.0), available(true) {}
    
    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_ = 1.0, double reliability_ = 1.0, bool available_ = true,
               double tStart = 0.0, double tEnd = INF)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_), timeStart(tStart), timeEnd(tEnd) {}
};

/**
 * @brief Formal Planning Problem definition.
 */
struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    // Optional multi-goal sequence (Bonus feature)
    std::vector<uint64_t> waypoints;
    
    // Multi-objective weights: Score(P) = alpha*G - beta*C + gamma*D + delta*R
    double alpha = 100.0; // Goal completion weight
    double beta = 1.0;    // Transition cost weight
    double gamma = 2.0;   // Safety distance margin weight
    double delta = 5.0;   // Reliability weight
    double safetyMarginThreshold = 0.0; // Required clearance buffer
};

/**
 * @brief Result produced by the Safe Planner.
 */
struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    
    // Minimum Euclidean distance from any visited state to the nearest bad state.
    // If no bad states exist in the environment, this is +infinity.
    double safetyScore = INF;
    
    // Additional evaluation metrics
    double cumulativeReliability = 1.0;
    double averageSafetyDistance = INF;
    double objectiveScore = 0.0; // Score(P) = alpha*G - beta*C + gamma*D + delta*R
    
    // Performance telemetry
    size_t exploredStates = 0;
    size_t queueOperations = 0;
    double planningTimeMicroseconds = 0.0;
    std::string algorithmName = "LPA*";
};

/**
 * @brief Abstract Planner Interface specified by assignment.
 */
class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

} // namespace SafePlanner
