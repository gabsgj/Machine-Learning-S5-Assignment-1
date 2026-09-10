#pragma once

#include <string>
#include <vector>
#include <limits>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <iomanip>

namespace safe_semantic_planner {

/**
 * @brief Weight parameters for multi-objective cost and safety filtering:
 *        w(u, v) = beta * cost(u, v) - delta * reliability(u, v)
 *        r = hard safety exclusion radius around bad states.
 *        alpha, gamma = additional semantic / safety weights per specification.
 */
struct WeightParams {
    double alpha = 1.0;
    double beta = 1.0;
    double gamma = 1.0;
    double delta = 0.0;
    double r = 0.0;

    WeightParams() = default;
    WeightParams(double a, double b, double g, double d, double radius)
        : alpha(a), beta(b), gamma(g), delta(d), r(radius) {}
};

/**
 * @brief State in the discrete semantic graph with a Cartesian embedding vector.
 */
struct State {
    std::string id;
    std::string label;
    std::vector<double> embedding;
    
    // Safety & exclusion metadata
    double distanceToNearestBadState = std::numeric_limits<double>::infinity();
    bool isBadState = false;
    bool isExcluded = false;

    State() = default;
    State(std::string id_, std::vector<double> embedding_, std::string label_ = "")
        : id(std::move(id_)), label(label_.empty() ? id : std::move(label_)), embedding(std::move(embedding_)) {}
};

/**
 * @brief Directed transition between two states.
 */
struct Transition {
    std::string id;
    std::string from;
    std::string to;
    double cost = 1.0;
    double reliability = 1.0;
    double safetyScore = 1.0;
    bool available = true;

    Transition() = default;
    Transition(std::string id_, std::string from_, std::string to_,
               double cost_, double reliability_, double safetyScore_ = 1.0, bool available_ = true)
        : id(std::move(id_)), from(std::move(from_)), to(std::move(to_)),
          cost(cost_), reliability(reliability_), safetyScore(safetyScore_), available(available_) {}
};

/**
 * @brief Complete planning problem definition containing all states and transitions.
 */
struct PlanningProblem {
    std::string initialState;
    std::string goalState;
    std::vector<std::string> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem() = default;
};

/**
 * @brief Result of path planning execution with detailed metrics.
 */
struct PlanningResult {
    bool success = false;
    std::vector<std::string> statePath;
    std::vector<std::string> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;
    std::string errorMessage;

    PlanningResult() = default;
};

/**
 * @brief Complete metrics reported by the planner.
 */
struct PlannerMetrics {
    size_t statesExplored = 0;       ///< Expansions in the latest compute/replan call
    double planningTimeMs = 0.0;     ///< Initial solve wall-clock time in ms
    double replanningTimeMs = 0.0;   ///< Latest replan cycle wall-clock time in ms
    size_t memoryBytes = 0;          ///< Analytical memory calculation in bytes
    double totalCost = 0.0;          ///< Total cost of current path (0 if no path)
    double minClearance = 0.0;       ///< Minimum distance to nearest bad state along path
    size_t badStatesVisited = 0;     ///< Number of bad states visited (should be 0)
    size_t goalSuccessCount = 0;     ///< Running count of successful goal reaches across lifetime
    size_t attemptCount = 0;         ///< Running count of total planning attempts across lifetime

    double getGoalSuccessRate() const {
        return (attemptCount == 0) ? 0.0 : (static_cast<double>(goalSuccessCount) / static_cast<double>(attemptCount));
    }
};

/**
 * @brief Validation exception thrown when weight or graph bounds are violated.
 */
class ValidationException : public std::runtime_error {
public:
    explicit ValidationException(const std::string& message)
        : std::runtime_error(message) {}
};

} // namespace safe_semantic_planner
