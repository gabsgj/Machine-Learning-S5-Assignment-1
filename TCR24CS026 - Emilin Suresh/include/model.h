#ifndef MODEL_H
#define MODEL_H

#include <cstdint>
#include <vector>
#include <string>

/*
 * Represents a state in the Cartesian state space.
 *
 * Example:
 * State S = (0, 0)
 * State A = (1, 1)
 */
struct State
{
    std::uint64_t id;
    std::vector<double> embedding;
    std::string name;

    State()
        : id(0)
    {
    }

    State(std::uint64_t id_,
          const std::vector<double>& embedding_,
          const std::string& name_ = "")
        : id(id_),
          embedding(embedding_),
          name(name_)
    {
    }
};


/*
 * Represents a directed transition.
 *
 * from -> to
 *
 * cost        : cost of using the transition
 * safety      : transition-level safety score
 * reliability : reliability in the range [0,1]
 * available   : whether the transition can currently be used
 */
struct Transition
{
    std::uint64_t id;
    std::uint64_t from;
    std::uint64_t to;

    double cost;
    double safety;
    double reliability;

    bool available;

    Transition()
        : id(0),
          from(0),
          to(0),
          cost(0.0),
          safety(0.0),
          reliability(1.0),
          available(true)
    {
    }

    Transition(std::uint64_t id_,
               std::uint64_t from_,
               std::uint64_t to_,
               double cost_,
               double safety_,
               double reliability_,
               bool available_ = true)
        : id(id_),
          from(from_),
          to(to_),
          cost(cost_),
          safety(safety_),
          reliability(reliability_),
          available(available_)
    {
    }
};


/*
 * Represents the complete planning problem.
 */
struct PlanningProblem
{
    std::uint64_t initialState;
    std::uint64_t goalState;

    std::vector<std::uint64_t> badStates;

    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem()
        : initialState(0),
          goalState(0)
    {
    }
};


/*
 * Result returned by the planner.
 */
struct PlanningResult
{
    bool success;

    std::vector<std::uint64_t> statePath;
    std::vector<std::uint64_t> transitionPath;

    double totalCost;
    double safetyScore;
    double minimumSafetyDistance;

    double cumulativeReliability;

    std::size_t exploredStates;

    double planningTimeMs;
    double replanningTimeMs;

    std::size_t memoryUsageBytes;

    PlanningResult()
        : success(false),
          totalCost(0.0),
          safetyScore(0.0),
          minimumSafetyDistance(0.0),
          cumulativeReliability(0.0),
          exploredStates(0),
          planningTimeMs(0.0),
          replanningTimeMs(0.0),
          memoryUsageBytes(0)
    {
    }
};

#endif