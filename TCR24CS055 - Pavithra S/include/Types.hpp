#ifndef TYPES_HPP
#define TYPES_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <iostream>

/**
 * State representation in a finite Cartesian state space R^d.
 * Each state contains a unique identifier and an embedding vector.
 */
class State {
public:
    uint64_t id;
    std::vector<double> embedding;

    State() : id(0), embedding({}) {}
    State(uint64_t id, std::vector<double> embedding) 
        : id(id), embedding(std::move(embedding)) {}
};

/**
 * Directed transition between two states in the state space.
 * Contains cost, safety rating, reliability metric, and dynamic availability status.
 */
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;

    Transition() 
        : id(0), from(0), to(0), cost(1.0), safety(1.0), reliability(1.0), available(true) {}

    Transition(uint64_t id, uint64_t from, uint64_t to, double cost = 1.0, 
               double safety = 1.0, double reliability = 1.0, bool available = true)
        : id(id), from(from), to(to), cost(cost), safety(safety), 
          reliability(reliability), available(available) {}
};

/**
 * Encapsulates the entire planning problem instance.
 */
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem() : initialState(0), goalState(0) {}
};

/**
 * Output result produced by the planner.
 */
class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;

    // Diagnostic evaluation metrics
    double cumulativeReliability;
    double minSafetyDistance;
    double multiObjectiveScore;
    size_t exploredStatesCount;
    double executionTimeMicroseconds;

    PlanningResult() 
        : success(false), totalCost(0.0), safetyScore(0.0),
          cumulativeReliability(1.0), minSafetyDistance(0.0),
          multiObjectiveScore(0.0), exploredStatesCount(0), 
          executionTimeMicroseconds(0.0) {}
};

/**
 * Abstract base class defining the Planner interface.
 */
class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

#endif // TYPES_HPP
