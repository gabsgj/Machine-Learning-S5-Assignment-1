#pragma once

#include <cstdint>
#include <cstddef>
#include <vector>

class State {
public:
    uint64_t id;
    std::vector<double> embedding;
};

class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;
    std::size_t exploredStates = 0;
    double planningTimeMs = 0.0;
    double replanningTimeMs = 0.0;
    double peakMemoryKB = 0.0;
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};
