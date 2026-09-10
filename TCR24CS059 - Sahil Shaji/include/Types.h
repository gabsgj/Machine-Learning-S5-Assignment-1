#pragma once

#include <vector>
#include <cstdint>

// State class as specified in assignment
class State {
public:
    uint64_t id;
    std::vector<double> embedding;
};

// Transition class as specified in assignment
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

// PlanningProblem class as specified in assignment
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

// PlanningResult class as specified in assignment
class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
};
