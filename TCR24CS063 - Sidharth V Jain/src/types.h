#pragma once

#include <cstdint>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// Core domain types (matches assignment interface)
// ---------------------------------------------------------------------------

struct State {
    uint64_t id;
    std::vector<double> embedding; // position in R^d
};

struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

struct PlanningResult {
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;      // min Euclidean distance to nearest bad state
    double totalReliability;
    int exploredStates;
    double planningTimeMs;
    size_t memoryBytes;      // approximate
};

// ---------------------------------------------------------------------------
// Dynamic environment change descriptors
// ---------------------------------------------------------------------------

enum class ChangeType {
    TRANSITION_UNAVAILABLE,
    TRANSITION_AVAILABLE,
    TRANSITION_ADDED,
    TRANSITION_REMOVED,
    BAD_STATE_ADDED,
    BAD_STATE_REMOVED,
    GOAL_CHANGED
};

struct EnvironmentChange {
    ChangeType type;
    uint64_t transitionId  = 0; // for transition changes
    uint64_t stateId       = 0; // for bad-state / goal changes
    Transition newTransition{};  // payload for TRANSITION_ADDED
};
