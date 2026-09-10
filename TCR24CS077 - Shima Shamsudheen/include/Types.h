#pragma once
#include <cstdint>
#include <vector>

// A state embedded in R^d.
struct State {
    uint64_t id;
    std::vector<double> embedding;
};

// A directed transition between two states.
struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;       // per-edge safety score in [0,1], higher = safer
    double reliability;  // per-edge reliability in [0,1], higher = more reliable
    bool available;

    // --- Bonus: time-dependent availability -------------------------------
    // The transition is only usable when the planner's current time t
    // satisfies availableFrom <= t < availableUntil, IN ADDITION to the
    // `available` flag above (both must hold). Defaults span all time, so
    // existing code/tests that never set these fields are unaffected.
    double availableFrom = -1e300;
    double availableUntil = 1e300;
};

// A planning problem instance.
struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

// Result of a planning query, plus the metrics the assignment asks us to report.
struct PlanningResult {
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;        // sum of raw transition costs along the path
    double safetyScore = 0.0;      // minimum Euclidean clearance to nearest bad state along the path
    double cumulativeReliability = 1.0;
    int statesExplored = 0;        // number of vertices popped from the priority queue (search effort)
    double planningTimeMs = 0.0;
    double replanTimeMs = -1.0;    // -1 means "this was a cold plan, not a replan"
};
