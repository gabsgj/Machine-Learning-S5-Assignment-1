#pragma once
#include "State.h"
#include "Transition.h"
#include <vector>
#include <unordered_map>

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    // adjacency list built on demand
    std::unordered_map<uint64_t, std::vector<Transition>> adj;

    void buildAdjacency() {
        adj.clear();
        for (auto &t : transitions) {
            if (t.available) adj[t.from].push_back(t);
        }
    }
};
