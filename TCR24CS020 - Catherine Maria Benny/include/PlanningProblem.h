#pragma once
#include <cstdint>
#include <vector>
#include "State.h"
#include "Transition.h"

// Full specification of a planning problem instance: the graph, the initial
// and goal states, and the set of bad (forbidden) states.
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};
