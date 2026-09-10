#pragma once
#include <cstdint>
#include <vector>
#include "State.hpp"
#include "Transition.hpp"

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};
