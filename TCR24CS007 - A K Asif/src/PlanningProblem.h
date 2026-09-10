#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#include "State.h"
#include "Transition.h"
#include <cstdint>
#include <vector>

struct PlanningProblem {
    uint64_t initialState{};
    uint64_t goalState{};
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

#endif
