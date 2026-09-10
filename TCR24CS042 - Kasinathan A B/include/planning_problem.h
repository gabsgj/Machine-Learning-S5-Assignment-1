#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#include <cstdint>
#include <vector>
#include "state.h"
#include "transition.h"

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

#endif