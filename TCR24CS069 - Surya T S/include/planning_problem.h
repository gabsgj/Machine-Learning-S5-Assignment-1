#ifndef PLANNING_PROBLEM_H
#define PLANNING_PROBLEM_H

#include <cstdint>
#include <vector>
#include "state.h"
#include "transition.h"

/// @brief Encapsulates the complete specification of a planning problem.
class PlanningProblem {
public:
    uint64_t initialState;                ///< ID of the initial state s_I
    uint64_t goalState;                   ///< ID of the goal state s_G
    std::vector<uint64_t> badStates;      ///< Set of bad states B to avoid
    std::vector<State> states;            ///< All states in S
    std::vector<Transition> transitions;  ///< All directed transitions T

    PlanningProblem()
        : initialState(0), goalState(0) {}
};

#endif // PLANNING_PROBLEM_H
