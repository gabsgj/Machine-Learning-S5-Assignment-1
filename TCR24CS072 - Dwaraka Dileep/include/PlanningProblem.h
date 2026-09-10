#ifndef SAFE_PLANNER_PLANNING_PROBLEM_H
#define SAFE_PLANNER_PLANNING_PROBLEM_H

#include <cstdint>
#include <vector>
#include "State.h"
#include "Transition.h"

namespace planner {

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    // Scoring weights for Score(P) = alpha*G - beta*C + gamma*D + delta*R
    // Exposed here so experiments can sweep them.
    double alpha = 1.0;
    double beta = 1.0;
    double gamma = 1.0;
    double delta = 0.25;

    // Minimum clearance (Euclidean distance) any visited state must keep
    // from every bad state. Transitions that land a state closer than this
    // to a bad state are treated as unsafe and excluded from the search.
    double safetyRadius = 0.0;
};

} // namespace planner

#endif // SAFE_PLANNER_PLANNING_PROBLEM_H
