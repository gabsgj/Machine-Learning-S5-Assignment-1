#ifndef PLANNING_PROBLEM_HPP
#define PLANNING_PROBLEM_HPP

#include <cstdint>
#include <vector>
#include "State.hpp"
#include "Transition.hpp"

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem() : initialState(0), goalState(0) {}

    PlanningProblem(uint64_t initial, uint64_t goal,
                    const std::vector<uint64_t>& bads,
                    const std::vector<State>& sts,
                    const std::vector<Transition>& trans)
        : initialState(initial), goalState(goal), badStates(bads),
          states(sts), transitions(trans) {}
};

#endif // PLANNING_PROBLEM_HPP
