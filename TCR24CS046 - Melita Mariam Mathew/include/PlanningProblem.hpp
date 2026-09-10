#ifndef PLANNING_PROBLEM_HPP
#define PLANNING_PROBLEM_HPP

#include "State.hpp"
#include "Transition.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <stdexcept>

/**
 * @brief Encapsulates a Safe Semantic Planning Problem in a finite Cartesian state space.
 */
class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;

    PlanningProblem() : initialState(0), goalState(0) {}

    PlanningProblem(uint64_t init, uint64_t goal,
                    const std::vector<uint64_t>& bads,
                    const std::vector<State>& st,
                    const std::vector<Transition>& tr)
        : initialState(init), goalState(goal), badStates(bads), states(st), transitions(tr) {}

    /**
     * @brief Checks if a state ID belongs to the bad states set.
     */
    bool isBadState(uint64_t stateId) const {
        for (uint64_t b : badStates) {
            if (b == stateId) return true;
        }
        return false;
    }

    /**
     * @brief Returns a pointer to a State given its ID, or nullptr if not found.
     */
    const State* getState(uint64_t id) const {
        for (const auto& s : states) {
            if (s.id == id) return &s;
        }
        return nullptr;
    }

    /**
     * @brief Returns a pointer to a Transition given its ID, or nullptr if not found.
     */
    const Transition* getTransition(uint64_t id) const {
        for (const auto& t : transitions) {
            if (t.id == id) return &t;
        }
        return nullptr;
    }

    /**
     * @brief Computes minimum Euclidean distance from a state to any bad state in Cartesian space.
     * If no bad states exist, returns infinity (represented as a large positive constant 1e9).
     */
    double minDistanceToBadStates(uint64_t stateId) const {
        const State* s = getState(stateId);
        if (!s || badStates.empty()) return 1e9;

        double minDist = 1e9;
        for (uint64_t bId : badStates) {
            const State* bState = getState(bId);
            if (bState) {
                double d = s->euclideanDistance(*bState);
                if (d < minDist) minDist = d;
            }
        }
        return minDist;
    }
};

#endif // PLANNING_PROBLEM_HPP
