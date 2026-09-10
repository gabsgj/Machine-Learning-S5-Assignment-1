#include "LPAStar.h"
#include "Safety.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <chrono>

using namespace std;


LPAStar::LPAStar() {
    currentProblem = nullptr;
}

double LPAStar::getG(uint64_t stateID) {
    if (g.find(stateID) == g.end())
        return numeric_limits<double>::infinity();

    return g[stateID];
}

double LPAStar::getRHS(uint64_t stateID) {
    if (rhs.find(stateID) == rhs.end())
        return numeric_limits<double>::infinity();

    return rhs[stateID];
}

double LPAStar::heuristic(
    uint64_t stateID,
    uint64_t goalID
) {
    return 0.0;
}

double LPAStar::calculateRHS(uint64_t stateID) {

    if (stateID == currentProblem->initialState)
        return 0.0;

    double best = numeric_limits<double>::infinity();

    for (size_t i = 0;
         i < currentProblem->transitions.size();
         i++) {

        const Transition& t =
            currentProblem->transitions[i];

        if (!t.available)
            continue;

        if (t.to != stateID)
            continue;

        // Check whether destination is bad
        bool bad = false;

        for (size_t j = 0;
             j < currentProblem->badStates.size();
             j++) {

            if (t.to ==
                currentProblem->badStates[j]) {

                bad = true;
                break;
            }
        }

        if (bad)
            continue;

        double candidate =
            getG(t.from) + t.cost;

        if (candidate < best)
            best = candidate;
    }

    return best;
}

void LPAStar::updateVertex(uint64_t stateID) {

    if (stateID != currentProblem->initialState)
        rhs[stateID] =
            calculateRHS(stateID);
}


PlanningResult LPAStar::plan(
    const PlanningProblem& problem
) {

    currentProblem = &problem;

    PlanningResult result;
    auto startTime = chrono::high_resolution_clock::now();

    g.clear();
    rhs.clear();
    parent.clear();
    parentTransition.clear();

    // Initialize states
    for (size_t i = 0;
         i < problem.states.size();
         i++) {

        uint64_t id =
            problem.states[i].id;

        g[id] =
            numeric_limits<double>::infinity();

        rhs[id] =
            numeric_limits<double>::infinity();
    }

    // Initial state
    g[problem.initialState] = 0.0;
    rhs[problem.initialState] = 0.0;

    /*
        Repeated relaxation.

        Find the cheapest currently known
        path to every reachable state.
    */

    bool changed = true;

    while (changed) {

        changed = false;

        for (size_t i = 0;
             i < problem.transitions.size();
             i++) {
                 result.exploredStates++;

            const Transition& t =
                problem.transitions[i];

            if (!t.available)
                continue;

            // Don't enter bad states
            bool bad = false;

            for (size_t j = 0;
                 j < problem.badStates.size();
                 j++) {

                if (t.to ==
                    problem.badStates[j]) {

                    bad = true;
                    break;
                }
            }

            if (bad)
                continue;

            double fromCost =
                getG(t.from);

            if (fromCost ==
                numeric_limits<double>::infinity())
                continue;

            double newCost =
                fromCost + t.cost;

            if (newCost < getG(t.to)) {

                g[t.to] = newCost;
                rhs[t.to] = newCost;

                parent[t.to] = t.from;

                parentTransition[t.to] = t.id;

                changed = true;
            }
        }
    }

    // Check goal
    if (getG(problem.goalState) ==
        numeric_limits<double>::infinity()) {

        result.success = false;
        return result;
    }

    // -------------------------
    // RECONSTRUCT PATH
    // -------------------------

    uint64_t current =
        problem.goalState;

    result.statePath.push_back(current);

    while (current !=
           problem.initialState) {

        if (parent.find(current) ==
            parent.end()) {

            result.success = false;
            return result;
        }

        result.transitionPath.push_back(
            parentTransition[current]
        );

        current = parent[current];

        result.statePath.push_back(current);
    }

    reverse(
        result.statePath.begin(),
        result.statePath.end()
    );

    reverse(
        result.transitionPath.begin(),
        result.transitionPath.end()
    );

    result.success = true;

    result.totalCost =
        getG(problem.goalState);

    result.safetyScore =
        calculateSafetyDistance(
            result.statePath,
            problem
        );
    // Count bad states visited
result.badStatesVisited = 0;

for (size_t i = 0;
     i < result.statePath.size();
     i++) {

    for (size_t j = 0;
         j < problem.badStates.size();
         j++) {

        if (result.statePath[i] ==
            problem.badStates[j]) {

            result.badStatesVisited++;
        }
    }
}   
    // Estimate memory used by planner data structures
result.memoryUsageKB =
    (
        g.size() * sizeof(pair<const uint64_t, double>) +
        rhs.size() * sizeof(pair<const uint64_t, double>) +
        parent.size() * sizeof(pair<const uint64_t, uint64_t>) +
        parentTransition.size() *
        sizeof(pair<const uint64_t, uint64_t>)
    ) / 1024.0;

    auto endTime = chrono::high_resolution_clock::now();

    result.planningTimeMs =
       chrono::duration<double, milli>(
        endTime - startTime
       ).count();

    return result;
}