#include "DStarLitePlanner.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <unordered_set>

DStarLitePlanner::DStarLitePlanner()
    : km(0.0),
      start(0),
      goal(0),
      exploredStates(0) {
}


// ==================================================
// Get g value
// ==================================================

double DStarLitePlanner::getG(
    uint64_t state
) const {

    auto it = g.find(state);

    if (it == g.end())
        return INF;

    return it->second;
}


// ==================================================
// Get rhs value
// ==================================================

double DStarLitePlanner::getRHS(
    uint64_t state
) const {

    auto it = rhs.find(state);

    if (it == rhs.end())
        return INF;

    return it->second;
}


// ==================================================
// Euclidean heuristic
// ==================================================

double DStarLitePlanner::heuristic(
    uint64_t stateA,
    uint64_t stateB
) const {

    return problem.distanceBetweenStates(
        stateA,
        stateB
    );
}


// ==================================================
// Calculate D* Lite key
// ==================================================

std::pair<double, double>
DStarLitePlanner::calculateKey(
    uint64_t state
) const {

    double minimum =
        std::min(
            getG(state),
            getRHS(state)
        );

    return {
        minimum +
        heuristic(start, state) +
        km,

        minimum
    };
}


// ==================================================
// Safety penalty
// ==================================================

double DStarLitePlanner::safetyPenalty(
    uint64_t state
) const {

    if (problem.isBadState(state))
        return INF;

    if (problem.badStates.empty())
        return 0.0;

    double distance =
        problem.distanceToNearestBadState(
            state
        );

    if (std::isinf(distance))
        return 0.0;

    /*
       Smaller distance to a bad state produces
       a larger penalty.

       This encourages the planner to maintain
       a larger safety margin.
    */

    const double SAFETY_WEIGHT = 1.5;

    return SAFETY_WEIGHT /
           (distance + 0.1);
}


// ==================================================
// Effective transition cost
// ==================================================

double DStarLitePlanner::transitionCost(
    const Transition& transition
) const {

    double safety =
        std::max(
            0.0,
            std::min(
                1.0,
                transition.safety
            )
        );

    double reliability =
        std::max(
            0.0,
            std::min(
                1.0,
                transition.reliability
            )
        );

    double safetyAttributePenalty =
        1.0 - safety;

    double reliabilityPenalty =
        1.0 - reliability;

    double destinationSafetyPenalty =
        safetyPenalty(
            transition.to
        );

    if (std::isinf(
            destinationSafetyPenalty))
        return INF;

    return transition.cost
           + 1.5 *
             safetyAttributePenalty
           + 1.0 *
             reliabilityPenalty
           + destinationSafetyPenalty;
}


// ==================================================
// Build graph
// ==================================================

void DStarLitePlanner::buildGraph() {

    outgoing.clear();
    incoming.clear();

    for (const auto& transition :
         problem.transitions) {

        if (!transition.available)
            continue;

        outgoing[
            transition.from
        ].push_back(
            &transition
        );

        incoming[
            transition.to
        ].push_back(
            &transition
        );
    }
}


// ==================================================
// Initialize D* Lite
// ==================================================

void DStarLitePlanner::initialize() {

    g.clear();
    rhs.clear();

    while (!open.empty())
        open.pop();

    exploredStates = 0;

    km = 0.0;

    start =
        problem.initialState;

    goal =
        problem.goalState;

    for (const auto& state :
         problem.states) {

        g[state.id] = INF;
        rhs[state.id] = INF;
    }

    rhs[goal] = 0.0;

    auto key =
        calculateKey(goal);

    open.push({
        key.first,
        key.second,
        goal
    });
}


// ==================================================
// Update vertex
// ==================================================

void DStarLitePlanner::updateVertex(
    uint64_t state
) {

    if (state != goal) {

        double best =
            INF;

        auto it =
            outgoing.find(state);

        if (it != outgoing.end()) {

            for (const Transition* transition :
                 it->second) {

                if (problem.isBadState(
                        transition->from))
                    continue;

                if (problem.isBadState(
                        transition->to))
                    continue;

                double cost =
                    transitionCost(
                        *transition
                    );

                if (std::isinf(cost))
                    continue;

                double candidate =
                    cost +
                    getG(
                        transition->to
                    );

                best =
                    std::min(
                        best,
                        candidate
                    );
            }
        }

        rhs[state] = best;
    }

    if (getG(state) !=
        getRHS(state)) {

        auto key =
            calculateKey(state);

        open.push({
            key.first,
            key.second,
            state
        });
    }
}


// ==================================================
// Compute shortest path
// ==================================================

void DStarLitePlanner::computeShortestPath() {

    size_t iterations = 0;

    const size_t MAX_ITERATIONS =
        1000000;

    while (!open.empty() &&
           iterations < MAX_ITERATIONS) {

        auto top =
            open.top();

        auto startKey =
            calculateKey(start);

        bool topLessThanStart =
            (top.k1 < startKey.first) ||
            (
                top.k1 == startKey.first &&
                top.k2 < startKey.second
            );

        if (!topLessThanStart &&
            getRHS(start) ==
            getG(start)) {

            break;
        }

        open.pop();

        uint64_t u =
            top.state;

        auto currentKey =
            calculateKey(u);

        bool stale =
            (top.k1 < currentKey.first) ||
            (
                top.k1 == currentKey.first &&
                top.k2 < currentKey.second
            );

        if (stale) {

            open.push({
                currentKey.first,
                currentKey.second,
                u
            });

            ++iterations;
            continue;
        }

        ++exploredStates;

        if (getG(u) >
            getRHS(u)) {

            g[u] =
                getRHS(u);

            auto it =
                incoming.find(u);

            if (it != incoming.end()) {

                for (const Transition* transition :
                     it->second) {

                    updateVertex(
                        transition->from
                    );
                }
            }

        } else {

            g[u] = INF;

            updateVertex(u);

            auto it =
                incoming.find(u);

            if (it != incoming.end()) {

                for (const Transition* transition :
                     it->second) {

                    updateVertex(
                        transition->from
                    );
                }
            }
        }

        ++iterations;
    }
}


// ==================================================
// Choose next state
// ==================================================

uint64_t DStarLitePlanner::chooseNextState(
    uint64_t current
) const {

    double bestValue =
        INF;

    uint64_t bestState =
        current;

    auto it =
        outgoing.find(current);

    if (it == outgoing.end())
        return current;

    for (const Transition* transition :
         it->second) {

        if (!transition->available)
            continue;

        if (problem.isBadState(
                transition->to))
            continue;

        double cost =
            transitionCost(
                *transition
            );

        if (std::isinf(cost))
            continue;

        double value =
            cost +
            getG(
                transition->to
            );

        if (value < bestValue) {

            bestValue =
                value;

            bestState =
                transition->to;
        }
    }

    return bestState;
}


// ==================================================
// Approximate memory usage
// ==================================================

size_t DStarLitePlanner::approximateMemoryUsage()
    const {

    size_t memory = 0;

    memory +=
        problem.states.size()
        * sizeof(State);

    memory +=
        problem.transitions.size()
        * sizeof(Transition);

    memory +=
        g.size()
        * (sizeof(uint64_t) +
           sizeof(double) +
           32);

    memory +=
        rhs.size()
        * (sizeof(uint64_t) +
           sizeof(double) +
           32);

    memory +=
        open.size()
        * sizeof(QueueNode);

    for (const auto& pair :
         outgoing) {

        memory +=
            pair.second.size()
            * sizeof(const Transition*);
    }

    for (const auto& pair :
         incoming) {

        memory +=
            pair.second.size()
            * sizeof(const Transition*);
    }

    return memory;
}


// ==================================================
// Construct planning result
// ==================================================

PlanningResult DStarLitePlanner::constructResult(
    double elapsedTimeMs
) const {

    PlanningResult result;

    result.planningTimeMs =
        elapsedTimeMs;

    result.exploredStates =
        exploredStates;

    result.memoryUsageBytes =
        approximateMemoryUsage();

    if (problem.isBadState(
            problem.initialState) ||
        problem.isBadState(
            problem.goalState)) {

        return result;
    }

    if (std::isinf(
            getG(start))) {

        return result;
    }

    uint64_t current =
        start;

    std::unordered_set<uint64_t>
        visited;

    visited.insert(current);

    result.statePath.push_back(
        current
    );

    double minimumDistance =
        INF;

    double minimumSafetyAttribute =
        1.0;

    size_t safetyGuard =
        0;

    while (current != goal &&
           safetyGuard < problem.states.size() + 1) {

        ++safetyGuard;

        uint64_t next =
            chooseNextState(current);

        if (next == current)
            break;

        if (problem.isBadState(next))
            break;

        if (visited.count(next))
            break;

        const Transition* selected =
            nullptr;

        auto it =
            outgoing.find(current);

        if (it != outgoing.end()) {

            double bestValue =
                INF;

            for (const Transition* transition :
                 it->second) {

                if (transition->to != next)
                    continue;

                double value =
                    transitionCost(
                        *transition
                    ) +
                    getG(
                        transition->to
                    );

                if (value < bestValue) {

                    bestValue = value;

                    selected =
                        transition;
                }
            }
        }

        if (selected == nullptr)
            break;

        visited.insert(next);

        result.transitionPath.push_back(
            selected->id
        );

        result.totalCost +=
            selected->cost;

        minimumSafetyAttribute =
            std::min(
                minimumSafetyAttribute,
                selected->safety
            );

        double distance =
            problem.distanceToNearestBadState(
                next
            );

        minimumDistance =
            std::min(
                minimumDistance,
                distance
            );

        current =
            next;

        result.statePath.push_back(
            current
        );
    }

    result.success =
        (current == goal);

    if (std::isinf(minimumDistance)) {

        if (problem.badStates.empty())
            minimumDistance = 0.0;
        else
            minimumDistance = 0.0;
    }

    result.minimumSafetyDistance =
        minimumDistance;

    result.safetyScore =
        minimumSafetyAttribute;

    return result;
}


// ==================================================
// Main planning function
// ==================================================

PlanningResult DStarLitePlanner::plan(
    const PlanningProblem& inputProblem
) {

    auto begin =
        std::chrono::high_resolution_clock::now();

    problem =
        inputProblem;

    buildGraph();

    initialize();

    computeShortestPath();

    auto end =
        std::chrono::high_resolution_clock::now();

    double elapsed =
        std::chrono::duration<
            double,
            std::milli
        >(end - begin).count();

    return constructResult(
        elapsed
    );
}


// ==================================================
// Replanning
// ==================================================

PlanningResult DStarLitePlanner::replan() {

    auto begin =
        std::chrono::high_resolution_clock::now();

    buildGraph();

    initialize();

    computeShortestPath();

    auto end =
        std::chrono::high_resolution_clock::now();

    double elapsed =
        std::chrono::duration<
            double,
            std::milli
        >(end - begin).count();

    PlanningResult result =
        constructResult(
            elapsed
        );

    result.replanningTimeMs =
        elapsed;

    return result;
}


// ==================================================
// Goal update
// ==================================================

void DStarLitePlanner::updateGoal(
    uint64_t newGoal
) {

    problem.goalState =
        newGoal;
}


// ==================================================
// Transition availability update
// ==================================================

void DStarLitePlanner::updateTransition(
    uint64_t transitionId,
    bool available
) {

    for (auto& transition :
         problem.transitions) {

        if (transition.id ==
            transitionId) {

            transition.available =
                available;

            return;
        }
    }
}


// ==================================================
// Bad state update
// ==================================================

void DStarLitePlanner::updateBadStates(
    const std::vector<uint64_t>& badStates
) {

    problem.badStates =
        badStates;
}


// ==================================================
// Add transition
// ==================================================

void DStarLitePlanner::addTransition(
    const Transition& transition
) {

    problem.transitions.push_back(
        transition
    );
}


// ==================================================
// Remove transition
// ==================================================

void DStarLitePlanner::removeTransition(
    uint64_t transitionId
) {

    problem.transitions.erase(
        std::remove_if(
            problem.transitions.begin(),
            problem.transitions.end(),
            [transitionId](
                const Transition& transition
            ) {
                return transition.id ==
                       transitionId;
            }
        ),
        problem.transitions.end()
    );
}