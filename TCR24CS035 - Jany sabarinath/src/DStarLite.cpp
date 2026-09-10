#include "DStarLite.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

DStarLite::DStarLite(double safetyWeight)
    : start(0),
      goal(0),
      km(0.0),
      safetyWeight(safetyWeight) {
}

double DStarLite::INF() const {
    return std::numeric_limits<double>::infinity();
}

bool DStarLite::stateExists(uint64_t stateId) const {
    for (const auto& state : problem.states) {
        if (state.id == stateId)
            return true;
    }

    return false;
}

bool DStarLite::isBadState(uint64_t stateId) const {
    return badStateSet.find(stateId) != badStateSet.end();
}

double DStarLite::getG(uint64_t stateId) const {
    auto it = g.find(stateId);

    if (it == g.end())
        return INF();

    return it->second;
}

double DStarLite::getRHS(uint64_t stateId) const {
    auto it = rhs.find(stateId);

    if (it == rhs.end())
        return INF();

    return it->second;
}

void DStarLite::setG(
    uint64_t stateId,
    double value
) {
    g[stateId] = value;
}

void DStarLite::setRHS(
    uint64_t stateId,
    double value
) {
    rhs[stateId] = value;
}

double DStarLite::heuristic(
    uint64_t a,
    uint64_t b
) const {

    const State* stateA = nullptr;
    const State* stateB = nullptr;

    for (const auto& state : problem.states) {

        if (state.id == a)
            stateA = &state;

        if (state.id == b)
            stateB = &state;
    }

    if (!stateA || !stateB)
        return INF();

    size_t dimensions =
        std::min(
            stateA->embedding.size(),
            stateB->embedding.size()
        );

    double sum = 0.0;

    for (size_t i = 0; i < dimensions; i++) {

        double difference =
            stateA->embedding[i] -
            stateB->embedding[i];

        sum += difference * difference;
    }

    return std::sqrt(sum);
}

double DStarLite::distanceToNearestBadState(
    uint64_t stateId
) const {

    if (badStateSet.empty())
        return INF();

    double minimumDistance = INF();

    for (uint64_t badId : badStateSet) {

        if (stateId == badId)
            return 0.0;

        double distance =
            heuristic(stateId, badId);

        minimumDistance =
            std::min(
                minimumDistance,
                distance
            );
    }

    return minimumDistance;
}

double DStarLite::edgeCost(
    const Transition& transition
) const {

    if (!transition.available)
        return INF();

    if (isBadState(transition.to))
        return INF();

    double safetyDistance =
        distanceToNearestBadState(transition.to);

    /*
       Safety penalty:

       penalty = safetyWeight / distance

       A state closer to a bad state
       receives a larger penalty.
    */

    double safetyPenalty = 0.0;

    if (std::isfinite(safetyDistance)) {

        const double EPSILON = 0.001;

        safetyPenalty =
            safetyWeight /
            (safetyDistance + EPSILON);
    }

    /*
       Reliability penalty.

       Higher reliability produces
       a smaller penalty.
    */

    double reliabilityPenalty =
        1.0 - transition.reliability;

    return transition.cost
           + safetyPenalty
           + reliabilityPenalty;
}

DStarLite::Key DStarLite::calculateKey(
    uint64_t stateId
) const {

    double minimum =
        std::min(
            getG(stateId),
            getRHS(stateId)
        );

    return {
        minimum + heuristic(start, stateId) + km,
        minimum
    };
}

void DStarLite::initialize() {

    g.clear();
    rhs.clear();

    while (!openList.empty())
        openList.pop();

    for (const auto& state : problem.states) {

        g[state.id] = INF();
        rhs[state.id] = INF();
    }

    rhs[goal] = 0.0;

    openList.push({
        goal,
        calculateKey(goal)
    });
}

void DStarLite::updateVertex(
    uint64_t stateId
) {

    if (stateId != goal) {

        double minimum =
            INF();

        auto it =
            incoming.find(stateId);

        if (it != incoming.end()) {

            for (const auto& transition : it->second) {

                if (!transition.available)
                    continue;

                if (isBadState(transition.from))
                    continue;

                double cost =
                    edgeCost(transition);

                if (!std::isfinite(cost))
                    continue;

                double candidate =
                    getG(transition.from)
                    + cost;

                /*
                   We calculate rhs using
                   the best predecessor.
                */

                minimum =
                    std::min(
                        minimum,
                        candidate
                    );
            }
        }

        /*
           For D* Lite orientation,
           rhs(u) should be based on successors.

           Therefore we calculate it again
           using outgoing edges.
        */

        minimum = INF();

        auto outgoingIt =
            outgoing.find(stateId);

        if (outgoingIt != outgoing.end()) {

            for (const auto& transition :
                 outgoingIt->second) {

                if (!transition.available)
                    continue;

                if (isBadState(transition.to))
                    continue;

                double cost =
                    edgeCost(transition);

                if (!std::isfinite(cost))
                    continue;

                double candidate =
                    cost +
                    getG(transition.to);

                minimum =
                    std::min(
                        minimum,
                        candidate
                    );
            }
        }

        rhs[stateId] = minimum;
    }

    if (getG(stateId) != getRHS(stateId)) {

        openList.push({
            stateId,
            calculateKey(stateId)
        });
    }
}

void DStarLite::computeShortestPath() {

    while (!openList.empty()) {

        QueueNode current =
            openList.top();

        openList.pop();

        uint64_t state =
            current.state;

        Key newKey =
            calculateKey(state);

        /*
           Ignore outdated queue entries.
        */

        if (
            current.key.first > newKey.first ||
            (
                current.key.first ==
                newKey.first &&
                current.key.second >
                newKey.second
            )
        ) {

            openList.push({
                state,
                newKey
            });

            continue;
        }

        double currentG =
            getG(state);

        double currentRHS =
            getRHS(state);

        if (currentG > currentRHS) {

            setG(
                state,
                currentRHS
            );

            auto it =
                incoming.find(state);

            if (it != incoming.end()) {

                for (const auto& transition :
                     it->second) {

                    updateVertex(
                        transition.from
                    );
                }
            }

        } else {

            setG(
                state,
                INF()
            );

            updateVertex(state);

            auto it =
                incoming.find(state);

            if (it != incoming.end()) {

                for (const auto& transition :
                     it->second) {

                    updateVertex(
                        transition.from
                    );
                }
            }
        }

        /*
           Stop when the start state is consistent
           and no better state remains.
        */

        Key startKey =
            calculateKey(start);

        if (
            getG(start) ==
            getRHS(start)
        ) {
            break;
        }
    }
}

uint64_t DStarLite::getBestSuccessor(
    uint64_t stateId
) const {

    auto it =
        outgoing.find(stateId);

    if (it == outgoing.end())
        return UINT64_MAX;

    double bestValue =
        INF();

    uint64_t bestState =
        UINT64_MAX;

    for (const auto& transition :
         it->second) {

        if (!transition.available)
            continue;

        if (isBadState(transition.to))
            continue;

        double cost =
            edgeCost(transition);

        if (!std::isfinite(cost))
            continue;

        double value =
            cost +
            getG(transition.to);

        if (value < bestValue) {

            bestValue = value;

            bestState =
                transition.to;
        }
    }

    return bestState;
}

std::vector<uint64_t>
DStarLite::reconstructPath(
    int& exploredStates
) {

    std::vector<uint64_t> path;

    if (!std::isfinite(getG(start)))
        return path;

    uint64_t current =
        start;

    std::unordered_set<uint64_t>
        visited;

    while (current != goal) {

        if (visited.find(current)
            != visited.end()) {

            break;
        }

        visited.insert(current);

        path.push_back(current);

        uint64_t next =
            getBestSuccessor(current);

        if (next == UINT64_MAX)
            break;

        current = next;

        exploredStates++;
    }

    if (current == goal)
        path.push_back(goal);

    return path;
}

PlanningResult DStarLite::plan(
    const PlanningProblem& newProblem
) {

    problem = newProblem;

    start =
        problem.initialState;

    goal =
        problem.goalState;

    badStateSet.clear();

    for (uint64_t bad :
         problem.badStates) {

        badStateSet.insert(bad);
    }

    outgoing.clear();
    incoming.clear();

    for (const auto& transition :
         problem.transitions) {

        outgoing[
            transition.from
        ].push_back(transition);

        incoming[
            transition.to
        ].push_back(transition);
    }

    auto begin =
        std::chrono::high_resolution_clock::now();

    initialize();

    computeShortestPath();

    int exploredStates = 0;

    std::vector<uint64_t> path =
        reconstructPath(
            exploredStates
        );

    auto end =
        std::chrono::high_resolution_clock::now();

    double planningTime =
        std::chrono::duration<double, std::milli>(
            end - begin
        ).count();

    PlanningResult result;

    result.statePath = path;

    result.exploredStates =
        exploredStates;

    result.planningTimeMs =
        planningTime;

    if (
        path.empty() ||
        path.back() != goal
    ) {

        result.success = false;

        return result;
    }

    result.success = true;

    result.totalCost = 0.0;

    result.safetyScore = INF();

    result.reliability = 1.0;

    for (size_t i = 0;
         i + 1 < path.size();
         i++) {

        uint64_t from =
            path[i];

        uint64_t to =
            path[i + 1];

        for (const auto& transition :
             outgoing[from]) {

            if (
                transition.to == to &&
                transition.available
            ) {

                result.totalCost +=
                    transition.cost;

                result.reliability *=
                    transition.reliability;

                break;
            }
        }

        double safety =
            distanceToNearestBadState(
                from
            );

        result.safetyScore =
            std::min(
                result.safetyScore,
                safety
            );
    }

    double goalSafety =
        distanceToNearestBadState(goal);

    result.safetyScore =
        std::min(
            result.safetyScore,
            goalSafety
        );

    return result;
}

void DStarLite::updateTransition(
    uint64_t transitionId,
    bool available
) {

    for (auto& transition :
         problem.transitions) {

        if (transition.id ==
            transitionId) {

            transition.available =
                available;
        }
    }

    outgoing.clear();
    incoming.clear();

    for (const auto& transition :
         problem.transitions) {

        outgoing[
            transition.from
        ].push_back(transition);

        incoming[
            transition.to
        ].push_back(transition);
    }

    /*
       Update affected vertices.
    */

    for (const auto& transition :
         problem.transitions) {

        if (transition.id ==
            transitionId) {

            updateVertex(
                transition.from
            );

            updateVertex(
                transition.to
            );
        }
    }
}

void DStarLite::addTransition(
    const Transition& transition
) {

    problem.transitions.push_back(
        transition
    );

    outgoing[
        transition.from
    ].push_back(
        transition
    );

    incoming[
        transition.to
    ].push_back(
        transition
    );

    updateVertex(
        transition.from
    );
}

void DStarLite::removeTransition(
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

    outgoing.clear();
    incoming.clear();

    for (const auto& transition :
         problem.transitions) {

        outgoing[
            transition.from
        ].push_back(
            transition
        );

        incoming[
            transition.to
        ].push_back(
            transition
        );
    }

    initialize();

    computeShortestPath();
}

void DStarLite::updateGoal(
    uint64_t newGoal
) {

    goal =
        newGoal;

    problem.goalState =
        newGoal;

    initialize();

    computeShortestPath();
}

void DStarLite::updateBadStates(
    const std::vector<uint64_t>& newBadStates
) {

    problem.badStates =
        newBadStates;

    badStateSet.clear();

    for (uint64_t bad :
         newBadStates) {

        badStateSet.insert(bad);
    }

    initialize();

    computeShortestPath();
}

PlanningResult DStarLite::replan() {

    auto begin =
        std::chrono::high_resolution_clock::now();

    computeShortestPath();

    int exploredStates = 0;

    std::vector<uint64_t> path =
        reconstructPath(
            exploredStates
        );

    auto end =
        std::chrono::high_resolution_clock::now();

    PlanningResult result;

    result.statePath =
        path;

    result.exploredStates =
        exploredStates;

    result.replanningTimeMs =
        std::chrono::duration<double, std::milli>(
            end - begin
        ).count();

    if (
        path.empty() ||
        path.back() != goal
    ) {

        result.success = false;

        return result;
    }

    result.success = true;

    result.totalCost = 0.0;

    result.safetyScore = INF();

    result.reliability = 1.0;

    for (size_t i = 0;
         i + 1 < path.size();
         i++) {

        uint64_t from =
            path[i];

        uint64_t to =
            path[i + 1];

        for (const auto& transition :
             outgoing[from]) {

            if (
                transition.to == to &&
                transition.available
            ) {

                result.totalCost +=
                    transition.cost;

                result.reliability *=
                    transition.reliability;

                break;
            }
        }

        result.safetyScore =
            std::min(
                result.safetyScore,
                distanceToNearestBadState(
                    from
                )
            );
    }

    result.safetyScore =
        std::min(
            result.safetyScore,
            distanceToNearestBadState(
                goal
            )
        );

    return result;
}