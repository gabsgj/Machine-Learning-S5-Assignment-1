#include "DStarLitePlanner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

DStarLitePlanner::DStarLitePlanner()
    : currentProblem(nullptr),
      currentGoal(0) {
}

DStarLitePlanner::~DStarLitePlanner() = default;


double DStarLitePlanner::heuristic(
    const State& a,
    const State& b
) const {

    size_t dimensions =
        min(a.embedding.size(), b.embedding.size());

    double sum = 0.0;

    for (size_t i = 0; i < dimensions; ++i) {
        double difference =
            a.embedding[i] - b.embedding[i];

        sum += difference * difference;
    }

    return sqrt(sum);
}


bool DStarLitePlanner::isBadState(
    uint64_t stateId,
    const PlanningProblem& problem
) const {

    return find(
        problem.badStates.begin(),
        problem.badStates.end(),
        stateId
    ) != problem.badStates.end();
}


const State* DStarLitePlanner::findState(
    uint64_t stateId,
    const PlanningProblem& problem
) const {

    for (const State& state : problem.states) {

        if (state.id == stateId) {
            return &state;
        }
    }

    return nullptr;
}


const Transition* DStarLitePlanner::findTransition(
    uint64_t from,
    uint64_t to,
    const PlanningProblem& problem
) const {

    for (const Transition& transition :
         problem.transitions) {

        if (transition.from == from &&
            transition.to == to &&
            transition.available) {

            return &transition;
        }
    }

    return nullptr;
}


double DStarLitePlanner::safetyDistance(
    uint64_t stateId,
    const PlanningProblem& problem
) const {

    const State* state =
        findState(stateId, problem);

    if (state == nullptr) {
        return 0.0;
    }

    if (problem.badStates.empty()) {
        return numeric_limits<double>::infinity();
    }

    double minimumDistance =
        numeric_limits<double>::infinity();

    for (uint64_t badId : problem.badStates) {

        const State* badState =
            findState(badId, problem);

        if (badState == nullptr) {
            continue;
        }

        double distance =
            heuristic(*state, *badState);

        minimumDistance =
            min(minimumDistance, distance);
    }

    return minimumDistance;
}


PlanningResult DStarLitePlanner::plan(
    const PlanningProblem& problem
) {

    auto startTime =
        chrono::high_resolution_clock::now();

    currentProblem = &problem;
    currentGoal = problem.goalState;

    PlanningResult result;

    if (problem.states.empty()) {
        return result;
    }

    if (isBadState(problem.initialState, problem)) {
        return result;
    }

    if (isBadState(problem.goalState, problem)) {
        return result;
    }

    const State* startState =
        findState(problem.initialState, problem);

    const State* goalState =
        findState(problem.goalState, problem);

    if (startState == nullptr ||
        goalState == nullptr) {

        return result;
    }


    /*
        Dijkstra / A* style search.

        The transition score combines:

        1. Transition cost
        2. Safety penalty
        3. Reliability penalty

        Lower score is better.
    */

    struct Node {
        uint64_t state;
        double score;
        double cost;

        bool operator<(const Node& other) const {
            return score > other.score;
        }
    };


    priority_queue<Node> open;

    unordered_map<uint64_t, double> distance;
    unordered_map<uint64_t, uint64_t> parent;
    unordered_map<uint64_t, uint64_t> parentTransition;

    unordered_set<uint64_t> visited;


    for (const State& state : problem.states) {
        distance[state.id] =
            numeric_limits<double>::infinity();
    }


    distance[problem.initialState] = 0.0;

    open.push({
        problem.initialState,
        heuristic(*startState, *goalState),
        0.0
    });


    while (!open.empty()) {

        Node current = open.top();
        open.pop();

        if (visited.count(current.state)) {
            continue;
        }

        visited.insert(current.state);

        result.exploredStates++;


        if (current.state == problem.goalState) {
            break;
        }


        for (const Transition& transition :
             problem.transitions) {

            if (!transition.available) {
                continue;
            }

            if (transition.from != current.state) {
                continue;
            }

            uint64_t nextState =
                transition.to;


            // Never enter bad states.
            if (isBadState(nextState, problem)) {
                continue;
            }


            double safety =
                safetyDistance(nextState, problem);


            /*
                Safety penalty.

                A larger safety distance is better.

                When there is a finite distance, the
                penalty becomes smaller as distance grows.
            */

            double safetyPenalty = 0.0;

            if (isfinite(safety)) {

                safetyPenalty =
                    1.0 / (1.0 + safety);
            }


            /*
                Reliability penalty.

                High reliability should produce a smaller
                penalty.
            */

            double reliabilityPenalty =
                1.0 - transition.reliability;


            /*
                Combined edge score.

                Cost has the highest importance.
                Safety and reliability influence the
                search without preventing valid paths.
            */

            double edgeScore =
                transition.cost
                + 2.0 * safetyPenalty
                + 1.0 * reliabilityPenalty;


            double newDistance =
                distance[current.state]
                + edgeScore;


            if (newDistance <
                distance[nextState]) {

                distance[nextState] =
                    newDistance;

                parent[nextState] =
                    current.state;

                parentTransition[nextState] =
                    transition.id;


                const State* next =
                    findState(nextState, problem);


                double h = 0.0;

                if (next != nullptr) {

                    h =
                        heuristic(*next, *goalState);
                }


                open.push({
                    nextState,
                    newDistance + h,
                    newDistance
                });
            }
        }
    }


    /*
        Goal was not reached.
    */

    if (!visited.count(problem.goalState)) {

        auto endTime =
            chrono::high_resolution_clock::now();

        result.planningTimeMs =
            chrono::duration<double, milli>(
                endTime - startTime
            ).count();

        return result;
    }


    /*
        Reconstruct state path.
    */

    vector<uint64_t> reversedStates;
    vector<uint64_t> reversedTransitions;

    uint64_t current =
        problem.goalState;


    reversedStates.push_back(current);


    while (current != problem.initialState) {

        auto parentIt =
            parent.find(current);

        auto transitionIt =
            parentTransition.find(current);

        if (parentIt == parent.end() ||
            transitionIt == parentTransition.end()) {

            result.success = false;
            return result;
        }


        reversedTransitions.push_back(
            transitionIt->second
        );

        current =
            parentIt->second;

        reversedStates.push_back(current);
    }


    reverse(
        reversedStates.begin(),
        reversedStates.end()
    );

    reverse(
        reversedTransitions.begin(),
        reversedTransitions.end()
    );


    result.statePath =
        reversedStates;

    result.transitionPath =
        reversedTransitions;


    /*
        Calculate actual path metrics.
    */

    double totalCost = 0.0;
    double minimumSafety =
        numeric_limits<double>::infinity();

    double cumulativeReliability = 1.0;


    for (size_t i = 0;
         i < result.statePath.size();
         ++i) {

        uint64_t stateId =
            result.statePath[i];


        double distanceToBad =
            safetyDistance(stateId, problem);


        if (isfinite(distanceToBad)) {

            minimumSafety =
                min(
                    minimumSafety,
                    distanceToBad
                );
        }


        if (i < result.transitionPath.size()) {

            uint64_t transitionId =
                result.transitionPath[i];


            for (const Transition& transition :
                 problem.transitions) {

                if (transition.id ==
                    transitionId) {

                    totalCost +=
                        transition.cost;

                    cumulativeReliability *=
                        transition.reliability;

                    break;
                }
            }
        }
    }


    result.success = true;

    result.totalCost =
        totalCost;

    result.safetyScore =
        minimumSafety;

    result.cumulativeReliability =
        cumulativeReliability;


    auto endTime =
        chrono::high_resolution_clock::now();

    result.planningTimeMs =
        chrono::duration<double, milli>(
            endTime - startTime
        ).count();


    return result;
}


PlanningResult DStarLitePlanner::replan(
    const PlanningProblem& problem
) {

    /*
        Replanning uses the updated problem.

        The same planner object can therefore be reused
        when the environment changes.
    */

    return plan(problem);
}


void DStarLitePlanner::updateTransition(
    uint64_t transitionId,
    bool available
) {

    if (currentProblem == nullptr) {
        return;
    }

    PlanningProblem* mutableProblem =
        const_cast<PlanningProblem*>(
            currentProblem
        );


    for (Transition& transition :
         mutableProblem->transitions) {

        if (transition.id ==
            transitionId) {

            transition.available =
                available;

            break;
        }
    }
}


void DStarLitePlanner::updateGoal(
    uint64_t newGoal
) {

    currentGoal = newGoal;
}