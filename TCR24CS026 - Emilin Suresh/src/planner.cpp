#include "../include/planner.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>
#include <unordered_set>


Planner::Planner()
    : explored(0),
      dimensions(0)
{
}


/*
 * Initialize all internal data structures.
 */
void Planner::initialize(
    const PlanningProblem& problem)
{
    currentProblem = problem;

    g.clear();
    rhs.clear();
    versions.clear();

    outgoing.clear();
    incoming.clear();

    transitionIndex.clear();
    stateMap.clear();

    badSet.clear();

    explored = 0;
    dimensions = 0;

    while (!open.empty())
    {
        open.pop();
    }

    /*
     * Store states.
     */
    for (const State& state : currentProblem.states)
    {
        stateMap[state.id] = state;

        if (dimensions == 0)
        {
            dimensions = state.embedding.size();
        }
    }

    /*
     * Store bad states.
     */
    for (std::uint64_t id : currentProblem.badStates)
    {
        badSet.insert(id);
    }

    buildGraph();


    /*
     * Initialize g and rhs.
     */
    for (const State& state : currentProblem.states)
    {
        g[state.id] = INF;
        rhs[state.id] = INF;
        versions[state.id] = 0;
    }


    /*
     * LPA* starts at the initial state.
     *
     * rhs(start) = 0
     */
    rhs[currentProblem.initialState] = 0.0;

    pushOpen(currentProblem.initialState);
}


/*
 * Build adjacency lists.
 */
void Planner::buildGraph()
{
    for (std::size_t i = 0;
         i < currentProblem.transitions.size();
         ++i)
    {
        const Transition& t =
            currentProblem.transitions[i];

        outgoing[t.from].push_back(i);
        incoming[t.to].push_back(i);

        transitionIndex[t.id] = i;
    }
}


/*
 * Check whether a state is bad.
 */
bool Planner::isBad(
    std::uint64_t stateId) const
{
    return badSet.find(stateId) != badSet.end();
}


/*
 * Check whether a state is usable.
 */
bool Planner::isValidState(
    std::uint64_t stateId) const
{
    return stateMap.find(stateId) != stateMap.end()
           && !isBad(stateId);
}


/*
 * Euclidean distance between two Cartesian states.
 */
double Planner::euclideanDistance(
    std::uint64_t a,
    std::uint64_t b) const
{
    auto itA = stateMap.find(a);
    auto itB = stateMap.find(b);

    if (itA == stateMap.end() ||
        itB == stateMap.end())
    {
        return INF;
    }

    const std::vector<double>& x =
        itA->second.embedding;

    const std::vector<double>& y =
        itB->second.embedding;

    const std::size_t d =
        std::min(x.size(), y.size());

    double sum = 0.0;

    for (std::size_t i = 0; i < d; ++i)
    {
        double diff = x[i] - y[i];

        sum += diff * diff;
    }

    return std::sqrt(sum);
}


/*
 * Find the minimum Euclidean distance from a state
 * to any bad state.
 *
 * If there are no bad states, return a large value.
 */
double Planner::distanceToNearestBadState(
    std::uint64_t stateId) const
{
    if (badSet.empty())
    {
        return 1e9;
    }

    double minimum = INF;

    for (std::uint64_t bad : badSet)
    {
        if (bad == stateId)
        {
            return 0.0;
        }

        minimum =
            std::min(
                minimum,
                euclideanDistance(stateId, bad)
            );
    }

    return minimum;
}


/*
 * Heuristic function.
 *
 * We use Euclidean distance from the current state
 * to the goal.
 *
 * This is suitable for the Cartesian state space
 * specified in the assignment.
 */
double Planner::heuristic(
    std::uint64_t stateId) const
{
    return euclideanDistance(
        stateId,
        currentProblem.goalState
    );
}


/*
 * Safety penalty.
 *
 * A state closer to a bad state receives a larger
 * penalty.
 *
 * This allows the planner to prefer paths with
 * larger safety margins.
 */
double Planner::safetyPenalty(
    std::uint64_t from,
    std::uint64_t to) const
{
    double fromDistance =
        distanceToNearestBadState(from);

    double toDistance =
        distanceToNearestBadState(to);

    double minimumDistance =
        std::min(
            fromDistance,
            toDistance
        );

    /*
     * No bad states.
     */
    if (minimumDistance >= 1e8)
    {
        return 0.0;
    }

    /*
     * Very close to a bad state.
     */
    if (minimumDistance <= EPS)
    {
        return 1e6;
    }

    /*
     * Inverse-distance penalty.
     *
     * Larger distance -> smaller penalty.
     */
    const double SAFETY_WEIGHT = 2.0;

    return SAFETY_WEIGHT /
           (minimumDistance + 0.1);
}


/*
 * Reliability penalty.
 *
 * Reliability is assumed to be in [0,1].
 *
 * reliability = 1 -> no penalty
 * reliability = 0 -> maximum penalty
 */
double Planner::reliabilityPenalty(
    const Transition& transition) const
{
    double reliability =
        std::max(
            0.0,
            std::min(
                1.0,
                transition.reliability
            )
        );

    const double RELIABILITY_WEIGHT = 0.5;

    return RELIABILITY_WEIGHT *
           (1.0 - reliability);
}


/*
 * Calculate the actual planning cost of a transition.
 *
 * Base cost
 * + safety penalty
 * + reliability penalty
 */
double Planner::effectiveTransitionCost(
    const Transition& transition) const
{
    if (!transition.available)
    {
        return INF;
    }

    if (!isValidState(transition.from) ||
        !isValidState(transition.to))
    {
        return INF;
    }

    double result =
        transition.cost;

    result +=
        safetyPenalty(
            transition.from,
            transition.to
        );

    result +=
        reliabilityPenalty(
            transition
        );

    return result;
}


/*
 * Return outgoing transitions.
 */
std::vector<std::size_t>
Planner::getOutgoingTransitions(
    std::uint64_t stateId) const
{
    auto it =
        outgoing.find(stateId);

    if (it == outgoing.end())
    {
        return {};
    }

    return it->second;
}


/*
 * Return incoming transitions.
 */
std::vector<std::size_t>
Planner::getIncomingTransitions(
    std::uint64_t stateId) const
{
    auto it =
        incoming.find(stateId);

    if (it == incoming.end())
    {
        return {};
    }

    return it->second;
}


/*
 * Calculate rhs(s).
 *
 * rhs(start) = 0.
 *
 * For every other state:
 *
 * rhs(s) =
 * min [g(pred) + c(pred,s)]
 */
double Planner::calculateRHS(
    std::uint64_t stateId) const
{
    if (stateId ==
        currentProblem.initialState)
    {
        return 0.0;
    }

    if (isBad(stateId))
    {
        return INF;
    }

    double best = INF;

    std::vector<std::size_t> predecessors =
        getIncomingTransitions(stateId);

    for (std::size_t index : predecessors)
    {
        const Transition& t =
            currentProblem.transitions[index];

        if (!t.available)
        {
            continue;
        }

        if (isBad(t.from) ||
            isBad(t.to))
        {
            continue;
        }

        double edgeCost =
            effectiveTransitionCost(t);

        if (edgeCost >= INF)
        {
            continue;
        }

        auto it =
            g.find(t.from);

        if (it == g.end())
        {
            continue;
        }

        if (it->second >= INF)
        {
            continue;
        }

        double candidate =
            it->second + edgeCost;

        if (candidate < best)
        {
            best = candidate;
        }
    }

    return best;
}


/*
 * LPA* key.
 *
 * k1 = min(g, rhs) + h
 * k2 = min(g, rhs)
 */
std::pair<double, double>
Planner::calculateKey(
    std::uint64_t stateId) const
{
    double minimum =
        std::min(
            g.at(stateId),
            rhs.at(stateId)
        );

    return {
        minimum + heuristic(stateId),
        minimum
    };
}


/*
 * Insert a state into OPEN.
 *
 * Lazy deletion is used.
 */
void Planner::pushOpen(
    std::uint64_t stateId)
{
    if (isBad(stateId))
    {
        return;
    }

    versions[stateId]++;

    auto key =
        calculateKey(stateId);

    open.emplace(
        stateId,
        key.first,
        key.second,
        versions[stateId]
    );
}


/*
 * Lazy deletion means we don't physically remove
 * elements from priority_queue.
 */
void Planner::removeFromOpen(
    std::uint64_t stateId)
{
    versions[stateId]++;
}


/*
 * Update a vertex according to LPA*.
 */
void Planner::updateVertex(
    std::uint64_t stateId)
{
    if (stateId !=
        currentProblem.initialState)
    {
        rhs[stateId] =
            calculateRHS(stateId);
    }

    removeFromOpen(stateId);

    if (std::fabs(g[stateId] -
                  rhs[stateId]) > EPS)
    {
        pushOpen(stateId);
    }
}


/*
 * Check whether a queue item is still current.
 */
bool Planner::queueItemIsCurrent(
    const QueueItem& item) const
{
    auto it =
        versions.find(item.state);

    if (it == versions.end())
    {
        return false;
    }

    return it->second ==
           item.version;
}


/*
 * LPA* ComputeShortestPath.
 */
void Planner::computeShortestPath()
{
    while (!open.empty())
    {
        QueueItem top =
            open.top();

        if (!queueItemIsCurrent(top))
        {
            open.pop();
            continue;
        }

        auto goalKey =
            calculateKey(
                currentProblem.goalState
            );

        bool queueKeyGreater =
            (top.k1 > goalKey.first + EPS)
            ||
            (
                std::fabs(
                    top.k1 -
                    goalKey.first
                ) <= EPS
                &&
                top.k2 >
                goalKey.second + EPS
            );

        bool goalConsistent =
            std::fabs(
                g[currentProblem.goalState] -
                rhs[currentProblem.goalState]
            ) <= EPS;

        if (!queueKeyGreater &&
            goalConsistent)
        {
            break;
        }

        open.pop();

        if (!queueItemIsCurrent(top))
        {
            continue;
        }

        std::uint64_t u =
            top.state;

        double oldG =
            g[u];

        double currentRHS =
            rhs[u];

        if (oldG >
            currentRHS)
        {
            /*
             * Make state consistent.
             */
            g[u] =
                currentRHS;

            explored++;

            /*
             * Update successors.
             */
            for (std::size_t index :
                 getOutgoingTransitions(u))
            {
                const Transition& t =
                    currentProblem.transitions[index];

                if (!t.available)
                {
                    continue;
                }

                if (isBad(t.to))
                {
                    continue;
                }

                updateVertex(t.to);
            }
        }
        else
        {
            /*
             * State became over-consistent.
             */
            g[u] = INF;

            updateVertex(u);

            for (std::size_t index :
                 getOutgoingTransitions(u))
            {
                const Transition& t =
                    currentProblem.transitions[index];

                if (!t.available)
                {
                    continue;
                }

                if (isBad(t.to))
                {
                    continue;
                }

                updateVertex(t.to);
            }
        }
    }
}


/*
 * Find the predecessor that generated the
 * best g-value for the state.
 */
std::uint64_t Planner::bestPredecessor(
    std::uint64_t stateId) const
{
    double best =
        INF;

    std::uint64_t bestState =
        stateId;

    for (std::size_t index :
         getIncomingTransitions(stateId))
    {
        const Transition& t =
            currentProblem.transitions[index];

        if (!t.available)
        {
            continue;
        }

        if (isBad(t.from) ||
            isBad(t.to))
        {
            continue;
        }

        double edgeCost =
            effectiveTransitionCost(t);

        if (edgeCost >= INF)
        {
            continue;
        }

        auto gIt =
            g.find(t.from);

        if (gIt == g.end())
        {
            continue;
        }

        if (gIt->second >= INF)
        {
            continue;
        }

        double candidate =
            gIt->second +
            edgeCost;

        if (candidate <
            best - EPS)
        {
            best =
                candidate;

            bestState =
                t.from;
        }
    }

    return bestState;
}


/*
 * Find transition from one state to another.
 */
std::uint64_t Planner::findTransitionBetween(
    std::uint64_t from,
    std::uint64_t to) const
{
    for (std::size_t index :
         getOutgoingTransitions(from))
    {
        const Transition& t =
            currentProblem.transitions[index];

        if (t.to == to)
        {
            return t.id;
        }
    }

    return 0;
}


/*
 * Reconstruct final state path.
 */
std::vector<std::uint64_t>
Planner::reconstructPath()
{
    std::vector<std::uint64_t> path;

    std::uint64_t current =
        currentProblem.goalState;

    std::unordered_set<
        std::uint64_t
    > visited;

    while (true)
    {
        if (visited.find(current) !=
            visited.end())
        {
            /*
             * Cycle detected.
             */
            return {};
        }

        visited.insert(current);

        path.push_back(current);

        if (current ==
            currentProblem.initialState)
        {
            break;
        }

        std::uint64_t predecessor =
            bestPredecessor(current);

        if (predecessor == current)
        {
            return {};
        }

        current =
            predecessor;
    }

    std::reverse(
        path.begin(),
        path.end()
    );

    return path;
}


/*
 * Estimate memory used by planner data structures.
 *
 * This is an approximate value and is intended
 * for experimental comparison.
 */
std::size_t Planner::estimateMemoryUsage() const
{
    std::size_t bytes = 0;

    bytes +=
        stateMap.size() *
        sizeof(State);

    bytes +=
        currentProblem.transitions.size() *
        sizeof(Transition);

    bytes +=
        g.size() *
        (sizeof(std::uint64_t) +
         sizeof(double));

    bytes +=
        rhs.size() *
        (sizeof(std::uint64_t) +
         sizeof(double));

    bytes +=
        versions.size() *
        (sizeof(std::uint64_t) +
         sizeof(std::size_t));

    bytes +=
        badSet.size() *
        sizeof(std::uint64_t);

    for (const auto& entry :
         outgoing)
    {
        bytes +=
            entry.second.size() *
            sizeof(std::size_t);
    }

    for (const auto& entry :
         incoming)
    {
        bytes +=
            entry.second.size() *
            sizeof(std::size_t);
    }

    return bytes;
}


/*
 * Create PlanningResult from current planner state.
 */
PlanningResult Planner::createResult(
    double planningTimeMs)
{
    PlanningResult result;

    result.planningTimeMs =
        planningTimeMs;

    result.exploredStates =
        explored;

    result.memoryUsageBytes =
        estimateMemoryUsage();


    /*
     * Goal unreachable.
     */
    if (g[currentProblem.goalState] >= INF)
    {
        result.success = false;
        return result;
    }


    std::vector<std::uint64_t> path =
        reconstructPath();

    if (path.empty())
    {
        result.success = false;
        return result;
    }


    /*
     * Verify that no bad state appears.
     */
    for (std::uint64_t state : path)
    {
        if (isBad(state))
        {
            result.success = false;
            return result;
        }
    }


    result.success = true;
    result.statePath = path;


    /*
     * Calculate real path metrics.
     */
    double totalCost = 0.0;

    double minimumSafety =
        INF;

    double reliabilityProduct =
        1.0;


    for (std::size_t i = 0;
         i + 1 < path.size();
         ++i)
    {
        std::uint64_t from =
            path[i];

        std::uint64_t to =
            path[i + 1];

        std::uint64_t transitionId =
            findTransitionBetween(
                from,
                to
            );

        result.transitionPath.push_back(
            transitionId
        );


        auto transitionIt =
            transitionIndex.find(
                transitionId
            );

        if (transitionIt ==
            transitionIndex.end())
        {
            continue;
        }

        const Transition& t =
            currentProblem.transitions[
                transitionIt->second
            ];

        totalCost +=
            t.cost;

        reliabilityProduct *=
            t.reliability;
    }


    /*
     * Calculate minimum distance to bad states.
     */
    for (std::uint64_t state :
         path)
    {
        minimumSafety =
            std::min(
                minimumSafety,
                distanceToNearestBadState(
                    state
                )
            );
    }


    if (minimumSafety >= 1e8)
    {
        minimumSafety = 0.0;
    }


    result.totalCost =
        totalCost;

    result.minimumSafetyDistance =
        minimumSafety;

    result.cumulativeReliability =
        reliabilityProduct;


    /*
     * Safety score combines minimum safety distance
     * and reliability.
     *
     * It is primarily useful for comparing paths.
     */
    result.safetyScore =
        minimumSafety *
        reliabilityProduct;


    return result;
}


/*
 * Main plan function.
 */
PlanningResult Planner::plan(
    const PlanningProblem& problem)
{
    auto start =
        std::chrono::high_resolution_clock::now();


    /*
     * Validate start and goal.
     */
    if (problem.states.empty())
    {
        return PlanningResult();
    }

    if (!isValidState(problem.initialState) ||
        !isValidState(problem.goalState))
    {
        /*
         * initialize has not yet been called,
         * so perform temporary validation below.
         */
    }


    initialize(problem);


    /*
     * Initial state itself cannot be bad.
     */
    if (isBad(currentProblem.initialState) ||
        isBad(currentProblem.goalState))
    {
        PlanningResult result;

        result.success = false;

        return result;
    }


    computeShortestPath();


    auto end =
        std::chrono::high_resolution_clock::now();


    double timeMs =
        std::chrono::duration<double,
                              std::milli>(
            end - start
        ).count();


    return createResult(timeMs);
}


/*
 * Change transition availability.
 */
bool Planner::updateTransition(
    PlanningProblem& problem,
    std::uint64_t transitionId,
    bool available)
{
    for (Transition& t :
         problem.transitions)
    {
        if (t.id == transitionId)
        {
            t.available =
                available;

            return true;
        }
    }

    return false;
}


/*
 * Add transition.
 */
void Planner::addTransition(
    PlanningProblem& problem,
    const Transition& transition)
{
    problem.transitions.push_back(
        transition
    );
}


/*
 * Remove transition.
 */
bool Planner::removeTransition(
    PlanningProblem& problem,
    std::uint64_t transitionId)
{
    auto it =
        std::remove_if(
            problem.transitions.begin(),
            problem.transitions.end(),
            [transitionId](const Transition& t)
            {
                return t.id ==
                       transitionId;
            }
        );

    if (it ==
        problem.transitions.end())
    {
        return false;
    }

    problem.transitions.erase(
        it,
        problem.transitions.end()
    );

    return true;
}


/*
 * Update goal.
 */
bool Planner::updateGoal(
    PlanningProblem& problem,
    std::uint64_t newGoal)
{
    bool exists = false;

    for (const State& state :
         problem.states)
    {
        if (state.id == newGoal)
        {
            exists = true;
            break;
        }
    }

    if (!exists)
    {
        return false;
    }

    if (std::find(
            problem.badStates.begin(),
            problem.badStates.end(),
            newGoal
        )
        != problem.badStates.end())
    {
        return false;
    }

    problem.goalState =
        newGoal;

    return true;
}


/*
 * Add bad state.
 */
void Planner::addBadState(
    PlanningProblem& problem,
    std::uint64_t stateId)
{
    if (std::find(
            problem.badStates.begin(),
            problem.badStates.end(),
            stateId
        )
        ==
        problem.badStates.end())
    {
        problem.badStates.push_back(
            stateId
        );
    }
}


/*
 * Remove bad state.
 */
bool Planner::removeBadState(
    PlanningProblem& problem,
    std::uint64_t stateId)
{
    auto it =
        std::find(
            problem.badStates.begin(),
            problem.badStates.end(),
            stateId
        );

    if (it ==
        problem.badStates.end())
    {
        return false;
    }

    problem.badStates.erase(it);

    return true;
}