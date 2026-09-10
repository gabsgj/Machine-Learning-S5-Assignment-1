#ifndef PLANNER_H
#define PLANNER_H

#include "model.h"

#include <cstdint>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include<cmath>


/*
 * LPA* Safe Semantic Planner
 *
 * The planner:
 *  - avoids bad states
 *  - ignores unavailable transitions
 *  - considers transition cost
 *  - considers safety
 *  - considers reliability
 *  - uses Euclidean distance as heuristic
 *  - supports dynamic changes
 */
class Planner
{
public:

    Planner();

    /*
     * Main planning function.
     */
    PlanningResult plan(const PlanningProblem& problem);


    /*
     * Update an existing transition.
     */
    bool updateTransition(
        PlanningProblem& problem,
        std::uint64_t transitionId,
        bool available);


    /*
     * Add a new transition.
     */
    void addTransition(
        PlanningProblem& problem,
        const Transition& transition);


    /*
     * Remove a transition by ID.
     */
    bool removeTransition(
        PlanningProblem& problem,
        std::uint64_t transitionId);


    /*
     * Change the goal.
     */
    bool updateGoal(
        PlanningProblem& problem,
        std::uint64_t newGoal);


    /*
     * Add a bad state.
     */
    void addBadState(
        PlanningProblem& problem,
        std::uint64_t stateId);


    /*
     * Remove a bad state.
     */
    bool removeBadState(
        PlanningProblem& problem,
        std::uint64_t stateId);


private:

    static constexpr double INF =
        std::numeric_limits<double>::infinity();

    static constexpr double EPS = 1e-9;


    /*
     * Internal priority queue item.
     */
    struct QueueItem
    {
        std::uint64_t state;

        double k1;
        double k2;

        std::size_t version;

        QueueItem(
            std::uint64_t state_,
            double k1_,
            double k2_,
            std::size_t version_)
            : state(state_),
              k1(k1_),
              k2(k2_),
              version(version_)
        {
        }
    };


    /*
     * Priority queue comparator.
     *
     * std::priority_queue is a max heap by default,
     * so the comparator is reversed to obtain a min heap.
     */
    struct QueueCompare
    {
        bool operator()(
            const QueueItem& a,
            const QueueItem& b) const
        {
            if (std::fabs(a.k1 - b.k1) > EPS)
            {
                return a.k1 > b.k1;
            }

            return a.k2 > b.k2;
        }
    };


    PlanningProblem currentProblem;

    /*
     * LPA* values.
     */
    std::unordered_map<std::uint64_t, double> g;

    std::unordered_map<std::uint64_t, double> rhs;


    /*
     * Version number for lazy priority queue deletion.
     */
    std::unordered_map<std::uint64_t, std::size_t> versions;


    /*
     * OPEN list.
     */
    std::priority_queue<
        QueueItem,
        std::vector<QueueItem>,
        QueueCompare
    > open;


    /*
     * Adjacency lists.
     */
    std::unordered_map<
        std::uint64_t,
        std::vector<std::size_t>
    > outgoing;


    std::unordered_map<
        std::uint64_t,
        std::vector<std::size_t>
    > incoming;


    /*
     * Mapping transition ID -> transition index.
     */
    std::unordered_map<
        std::uint64_t,
        std::size_t
    > transitionIndex;


    /*
     * State lookup.
     */
    std::unordered_map<
        std::uint64_t,
        State
    > stateMap;


    /*
     * Set of bad states.
     */
    std::unordered_set<std::uint64_t> badSet;


    /*
     * Number of states expanded.
     */
    std::size_t explored;


    /*
     * Number of dimensions.
     */
    std::size_t dimensions;


private:

    void initialize(
        const PlanningProblem& problem);


    void buildGraph();


    bool isBad(
        std::uint64_t stateId) const;


    bool isValidState(
        std::uint64_t stateId) const;


    double euclideanDistance(
        std::uint64_t a,
        std::uint64_t b) const;


    double distanceToNearestBadState(
        std::uint64_t stateId) const;


    double heuristic(
        std::uint64_t stateId) const;


    double safetyPenalty(
        std::uint64_t from,
        std::uint64_t to) const;


    double reliabilityPenalty(
        const Transition& transition) const;


    double effectiveTransitionCost(
        const Transition& transition) const;


    std::vector<std::size_t> getOutgoingTransitions(
        std::uint64_t stateId) const;


    std::vector<std::size_t> getIncomingTransitions(
        std::uint64_t stateId) const;


    double calculateRHS(
        std::uint64_t stateId) const;


    std::pair<double, double> calculateKey(
        std::uint64_t stateId) const;


    void pushOpen(
        std::uint64_t stateId);


    void removeFromOpen(
        std::uint64_t stateId);


    void updateVertex(
        std::uint64_t stateId);


    void computeShortestPath();


    bool queueItemIsCurrent(
        const QueueItem& item) const;


    std::uint64_t bestPredecessor(
        std::uint64_t stateId) const;


    std::vector<std::uint64_t> reconstructPath();


    std::uint64_t findTransitionBetween(
        std::uint64_t from,
        std::uint64_t to) const;


    std::size_t estimateMemoryUsage() const;


    PlanningResult createResult(
        double planningTimeMs);
};

#endif