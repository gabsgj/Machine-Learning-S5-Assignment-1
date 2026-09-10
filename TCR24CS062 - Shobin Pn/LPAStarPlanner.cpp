#include "LPAStarPlanner.h"

#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <unordered_set>

const State* LPAStarPlanner::findState(
    const PlanningProblem& problem,
    uint64_t id
) const
{
    for (const auto& state : problem.states)
    {
        if (state.id == id)
        {
            return &state;
        }
    }

    return nullptr;
}


double LPAStarPlanner::heuristic(
    const State& a,
    const State& b
) const
{
    double sum = 0.0;

    size_t dimensions =
        std::min(
            a.embedding.size(),
            b.embedding.size()
        );

    for (size_t i = 0; i < dimensions; ++i)
    {
        double difference =
            a.embedding[i] - b.embedding[i];

        sum += difference * difference;
    }

    return std::sqrt(sum);
}


bool LPAStarPlanner::isBadState(
    const PlanningProblem& problem,
    uint64_t stateId
) const
{
    return std::find(
        problem.badStates.begin(),
        problem.badStates.end(),
        stateId
    ) != problem.badStates.end();
}


PlanningResult LPAStarPlanner::plan(
    const PlanningProblem& problem
)
{
    PlanningResult result;

    result.success = false;
    result.totalCost = 0.0;
    result.safetyScore = 1.0;

    if (problem.states.empty())
    {
        return result;
    }

    // Find the goal state.
    const State* goal =
        findState(problem, problem.goalState);

    if (goal == nullptr)
    {
        return result;
    }

    // Initialize LPA* values.
    g.clear();
    rhs.clear();

    const double INF =
        std::numeric_limits<double>::infinity();

    for (const auto& state : problem.states)
    {
        g[state.id] = INF;
        rhs[state.id] = INF;
    }

    rhs[problem.initialState] = 0.0;

    /*
        Priority queue entry:

        first  = estimated total cost
        second = state ID
    */
    using QueueEntry =
        std::pair<double, uint64_t>;

    std::priority_queue<
        QueueEntry,
        std::vector<QueueEntry>,
        std::greater<QueueEntry>
    > open;

    const State* start =
        findState(problem, problem.initialState);

    if (start == nullptr)
    {
        return result;
    }

    open.push({
        heuristic(*start, *goal),
        problem.initialState
    });

    // Keep track of the previous state.
    std::unordered_map<uint64_t, uint64_t> parent;

    // Standard graph search with LPA* g/rhs bookkeeping.
    while (!open.empty())
{
    auto currentEntry = open.top();
    open.pop();

    uint64_t current =
        currentEntry.second;

    // Make the current state consistent.
    if (rhs[current] < g[current])
    {
        g[current] = rhs[current];
    }

    if (current == problem.goalState)
    {
        break;
    }

        // Ignore bad states.
        if (isBadState(problem, current))
        {
            continue;
        }

        const State* currentState =
            findState(problem, current);

        if (currentState == nullptr)
        {
            continue;
        }

        for (const auto& transition :
             problem.transitions)
        {
            if (transition.from != current)
            {
                continue;
            }

            if (!transition.available)
            {
                continue;
            }

            uint64_t next =
                transition.to;

            // Never enter a bad state.
            if (isBadState(problem, next))
            {
                continue;
            }

            const State* nextState =
                findState(problem, next);

            if (nextState == nullptr)
            {
                continue;
            }

            /*
                Cost calculation.

                Lower transition cost is better.
                Low reliability adds a penalty.
                Low safety adds a penalty.
            */

            double safetyPenalty =
                (1.0 - transition.safety) * 10.0;

            double reliabilityPenalty =
                (1.0 - transition.reliability) * 5.0;

            double newCost =
                g[current]
                + transition.cost
                + safetyPenalty
                + reliabilityPenalty;

            if (newCost < g[next])
            {
                g[next] = newCost;

                rhs[next] = newCost;

                parent[next] = current;

                double estimatedCost =
                    newCost
                    + heuristic(
                        *nextState,
                        *goal
                    );

                open.push({
                    estimatedCost,
                    next
                });
            }
        }
    }

    // Did we reach the goal?
    if (g[problem.goalState] == INF)
    {
        return result;
    }

    // Reconstruct path.
    std::vector<uint64_t> reversedPath;

    uint64_t current =
        problem.goalState;

    reversedPath.push_back(current);

    while (current != problem.initialState)
    {
        if (parent.find(current)
            == parent.end())
        {
            result.success = false;
            return result;
        }

        current = parent[current];

        reversedPath.push_back(current);
    }

    std::reverse(
        reversedPath.begin(),
        reversedPath.end()
    );

    result.statePath =
        reversedPath;

    // Calculate actual path information.
    double actualCost = 0.0;

    double minimumSafety =
        1.0;

    for (size_t i = 0;
         i + 1 < result.statePath.size();
         ++i)
    {
        uint64_t from =
            result.statePath[i];

        uint64_t to =
            result.statePath[i + 1];

        for (const auto& transition :
             problem.transitions)
        {
            if (transition.from == from &&
                transition.to == to)
            {
                result.transitionPath.push_back(
                    transition.id
                );

                actualCost +=
                    transition.cost;

                minimumSafety =
                    std::min(
                        minimumSafety,
                        transition.safety
                    );

                break;
            }
        }
    }

    result.totalCost =
        actualCost;

    result.safetyScore =
        minimumSafety;

    result.success = true;

    return result;
}