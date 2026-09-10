#include "planner.hpp"
#include "heuristics.hpp"
#include <algorithm>
#include <limits> // Fixes the std::numeric_limits error

const double INF = std::numeric_limits<double>::infinity();

double LPAPlanner::calcKey(uint64_t id)
{
    double min_val = std::min(g[id], rhs[id]);
    double h = Heuristics::calculate(env.getState(id), env.getState(env.getGoal()));
    return min_val + h;
}

PlanningResult LPAPlanner::plan(const PlanningProblem &problem)
{
    env.loadProblem(problem);
    startId = problem.initialState;
    return replan();
}

PlanningResult LPAPlanner::replan()
{
    g.clear();
    rhs.clear();
    came_from.clear();
    transition_used.clear();
    U.clear();

    // Simplification of LPA* to function reliably as A* for initialization
    rhs[startId] = 0;
    g[startId] = INF;
    U.insert({calcKey(startId), startId});

    while (!U.empty())
    {
        auto top = *U.begin();
        U.erase(U.begin());
        uint64_t u = top.second;

        if (u == env.getGoal())
            break;

        if (g[u] > rhs[u])
        {
            g[u] = rhs[u];
            for (const auto &trans : env.getNeighbors(u))
            {
                uint64_t v = trans.to;

                // Penalize moving close to bad states to satisfy safety objective
                double safety = env.getSafetyMargin(v);
                double penalty = (safety < 2.0) ? (2.0 - safety) * 5.0 : 0.0;

                double cost = g[u] + trans.cost + penalty;
                if (rhs.find(v) == rhs.end())
                    rhs[v] = INF;
                if (g.find(v) == g.end())
                    g[v] = INF;

                if (rhs[v] > cost)
                {
                    rhs[v] = cost;
                    came_from[v] = u;
                    transition_used[v] = trans.id;
                    U.insert({calcKey(v), v});
                }
            }
        }
    }
    return extractResult();
}

PlanningResult LPAPlanner::extractResult()
{
    PlanningResult res;
    res.success = (came_from.find(env.getGoal()) != came_from.end() || startId == env.getGoal());
    if (!res.success)
        return res;

    uint64_t curr = env.getGoal();
    res.totalCost = 0.0;
    res.safetyScore = 1000.0;

    while (curr != startId)
    {
        res.statePath.push_back(curr);
        res.transitionPath.push_back(transition_used[curr]);
        res.safetyScore = std::min(res.safetyScore, env.getSafetyMargin(curr));
        curr = came_from[curr];
    }
    res.statePath.push_back(startId);
    std::reverse(res.statePath.begin(), res.statePath.end());
    std::reverse(res.transitionPath.begin(), res.transitionPath.end());
    res.totalCost = g[env.getGoal()]; // Includes safety penalties

    return res;
}