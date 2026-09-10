#include "environment.hpp"
#include "heuristics.hpp"
#include <limits>

void Environment::loadProblem(const PlanningProblem &problem)
{
    goalId = problem.goalState;
    stateMap.clear();
    adjList.clear();
    badStatesSet.clear();

    for (const auto &s : problem.states)
    {
        stateMap[s.id] = s;
    }
    for (const auto &t : problem.transitions)
    {
        if (t.available)
        {
            adjList[t.from].push_back(t);
        }
    }
    for (uint64_t bs : problem.badStates)
    {
        badStatesSet.insert(bs);
    }
}

void Environment::addTransition(const Transition &t)
{
    if (t.available)
    {
        adjList[t.from].push_back(t);
    }
}

void Environment::removeTransition(uint64_t from, uint64_t to)
{
    if (adjList.find(from) == adjList.end())
        return;

    auto &neighbors = adjList[from];
    for (auto it = neighbors.begin(); it != neighbors.end(); ++it)
    {
        if (it->to == to)
        {
            neighbors.erase(it);
            break;
        }
    }
}

std::vector<Transition> Environment::getNeighbors(uint64_t stateId)
{
    if (badStatesSet.count(stateId))
        return {};
    return adjList[stateId];
}

State Environment::getState(uint64_t id)
{
    return stateMap[id];
}

double Environment::getSafetyMargin(uint64_t stateId)
{
    if (badStatesSet.empty())
        return 1000.0;

    double min_dist = std::numeric_limits<double>::infinity();
    State current = getState(stateId);

    for (uint64_t badId : badStatesSet)
    {
        double dist = Heuristics::euclideanDistance(current, getState(badId));
        if (dist < min_dist)
            min_dist = dist;
    }
    return min_dist;
}