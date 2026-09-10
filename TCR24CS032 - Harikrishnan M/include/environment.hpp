#pragma once
#include "planning_types.hpp"
#include <unordered_map>
#include <unordered_set>

class Environment
{
public:
    void loadProblem(const PlanningProblem &problem);

    // Dynamic update functions used in Tests 4, 5, and 6
    void updateGoal(uint64_t newGoal) { goalId = newGoal; }
    void addTransition(const Transition &t);
    void removeTransition(uint64_t from, uint64_t to);

    std::vector<Transition> getNeighbors(uint64_t stateId);
    double getSafetyMargin(uint64_t stateId);
    State getState(uint64_t id);
    uint64_t getGoal() const { return goalId; }

private:
    uint64_t goalId;
    std::unordered_map<uint64_t, State> stateMap;
    std::unordered_map<uint64_t, std::vector<Transition>> adjList;
    std::unordered_set<uint64_t> badStatesSet;
};