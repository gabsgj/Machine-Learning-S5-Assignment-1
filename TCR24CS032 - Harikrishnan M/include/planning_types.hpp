#pragma once
#include <vector>
#include <cstdint>

class State
{
public:
    uint64_t id;
    std::vector<double> embedding;
};

class Transition
{
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

class PlanningProblem
{
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

class PlanningResult
{
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
};