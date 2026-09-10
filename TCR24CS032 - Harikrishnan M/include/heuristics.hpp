#pragma once
#include "planning_types.hpp"

class Heuristics
{
public:
    // Calculates Euclidean distance between two states
    static double euclideanDistance(const State &a, const State &b);

    // Combined heuristic for A* / LPA* evaluation
    static double calculate(const State &current, const State &goal);
};