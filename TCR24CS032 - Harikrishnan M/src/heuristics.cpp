#include "heuristics.hpp"
#include <cmath>

double Heuristics::euclideanDistance(const State &a, const State &b)
{
    double dist = 0.0;
    for (size_t i = 0; i < a.embedding.size(); ++i)
    {
        double diff = a.embedding[i] - b.embedding[i];
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

double Heuristics::calculate(const State &current, const State &goal)
{
    return euclideanDistance(current, goal);
}