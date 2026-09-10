#include "../include/heuristic.h"
#include "../include/safety.h"

double heuristic(const State& s, const State& goal) {
    return euclideanDistance(s.embedding, goal.embedding);
}