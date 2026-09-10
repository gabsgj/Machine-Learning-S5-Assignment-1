#include "Safety.h"
#include <cmath>
#include <limits>
#include <algorithm>

double euclideanDistance(
    const State& a,
    const State& b
) {
    double sum = 0.0;

    std::size_t dimensions = std::min(
        a.embedding.size(),
        b.embedding.size()
    );

    for (std::size_t i = 0; i < dimensions; i++) {

        double difference =
            a.embedding[i] - b.embedding[i];

        sum += difference * difference;
    }

    return std::sqrt(sum);
}


double calculateSafetyDistance(
    const std::vector<uint64_t>& path,
    const PlanningProblem& problem
) {
    if (path.empty() || problem.badStates.empty()) {
        return std::numeric_limits<double>::infinity();
    }

    double minimumDistance =
        std::numeric_limits<double>::infinity();

    for (uint64_t stateID : path) {

        const State* currentState = nullptr;

        for (const State& state : problem.states) {

            if (state.id == stateID) {
                currentState = &state;
                break;
            }
        }

        if (currentState == nullptr)
            continue;

        for (uint64_t badID : problem.badStates) {

            const State* badState = nullptr;

            for (const State& state : problem.states) {

                if (state.id == badID) {
                    badState = &state;
                    break;
                }
            }

            if (badState == nullptr)
                continue;

            double distance =
                euclideanDistance(
                    *currentState,
                    *badState
                );

            if (distance < minimumDistance) {
                minimumDistance = distance;
            }
        }
    }

    return minimumDistance;
}