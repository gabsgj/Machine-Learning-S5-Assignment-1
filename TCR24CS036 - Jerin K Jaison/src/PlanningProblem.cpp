#include "PlanningProblem.h"

#include <cmath>
#include <limits>
#include <algorithm>

PlanningProblem::PlanningProblem()
    : initialState(0),
      goalState(0) {
}

bool PlanningProblem::isBadState(
    uint64_t stateId
) const {

    for (uint64_t badState : badStates) {

        if (badState == stateId)
            return true;
    }

    return false;
}

const State* PlanningProblem::getState(
    uint64_t stateId
) const {

    for (const auto& state : states) {

        if (state.id == stateId)
            return &state;
    }

    return nullptr;
}

double PlanningProblem::distanceBetweenStates(
    uint64_t stateA,
    uint64_t stateB
) const {

    const State* a = getState(stateA);
    const State* b = getState(stateB);

    if (a == nullptr || b == nullptr)
        return std::numeric_limits<double>::infinity();

    size_t dimensions =
        std::min(
            a->embedding.size(),
            b->embedding.size()
        );

    double sum = 0.0;

    for (size_t i = 0;
         i < dimensions;
         ++i) {

        double difference =
            a->embedding[i] -
            b->embedding[i];

        sum += difference * difference;
    }

    return std::sqrt(sum);
}

double PlanningProblem::distanceToNearestBadState(
    uint64_t stateId
) const {

    if (badStates.empty())
        return std::numeric_limits<double>::infinity();

    double minimumDistance =
        std::numeric_limits<double>::infinity();

    for (uint64_t badState : badStates) {

        double distance =
            distanceBetweenStates(
                stateId,
                badState
            );

        minimumDistance =
            std::min(
                minimumDistance,
                distance
            );
    }

    return minimumDistance;
}