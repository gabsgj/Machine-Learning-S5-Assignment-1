#ifndef DSTAR_LITE_PLANNER_H
#define DSTAR_LITE_PLANNER_H

#include "Planner.h"

#include <cstdint>

class DStarLitePlanner : public Planner {
public:
    DStarLitePlanner();

    ~DStarLitePlanner() override;

    PlanningResult plan(
        const PlanningProblem& problem
    ) override;

    PlanningResult replan(
        const PlanningProblem& problem
    );

    void updateTransition(
        uint64_t transitionId,
        bool available
    );

    void updateGoal(
        uint64_t newGoal
    );

private:
    const PlanningProblem* currentProblem;

    uint64_t currentGoal;

    double heuristic(
        const State& a,
        const State& b
    ) const;

    double safetyDistance(
        uint64_t stateId,
        const PlanningProblem& problem
    ) const;

    bool isBadState(
        uint64_t stateId,
        const PlanningProblem& problem
    ) const;

    const State* findState(
        uint64_t stateId,
        const PlanningProblem& problem
    ) const;

    const Transition* findTransition(
        uint64_t from,
        uint64_t to,
        const PlanningProblem& problem
    ) const;
};

#endif