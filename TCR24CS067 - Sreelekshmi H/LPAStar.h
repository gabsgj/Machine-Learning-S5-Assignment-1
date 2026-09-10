#ifndef LPASTAR_H
#define LPASTAR_H

#include <cstdint>
#include <vector>
#include <map>
#include "Planner.h"

class LPAStar : public Planner {

private:

    // g(s): current shortest-path estimate
    std::map<uint64_t, double> g;

    // rhs(s): one-step lookahead value
    std::map<uint64_t, double> rhs;

    // Parent state and transition used for path reconstruction
    std::map<uint64_t, uint64_t> parent;
    std::map<uint64_t, uint64_t> parentTransition;

    const PlanningProblem* currentProblem;

    double heuristic(uint64_t stateID, uint64_t goalID);

    double getG(uint64_t stateID);

    double getRHS(uint64_t stateID);

    double calculateRHS(uint64_t stateID);

    void updateVertex(uint64_t stateID);

public:

    LPAStar();

    PlanningResult plan(
        const PlanningProblem& problem
    ) override;
};

#endif