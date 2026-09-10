#ifndef PLANNER_HPP
#define PLANNER_HPP

#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"

/**
 * @brief Abstract Base Class for Planning Algorithms.
 */
class Planner {
public:
    virtual ~Planner() = default;

    /**
     * @brief Computes a safe path for the given planning problem.
     * @param problem The PlanningProblem definition.
     * @return PlanningResult containing the status, path, costs, and metrics.
     */
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

#endif // PLANNER_HPP
