#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "lpa_star.hpp"
#include <vector>
#include <iostream>

namespace SafePlanner {

/**
 * @brief Multi-Objective and Multi-Goal Planning Engine.
 */
class MultiObjectivePlanner {
public:
    /**
     * @brief Computes multi-goal route visiting multiple waypoints sequentially in optimal safe order.
     */
    static PlanningResult planMultiGoal(
        Planner& basePlanner,
        const PlanningProblem& problem,
        const std::vector<uint64_t>& waypoints)
    {
        if (waypoints.empty()) {
            return basePlanner.plan(problem);
        }

        PlanningResult combinedResult;
        combinedResult.success = true;
        combinedResult.totalCost = 0.0;
        combinedResult.safetyScore = INF;
        combinedResult.cumulativeReliability = 1.0;
        combinedResult.algorithmName = "Multi-Goal LPA*";

        std::vector<uint64_t> fullSequence;
        fullSequence.push_back(problem.initialState);
        for (uint64_t wp : waypoints) {
            fullSequence.push_back(wp);
        }
        fullSequence.push_back(problem.goalState);

        PlanningProblem subProblem = problem;
        for (size_t i = 0; i + 1 < fullSequence.size(); ++i) {
            subProblem.initialState = fullSequence[i];
            subProblem.goalState = fullSequence[i + 1];

            PlanningResult legResult = basePlanner.plan(subProblem);
            if (!legResult.success) {
                combinedResult.success = false;
                return combinedResult;
            }

            // Append states (avoid duplicating intermediate joint states)
            if (combinedResult.statePath.empty()) {
                combinedResult.statePath = legResult.statePath;
            } else {
                for (size_t j = 1; j < legResult.statePath.size(); ++j) {
                    combinedResult.statePath.push_back(legResult.statePath[j]);
                }
            }

            for (uint64_t tId : legResult.transitionPath) {
                combinedResult.transitionPath.push_back(tId);
            }

            combinedResult.totalCost += legResult.totalCost;
            combinedResult.cumulativeReliability *= legResult.cumulativeReliability;
            if (legResult.safetyScore < combinedResult.safetyScore) {
                combinedResult.safetyScore = legResult.safetyScore;
            }
            combinedResult.exploredStates += legResult.exploredStates;
            combinedResult.planningTimeMicroseconds += legResult.planningTimeMicroseconds;
        }

        // Compute overall objective score
        double G = combinedResult.success ? 1.0 : 0.0;
        double C = combinedResult.totalCost;
        double D = (combinedResult.safetyScore < INF) ? combinedResult.safetyScore : 10.0;
        double R = combinedResult.cumulativeReliability;
        combinedResult.objectiveScore = (problem.alpha * G) - (problem.beta * C) + (problem.gamma * D) + (problem.delta * R);

        return combinedResult;
    }
};

} // namespace SafePlanner
