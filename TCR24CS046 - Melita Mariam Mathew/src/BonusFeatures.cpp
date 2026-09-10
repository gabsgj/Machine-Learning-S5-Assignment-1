#include "BonusFeatures.hpp"
#include <iostream>

PlanningResult MultiGoalPlanner::planMultiGoalTour(
    const PlanningProblem& baseProblem,
    const std::vector<uint64_t>& goalSequence,
    LPAStarPlanner& planner) {

    PlanningResult overallTour;
    overallTour.success = true;
    overallTour.totalCost = 0.0;
    overallTour.cumulativeReliability = 1.0;
    overallTour.minSafetyDistance = 1e9;
    overallTour.badStatesVisited = 0;
    overallTour.exploredStates = 0;
    overallTour.planningTimeMs = 0.0;

    uint64_t currentStart = baseProblem.initialState;

    for (size_t i = 0; i < goalSequence.size(); ++i) {
        uint64_t currentGoal = goalSequence[i];
        PlanningProblem legProblem = baseProblem;
        legProblem.initialState = currentStart;
        legProblem.goalState = currentGoal;

        PlanningResult legRes = planner.plan(legProblem);

        if (!legRes.success) {
            overallTour.success = false;
            return overallTour;
        }

        // Stitch paths
        if (overallTour.statePath.empty()) {
            overallTour.statePath = legRes.statePath;
        } else {
            // Append starting from 2nd node of leg to avoid duplicate vertex
            for (size_t j = 1; j < legRes.statePath.size(); ++j) {
                overallTour.statePath.push_back(legRes.statePath[j]);
            }
        }
        for (uint64_t tid : legRes.transitionPath) {
            overallTour.transitionPath.push_back(tid);
        }

        overallTour.totalCost += legRes.totalCost;
        overallTour.cumulativeReliability *= legRes.cumulativeReliability;
        overallTour.minSafetyDistance = std::min(overallTour.minSafetyDistance, legRes.minSafetyDistance);
        overallTour.badStatesVisited += legRes.badStatesVisited;
        overallTour.exploredStates += legRes.exploredStates;
        overallTour.planningTimeMs += legRes.planningTimeMs;

        currentStart = currentGoal;
    }

    double goalComp = overallTour.success ? 1.0 : 0.0;
    overallTour.safetyScore = overallTour.minSafetyDistance;
    overallTour.compositeScore = (planner.alpha * goalComp) - (planner.beta * overallTour.totalCost)
                               + (planner.gamma * overallTour.minSafetyDistance) + (planner.delta * overallTour.cumulativeReliability);

    return overallTour;
}

PlanningProblem KnowledgeGraphSemanticSearch::buildSemanticKGProblem(
    const std::vector<KGNode>& nodes,
    const std::vector<KGRelation>& relations,
    uint64_t startConcept,
    uint64_t targetConcept,
    const std::vector<uint64_t>& tabooConcepts) {

    std::vector<State> states;
    for (const auto& n : nodes) {
        states.emplace_back(n.id, n.semanticEmbedding);
    }

    std::vector<Transition> transitions;
    for (const auto& r : relations) {
        // Semantic cost derived from relation confidence (cost = 1.0 / confidence)
        double cost = (r.confidence > 1e-4) ? (1.0 / r.confidence) : 10.0;
        transitions.emplace_back(r.id, r.from, r.to, cost, r.safetyScore, r.confidence, true);
    }

    return PlanningProblem(startConcept, targetConcept, tabooConcepts, states, transitions);
}
