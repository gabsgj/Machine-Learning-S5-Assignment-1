#pragma once
#include "Planner.h"
#include <unordered_map>

class LPAStarPlanner : public Planner {
public:
    // weights: costWeight (minimize), safetyWeight (maximize), heurWeight
    LPAStarPlanner(double costWeight=1.0, double safetyWeight=1.0, double heurWeight=1.0,
                 double reliabilityWeight=1.0)
        : costW(costWeight), safetyW(safetyWeight), heurW(heurWeight), reliabilityW(reliabilityWeight) {}

    PlanningResult plan(const PlanningProblem& problem) override;

private:
    double costW;
    double safetyW;
    double heurW;
    double reliabilityW;
    mutable bool adjacencyCacheValid = false;
    mutable std::size_t adjacencySignature = 0;
    mutable std::unordered_map<uint64_t, std::vector<Transition>> cachedAdjacency;

    double euclidean(const std::vector<double>& a, const std::vector<double>& b) const;
    double safetyOfState(uint64_t sid, const PlanningProblem& problem) const;
    std::size_t transitionSignature(const PlanningProblem& problem) const;
};
