#pragma once

#include "core_types.hpp"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <string>

namespace safe_semantic_planner {

class ProblemLoader {
public:
    /**
     * @brief Load and process a planning problem with specified weight and safety parameters.
     * @param problem The input planning problem.
     * @param params Weight parameters (alpha, beta, gamma, delta, r).
     * @throws ValidationException if w_min < 0 or if referenced states do not exist.
     */
    ProblemLoader(PlanningProblem problem, WeightParams params);

    // Getters for processed data
    const PlanningProblem& getProblem() const { return problem_; }
    const WeightParams& getParams() const { return params_; }
    
    double getWMin() const { return wMin_; }
    double getCMin() const { return cMin_; }
    double getMinCost() const { return minCost_; }
    double getMaxReliability() const { return maxReliability_; }

    const State* getState(const std::string& id) const;
    bool isInitialStateExcluded() const;
    bool isGoalStateExcluded() const;

    const std::vector<const State*>& getActiveStates() const { return activeStates_; }
    const std::vector<const State*>& getExcludedStates() const { return excludedStates_; }
    const std::vector<const Transition*>& getActiveTransitions() const { return activeTransitions_; }

    // Helper to compute Euclidean distance between two coordinate vectors
    static double computeEuclideanDistance(const std::vector<double>& a, const std::vector<double>& b);

private:
    void processBadStatesAndExclusion();
    void processTransitionsAndValidate();

    PlanningProblem problem_;
    WeightParams params_;

    std::unordered_map<std::string, size_t> stateIndexMap_;
    std::vector<const State*> activeStates_;
    std::vector<const State*> excludedStates_;
    std::vector<const Transition*> activeTransitions_;

    double wMin_ = 0.0;
    double cMin_ = 0.0;
    double minCost_ = 0.0;
    double maxReliability_ = 0.0;
};

} // namespace safe_semantic_planner
