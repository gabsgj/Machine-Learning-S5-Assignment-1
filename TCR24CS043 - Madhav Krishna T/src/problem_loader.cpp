#include "safe_semantic_planner/problem_loader.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>

namespace safe_semantic_planner {

double ProblemLoader::computeEuclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) {
        return 0.0;
    }
    size_t dim = std::min(a.size(), b.size());
    double sumSq = 0.0;
    for (size_t i = 0; i < dim; ++i) {
        double diff = a[i] - b[i];
        sumSq += diff * diff;
    }
    return std::sqrt(sumSq);
}

ProblemLoader::ProblemLoader(PlanningProblem problem, WeightParams params)
    : problem_(std::move(problem)), params_(params) {
    processBadStatesAndExclusion();
    processTransitionsAndValidate();
}

void ProblemLoader::processBadStatesAndExclusion() {
    stateIndexMap_.clear();
    activeStates_.clear();
    excludedStates_.clear();

    for (size_t i = 0; i < problem_.states.size(); ++i) {
        stateIndexMap_[problem_.states[i].id] = i;
        problem_.states[i].isBadState = false;
        problem_.states[i].isExcluded = false;
        problem_.states[i].distanceToNearestBadState = std::numeric_limits<double>::infinity();
    }

    std::unordered_set<std::string> badSet(problem_.badStates.begin(), problem_.badStates.end());
    std::vector<const State*> badStatePtrs;
    for (const auto& badId : problem_.badStates) {
        auto it = stateIndexMap_.find(badId);
        if (it != stateIndexMap_.end()) {
            problem_.states[it->second].isBadState = true;
            badStatePtrs.push_back(&problem_.states[it->second]);
        }
    }

    for (auto& s : problem_.states) {
        if (badStatePtrs.empty()) {
            s.distanceToNearestBadState = std::numeric_limits<double>::infinity();
        } else if (s.isBadState) {
            s.distanceToNearestBadState = 0.0;
        } else {
            double minDist = std::numeric_limits<double>::infinity();
            for (const auto* badState : badStatePtrs) {
                double dist = computeEuclideanDistance(s.embedding, badState->embedding);
                if (dist < minDist) {
                    minDist = dist;
                }
            }
            s.distanceToNearestBadState = minDist;
        }

        // Hard exclusion rule: within radius r of any bad state
        if (s.distanceToNearestBadState <= params_.r) {
            s.isExcluded = true;
        }
    }

    for (const auto& s : problem_.states) {
        if (s.isExcluded) {
            excludedStates_.push_back(&s);
        } else {
            activeStates_.push_back(&s);
        }
    }
}

void ProblemLoader::processTransitionsAndValidate() {
    activeTransitions_.clear();

    minCost_ = std::numeric_limits<double>::infinity();
    maxReliability_ = 0.0;
    cMin_ = std::numeric_limits<double>::infinity();

    for (const auto& t : problem_.transitions) {
        if (!t.available) {
            continue;
        }
        auto itFrom = stateIndexMap_.find(t.from);
        auto itTo = stateIndexMap_.find(t.to);
        if (itFrom == stateIndexMap_.end() || itTo == stateIndexMap_.end()) {
            continue;
        }

        const State& fromState = problem_.states[itFrom->second];
        const State& toState = problem_.states[itTo->second];

        // Active transition must connect non-excluded states
        if (!fromState.isExcluded && !toState.isExcluded) {
            activeTransitions_.push_back(&t);
            minCost_ = std::min(minCost_, t.cost);
            maxReliability_ = std::max(maxReliability_, t.reliability);

            double dist = computeEuclideanDistance(fromState.embedding, toState.embedding);
            if (dist > 1e-9) {
                double w = params_.beta * t.cost - params_.delta * t.reliability;
                double ratio = w / dist;
                cMin_ = std::min(cMin_, ratio);
            }
        }
    }

    if (activeTransitions_.empty()) {
        minCost_ = 0.0;
        maxReliability_ = 0.0;
        wMin_ = 0.0;
        cMin_ = 0.0;
        return;
    }

    if (cMin_ == std::numeric_limits<double>::infinity()) {
        cMin_ = 1.0;
    }

    // Compute w_min = beta * min(cost) - delta * max(reliability)
    wMin_ = params_.beta * minCost_ - params_.delta * maxReliability_;

    if (wMin_ < 0.0) {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(4);
        oss << "Load-time validation failed: Non-negative edge weight assumption violated. "
            << "w_min = beta * min(cost) - delta * max(reliability) = "
            << params_.beta << " * " << minCost_ << " - " << params_.delta << " * " << maxReliability_
            << " = " << wMin_ << " < 0.0. "
            << "Offending bounds: min(cost)=" << minCost_ << ", max(reliability)=" << maxReliability_
            << " with weights (beta=" << params_.beta << ", delta=" << params_.delta << ").";
        throw ValidationException(oss.str());
    }
}

const State* ProblemLoader::getState(const std::string& id) const {
    auto it = stateIndexMap_.find(id);
    if (it != stateIndexMap_.end()) {
        return &problem_.states[it->second];
    }
    return nullptr;
}

bool ProblemLoader::isInitialStateExcluded() const {
    const State* s = getState(problem_.initialState);
    return s == nullptr || s->isExcluded;
}

bool ProblemLoader::isGoalStateExcluded() const {
    const State* s = getState(problem_.goalState);
    return s == nullptr || s->isExcluded;
}

} // namespace safe_semantic_planner
