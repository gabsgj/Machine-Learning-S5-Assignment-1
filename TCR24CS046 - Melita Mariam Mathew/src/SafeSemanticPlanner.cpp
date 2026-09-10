#include "SafeSemanticPlanner.hpp"

SafeSemanticPlanner::SafeSemanticPlanner(AlgorithmType type, double a, double b, double g, double d, double sRadius)
    : alpha(a), beta(b), gamma(g), delta(d), safetyRadius(sRadius), algoType(type), initialized(false) {
    lpaPlanner = std::make_unique<LPAStarPlanner>(alpha, beta, gamma, delta, safetyRadius);
    dstarPlanner = std::make_unique<DstarlitePlanner>(alpha, beta, gamma, delta, safetyRadius);
}

std::string SafeSemanticPlanner::getAlgorithmName() const {
    switch (algoType) {
        case AlgorithmType::LPA_STAR: return "Lifelong Planning A* (LPA*)";
        case AlgorithmType::D_STAR_LITE: return "D* Lite";
        case AlgorithmType::SAFE_A_STAR: return "Safe Multi-Objective A*";
        default: return "Unknown";
    }
}

void SafeSemanticPlanner::init(const PlanningProblem& problem) {
    currentProblem = problem;
    lpaPlanner = std::make_unique<LPAStarPlanner>(alpha, beta, gamma, delta, safetyRadius);
    dstarPlanner = std::make_unique<DstarlitePlanner>(alpha, beta, gamma, delta, safetyRadius);

    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->initProblem(problem);
    } else {
        lpaPlanner->initProblem(problem);
    }
    initialized = true;
}

PlanningResult SafeSemanticPlanner::plan(const PlanningProblem& problem) {
    init(problem);
    return replan();
}

PlanningResult SafeSemanticPlanner::replan() {
    if (!initialized) {
        PlanningResult res;
        res.success = false;
        return res;
    }
    if (algoType == AlgorithmType::D_STAR_LITE) {
        return dstarPlanner->computeShortestPath();
    } else {
        return lpaPlanner->computeShortestPath();
    }
}

void SafeSemanticPlanner::setTransitionAvailability(uint64_t tid, bool avail, double cost) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->updateTransition(tid, avail, cost);
    } else {
        lpaPlanner->updateTransition(tid, avail, cost);
    }
}

void SafeSemanticPlanner::insertTransition(const Transition& t) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->addTransition(t);
    } else {
        lpaPlanner->addTransition(t);
    }
}

void SafeSemanticPlanner::removeTransition(uint64_t tid) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->removeTransition(tid);
    } else {
        lpaPlanner->removeTransition(tid);
    }
}

void SafeSemanticPlanner::updateBadStates(const std::vector<uint64_t>& bads) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->updateBadStates(bads);
    } else {
        lpaPlanner->updateBadStates(bads);
    }
}

void SafeSemanticPlanner::setGoal(uint64_t newGoal) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->updateGoal(newGoal);
    } else {
        lpaPlanner->updateGoal(newGoal);
    }
}

void SafeSemanticPlanner::setStart(uint64_t newStart) {
    if (algoType == AlgorithmType::D_STAR_LITE) {
        dstarPlanner->updateStart(newStart);
    } else {
        // LPA* can update start by re-initializing or key shifting
        currentProblem.initialState = newStart;
        lpaPlanner->initProblem(currentProblem);
    }
}
