#include "../include/LPAStarPlanner.hpp"
#include <algorithm>
#include <iostream>

LPAStarPlanner::LPAStarPlanner(PlannerWeights weights)
    : weights_(weights), startId_(0), goalId_(0), expandedCount_(0) {}

void LPAStarPlanner::computeBadDistances() {
    badDistanceCache_.clear();
    for (const auto& kv : states_) {
        uint64_t sId = kv.first;
        const State& st = kv.second;

        if (badStatesSet_.empty()) {
            badDistanceCache_[sId] = std::numeric_limits<double>::infinity();
            continue;
        }

        if (badStatesSet_.find(sId) != badStatesSet_.end()) {
            badDistanceCache_[sId] = 0.0;
            continue;
        }

        double minDist = std::numeric_limits<double>::infinity();
        for (uint64_t bId : badStatesSet_) {
            auto it = states_.find(bId);
            if (it != states_.end()) {
                double dist = st.euclideanDistance(it->second);
                if (dist < minDist) {
                    minDist = dist;
                }
            }
        }
        badDistanceCache_[sId] = minDist;
    }
}

double LPAStarPlanner::computeTransitionWeight(const Transition& t) const {
    if (!t.available) {
        return INF;
    }
    if (badStatesSet_.find(t.to) != badStatesSet_.end() ||
        badStatesSet_.find(t.from) != badStatesSet_.end()) {
        return INF;
    }

    double dBad = getDistanceToBad(t.to);
    if (dBad < weights_.safeDistanceThreshold) {
        return INF;
    }

    double costComponent = weights_.beta * std::max(0.0, t.cost);

    // Non-linear repulsive safety potential from bad states
    double safetyPenalty = 0.0;
    if (!badStatesSet_.empty() && !std::isinf(dBad)) {
        // High penalty when close to bad states, asymptotic drop-off with distance
        safetyPenalty = weights_.gamma / ((dBad + 0.05) * (dBad + 0.05));
    }

    // Reliability cost: -ln(reliability)
    double r = std::max(1e-5, std::min(1.0, t.reliability));
    double reliabilityPenalty = -weights_.delta * std::log(r);

    // Transition intrinsic safety score (1 - safety)
    double s = std::max(1e-5, std::min(1.0, t.safety));
    double transSafetyPenalty = (1.0 - s) * 2.0;

    return costComponent + safetyPenalty + reliabilityPenalty + transSafetyPenalty;
}

double LPAStarPlanner::heuristic(uint64_t u, uint64_t v) const {
    auto itU = states_.find(u);
    auto itV = states_.find(v);
    if (itU == states_.end() || itV == states_.end()) {
        return 0.0;
    }
    double euclidean = itU->second.euclideanDistance(itV->second);
    return euclidean * weights_.heuristicScale * weights_.beta;
}

Key LPAStarPlanner::calculateKey(uint64_t stateId) const {
    double minVal = std::min(getG(stateId), getRhs(stateId));
    double h = heuristic(stateId, goalId_);
    return Key{minVal + h, minVal};
}

double LPAStarPlanner::getDistanceToBad(uint64_t stateId) const {
    auto it = badDistanceCache_.find(stateId);
    return it != badDistanceCache_.end() ? it->second : std::numeric_limits<double>::infinity();
}

void LPAStarPlanner::initialize() {
    g_.clear();
    rhs_.clear();
    openSetKeys_.clear();
    while (!openQueue_.empty()) {
        openQueue_.pop();
    }
    expandedCount_ = 0;

    for (const auto& kv : states_) {
        g_[kv.first] = INF;
        rhs_[kv.first] = INF;
    }

    rhs_[startId_] = 0.0;
    Key startKey = calculateKey(startId_);
    openSetKeys_[startId_] = startKey;
    openQueue_.push(QueueElement{startKey, startId_});
}

void LPAStarPlanner::updateVertex(uint64_t stateId) {
    if (stateId != startId_) {
        double minRhs = INF;
        auto predIt = predTransitions_.find(stateId);
        if (predIt != predTransitions_.end()) {
            for (uint64_t tId : predIt->second) {
                const Transition& t = transitions_[tId];
                double edgeWeight = computeTransitionWeight(t);
                if (!std::isinf(edgeWeight)) {
                    double gFrom = getG(t.from);
                    if (!std::isinf(gFrom)) {
                        double candidate = gFrom + edgeWeight;
                        if (candidate < minRhs) {
                            minRhs = candidate;
                        }
                    }
                }
            }
        }
        rhs_[stateId] = minRhs;
    }

    // Remove from open set key tracking
    openSetKeys_.erase(stateId);

    // If locally inconsistent, re-insert with new key
    double gVal = getG(stateId);
    double rhsVal = getRhs(stateId);
    if (std::abs(gVal - rhsVal) > 1e-9 && !(std::isinf(gVal) && std::isinf(rhsVal))) {
        Key key = calculateKey(stateId);
        openSetKeys_[stateId] = key;
        openQueue_.push(QueueElement{key, stateId});
    }
}

void LPAStarPlanner::computeShortestPath() {
    while (!openQueue_.empty()) {
        QueueElement top = openQueue_.top();

        // Lazy deletion check: verify if the queued element is still valid and up-to-date
        auto it = openSetKeys_.find(top.stateId);
        if (it == openSetKeys_.end() || std::abs(it->second.k1 - top.key.k1) > 1e-9 ||
            std::abs(it->second.k2 - top.key.k2) > 1e-9) {
            openQueue_.pop();
            continue;
        }

        Key goalKey = calculateKey(goalId_);
        double goalG = getG(goalId_);
        double goalRhs = getRhs(goalId_);

        // Termination condition: top key >= goal key AND goal is consistent
        if (!(top.key < goalKey) && std::abs(goalG - goalRhs) <= 1e-9) {
            break;
        }

        openQueue_.pop();
        openSetKeys_.erase(top.stateId);
        uint64_t u = top.stateId;
        expandedCount_++;

        double uG = getG(u);
        double uRhs = getRhs(u);

        if (uG > uRhs) {
            // Locally overconsistent: set g = rhs
            g_[u] = uRhs;
            auto succIt = succTransitions_.find(u);
            if (succIt != succTransitions_.end()) {
                for (uint64_t tId : succIt->second) {
                    updateVertex(transitions_[tId].to);
                }
            }
        } else {
            // Locally underconsistent: set g = INF and update u and successors
            g_[u] = INF;
            updateVertex(u);
            auto succIt = succTransitions_.find(u);
            if (succIt != succTransitions_.end()) {
                for (uint64_t tId : succIt->second) {
                    updateVertex(transitions_[tId].to);
                }
            }
        }
    }
}

PlanningResult LPAStarPlanner::extractPath() {
    PlanningResult result;
    result.expandedStates = expandedCount_;

    double goalG = getG(goalId_);
    if (std::isinf(goalG) || std::isinf(getRhs(goalId_))) {
        result.success = false;
        return result;
    }

    // Backtrack from goal to start
    std::vector<uint64_t> revStatePath;
    std::vector<uint64_t> revTransPath;
    uint64_t current = goalId_;
    revStatePath.push_back(current);

    double totalTransitionCost = 0.0;
    double cumulativeRel = 1.0;
    double minBadDist = getDistanceToBad(goalId_);
    std::unordered_set<uint64_t> visited;
    visited.insert(current);

    while (current != startId_) {
        auto predIt = predTransitions_.find(current);
        if (predIt == predTransitions_.end() || predIt->second.empty()) {
            result.success = false;
            return result;
        }

        uint64_t bestPred = 0;
        uint64_t bestTransId = 0;
        double bestVal = INF;

        for (uint64_t tId : predIt->second) {
            const Transition& t = transitions_[tId];
            double edgeWeight = computeTransitionWeight(t);
            if (!std::isinf(edgeWeight)) {
                double gPred = getG(t.from);
                if (!std::isinf(gPred)) {
                    double val = gPred + edgeWeight;
                    if (val < bestVal) {
                        bestVal = val;
                        bestPred = t.from;
                        bestTransId = t.id;
                    }
                }
            }
        }

        if (bestVal == INF || visited.find(bestPred) != visited.end()) {
            // No valid predecessor or cycle detected
            result.success = false;
            return result;
        }

        const Transition& chosenTrans = transitions_[bestTransId];
        totalTransitionCost += chosenTrans.cost;
        cumulativeRel *= chosenTrans.reliability;

        double dBad = getDistanceToBad(bestPred);
        if (dBad < minBadDist) {
            minBadDist = dBad;
        }

        revTransPath.push_back(bestTransId);
        current = bestPred;
        revStatePath.push_back(current);
        visited.insert(current);
    }

    result.statePath.assign(revStatePath.rbegin(), revStatePath.rend());
    result.transitionPath.assign(revTransPath.rbegin(), revTransPath.rend());
    result.success = true;
    result.totalCost = totalTransitionCost;
    result.cumulativeReliability = cumulativeRel;
    result.minBadStateDistance = minBadDist;

    // Composite safety score: α*G - β*C + γ*D + δ*R
    double goalCompletion = result.success ? 1.0 : 0.0;
    result.safetyScore = (weights_.alpha * goalCompletion) -
                         (weights_.beta * totalTransitionCost) +
                         (weights_.gamma * (std::isinf(minBadDist) ? 10.0 : minBadDist)) +
                         (weights_.delta * cumulativeRel);

    return result;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    auto startTime = std::chrono::high_resolution_clock::now();

    currentProblem_ = problem;
    startId_ = problem.initialState;
    goalId_ = problem.goalState;

    states_.clear();
    for (const auto& s : problem.states) {
        states_[s.id] = s;
    }

    badStatesSet_.clear();
    for (uint64_t b : problem.badStates) {
        badStatesSet_.insert(b);
    }

    succTransitions_.clear();
    predTransitions_.clear();
    transitions_.clear();
    for (const auto& t : problem.transitions) {
        transitions_[t.id] = t;
        succTransitions_[t.from].push_back(t.id);
        predTransitions_[t.to].push_back(t.id);
    }

    computeBadDistances();
    initialize();
    computeShortestPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    PlanningResult res = extractPath();
    res.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
    return res;
}

void LPAStarPlanner::updateTransition(uint64_t transitionId, double newCost, double newSafety,
                                     double newReliability, bool newAvailable) {
    auto it = transitions_.find(transitionId);
    if (it != transitions_.end()) {
        it->second.cost = newCost;
        it->second.safety = newSafety;
        it->second.reliability = newReliability;
        it->second.available = newAvailable;
        updateVertex(it->second.to);
    }
}

void LPAStarPlanner::addTransition(const Transition& t) {
    transitions_[t.id] = t;
    succTransitions_[t.from].push_back(t.id);
    predTransitions_[t.to].push_back(t.id);
    updateVertex(t.to);
}

void LPAStarPlanner::removeTransition(uint64_t transitionId) {
    auto it = transitions_.find(transitionId);
    if (it != transitions_.end()) {
        uint64_t from = it->second.from;
        uint64_t to = it->second.to;

        auto& sList = succTransitions_[from];
        sList.erase(std::remove(sList.begin(), sList.end(), transitionId), sList.end());

        auto& pList = predTransitions_[to];
        pList.erase(std::remove(pList.begin(), pList.end(), transitionId), pList.end());

        transitions_.erase(it);
        updateVertex(to);
    }
}

void LPAStarPlanner::updateBadStates(const std::vector<uint64_t>& newBadStates) {
    badStatesSet_.clear();
    for (uint64_t b : newBadStates) {
        badStatesSet_.insert(b);
    }
    computeBadDistances();

    // Re-evaluate affected vertices
    for (const auto& kv : states_) {
        updateVertex(kv.first);
    }
}

void LPAStarPlanner::updateGoal(uint64_t newGoalId) {
    if (goalId_ == newGoalId) return;
    goalId_ = newGoalId;

    // When goal changes, keys in open set depend on h(s, newGoal). Recompute open keys.
    std::unordered_map<uint64_t, Key> oldKeys = openSetKeys_;
    openSetKeys_.clear();
    while (!openQueue_.empty()) {
        openQueue_.pop();
    }

    for (const auto& kv : oldKeys) {
        uint64_t u = kv.first;
        Key newKey = calculateKey(u);
        openSetKeys_[u] = newKey;
        openQueue_.push(QueueElement{newKey, u});
    }

    updateVertex(goalId_);
}

void LPAStarPlanner::updateStart(uint64_t newStartId) {
    if (startId_ == newStartId) return;
    uint64_t oldStart = startId_;
    startId_ = newStartId;

    rhs_[oldStart] = INF;
    updateVertex(oldStart);

    rhs_[startId_] = 0.0;
    updateVertex(startId_);
}

PlanningResult LPAStarPlanner::replan() {
    auto startTime = std::chrono::high_resolution_clock::now();
    size_t prevExp = expandedCount_;

    computeShortestPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    PlanningResult res = extractPath();
    res.expandedStates = expandedCount_ - prevExp;
    res.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
    return res;
}
