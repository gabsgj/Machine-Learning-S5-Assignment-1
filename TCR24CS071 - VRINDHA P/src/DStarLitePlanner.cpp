#include "../include/DStarLitePlanner.hpp"
#include <algorithm>
#include <iostream>

DStarLitePlanner::DStarLitePlanner(PlannerWeights weights)
    : weights_(weights), startId_(0), goalId_(0), lastStartId_(0), km_(0.0), expandedCount_(0) {}

void DStarLitePlanner::computeBadDistances() {
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

double DStarLitePlanner::getDistanceToBad(uint64_t stateId) const {
    auto it = badDistanceCache_.find(stateId);
    return it != badDistanceCache_.end() ? it->second : std::numeric_limits<double>::infinity();
}

double DStarLitePlanner::computeTransitionWeight(const Transition& t) const {
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
    double safetyPenalty = 0.0;
    if (!badStatesSet_.empty() && !std::isinf(dBad)) {
        safetyPenalty = weights_.gamma / ((dBad + 0.05) * (dBad + 0.05));
    }

    double r = std::max(1e-5, std::min(1.0, t.reliability));
    double reliabilityPenalty = -weights_.delta * std::log(r);

    double s = std::max(1e-5, std::min(1.0, t.safety));
    double transSafetyPenalty = (1.0 - s) * 2.0;

    return costComponent + safetyPenalty + reliabilityPenalty + transSafetyPenalty;
}

double DStarLitePlanner::heuristic(uint64_t u, uint64_t v) const {
    auto itU = states_.find(u);
    auto itV = states_.find(v);
    if (itU == states_.end() || itV == states_.end()) {
        return 0.0;
    }
    double euclidean = itU->second.euclideanDistance(itV->second);
    return euclidean * weights_.heuristicScale * weights_.beta;
}

Key DStarLitePlanner::calculateKey(uint64_t stateId) const {
    double minVal = std::min(getG(stateId), getRhs(stateId));
    double h = heuristic(startId_, stateId) + km_;
    return Key{minVal + h, minVal};
}

void DStarLitePlanner::initialize() {
    g_.clear();
    rhs_.clear();
    openSetKeys_.clear();
    while (!openQueue_.empty()) {
        openQueue_.pop();
    }
    km_ = 0.0;
    lastStartId_ = startId_;
    expandedCount_ = 0;

    for (const auto& kv : states_) {
        g_[kv.first] = INF;
        rhs_[kv.first] = INF;
    }

    // In D* Lite, search is backwards from goal: rhs(goal) = 0
    rhs_[goalId_] = 0.0;
    Key goalKey = calculateKey(goalId_);
    openSetKeys_[goalId_] = goalKey;
    openQueue_.push(QueueElement{goalKey, goalId_});
}

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (u != goalId_) {
        double minRhs = INF;
        // In reverse search, rhs(u) = min_{s \in Succ(u)} (w(u, s) + g(s))
        auto succIt = succTransitions_.find(u);
        if (succIt != succTransitions_.end()) {
            for (uint64_t tId : succIt->second) {
                const Transition& t = transitions_[tId];
                double edgeWeight = computeTransitionWeight(t);
                if (!std::isinf(edgeWeight)) {
                    double gSucc = getG(t.to);
                    if (!std::isinf(gSucc)) {
                        double candidate = edgeWeight + gSucc;
                        if (candidate < minRhs) {
                            minRhs = candidate;
                        }
                    }
                }
            }
        }
        rhs_[u] = minRhs;
    }

    openSetKeys_.erase(u);

    double gVal = getG(u);
    double rhsVal = getRhs(u);
    if (std::abs(gVal - rhsVal) > 1e-9 && !(std::isinf(gVal) && std::isinf(rhsVal))) {
        Key key = calculateKey(u);
        openSetKeys_[u] = key;
        openQueue_.push(QueueElement{key, u});
    }
}

void DStarLitePlanner::computeShortestPath() {
    while (!openQueue_.empty()) {
        QueueElement top = openQueue_.top();

        // Lazy deletion check
        auto it = openSetKeys_.find(top.stateId);
        if (it == openSetKeys_.end() || std::abs(it->second.k1 - top.key.k1) > 1e-9 ||
            std::abs(it->second.k2 - top.key.k2) > 1e-9) {
            openQueue_.pop();
            continue;
        }

        Key startKey = calculateKey(startId_);
        double startG = getG(startId_);
        double startRhs = getRhs(startId_);

        if (!(top.key < startKey) && std::abs(startG - startRhs) <= 1e-9) {
            break;
        }

        Key kOld = top.key;
        Key kNew = calculateKey(top.stateId);

        if (kOld < kNew) {
            openQueue_.pop();
            openSetKeys_[top.stateId] = kNew;
            openQueue_.push(QueueElement{kNew, top.stateId});
            continue;
        }

        openQueue_.pop();
        openSetKeys_.erase(top.stateId);
        uint64_t u = top.stateId;
        expandedCount_++;

        double uG = getG(u);
        double uRhs = getRhs(u);

        if (uG > uRhs) {
            g_[u] = uRhs;
            // Update predecessors of u in reverse search
            auto predIt = predTransitions_.find(u);
            if (predIt != predTransitions_.end()) {
                for (uint64_t tId : predIt->second) {
                    updateVertex(transitions_[tId].from);
                }
            }
        } else {
            g_[u] = INF;
            updateVertex(u);
            auto predIt = predTransitions_.find(u);
            if (predIt != predTransitions_.end()) {
                for (uint64_t tId : predIt->second) {
                    updateVertex(transitions_[tId].from);
                }
            }
        }
    }
}

PlanningResult DStarLitePlanner::extractPath() {
    PlanningResult result;
    result.expandedStates = expandedCount_;

    double startG = getG(startId_);
    if (std::isinf(startG) && std::isinf(getRhs(startId_))) {
        result.success = false;
        return result;
    }

    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transPath;
    uint64_t current = startId_;
    statePath.push_back(current);

    double totalTransitionCost = 0.0;
    double cumulativeRel = 1.0;
    double minBadDist = getDistanceToBad(startId_);
    std::unordered_set<uint64_t> visited;
    visited.insert(current);

    while (current != goalId_) {
        auto succIt = succTransitions_.find(current);
        if (succIt == succTransitions_.end() || succIt->second.empty()) {
            result.success = false;
            return result;
        }

        uint64_t bestSucc = 0;
        uint64_t bestTransId = 0;
        double bestVal = INF;

        for (uint64_t tId : succIt->second) {
            const Transition& t = transitions_[tId];
            double edgeWeight = computeTransitionWeight(t);
            if (!std::isinf(edgeWeight)) {
                double gSucc = getG(t.to);
                if (!std::isinf(gSucc)) {
                    double val = edgeWeight + gSucc;
                    if (val < bestVal) {
                        bestVal = val;
                        bestSucc = t.to;
                        bestTransId = t.id;
                    }
                }
            }
        }

        if (bestVal == INF || visited.find(bestSucc) != visited.end()) {
            result.success = false;
            return result;
        }

        const Transition& chosenTrans = transitions_[bestTransId];
        totalTransitionCost += chosenTrans.cost;
        cumulativeRel *= chosenTrans.reliability;

        double dBad = getDistanceToBad(bestSucc);
        if (dBad < minBadDist) {
            minBadDist = dBad;
        }

        transPath.push_back(bestTransId);
        current = bestSucc;
        statePath.push_back(current);
        visited.insert(current);
    }

    result.statePath = statePath;
    result.transitionPath = transPath;
    result.success = true;
    result.totalCost = totalTransitionCost;
    result.cumulativeReliability = cumulativeRel;
    result.minBadStateDistance = minBadDist;

    double goalCompletion = result.success ? 1.0 : 0.0;
    result.safetyScore = (weights_.alpha * goalCompletion) -
                         (weights_.beta * totalTransitionCost) +
                         (weights_.gamma * (std::isinf(minBadDist) ? 10.0 : minBadDist)) +
                         (weights_.delta * cumulativeRel);

    return result;
}

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
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

void DStarLitePlanner::updateStart(uint64_t newStartId) {
    if (startId_ == newStartId) return;
    km_ += heuristic(lastStartId_, newStartId);
    lastStartId_ = newStartId;
    startId_ = newStartId;
}

void DStarLitePlanner::updateTransition(uint64_t transitionId, double newCost, double newSafety,
                                       double newReliability, bool newAvailable) {
    auto it = transitions_.find(transitionId);
    if (it != transitions_.end()) {
        it->second.cost = newCost;
        it->second.safety = newSafety;
        it->second.reliability = newReliability;
        it->second.available = newAvailable;
        updateVertex(it->second.from);
    }
}

void DStarLitePlanner::addTransition(const Transition& t) {
    transitions_[t.id] = t;
    succTransitions_[t.from].push_back(t.id);
    predTransitions_[t.to].push_back(t.id);
    updateVertex(t.from);
}

void DStarLitePlanner::removeTransition(uint64_t transitionId) {
    auto it = transitions_.find(transitionId);
    if (it != transitions_.end()) {
        uint64_t from = it->second.from;
        uint64_t to = it->second.to;

        auto& sList = succTransitions_[from];
        sList.erase(std::remove(sList.begin(), sList.end(), transitionId), sList.end());

        auto& pList = predTransitions_[to];
        pList.erase(std::remove(pList.begin(), pList.end(), transitionId), pList.end());

        transitions_.erase(it);
        updateVertex(from);
    }
}

PlanningResult DStarLitePlanner::replan() {
    auto startTime = std::chrono::high_resolution_clock::now();
    size_t prevExp = expandedCount_;

    computeShortestPath();

    auto endTime = std::chrono::high_resolution_clock::now();
    PlanningResult res = extractPath();
    res.expandedStates = expandedCount_ - prevExp;
    res.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
    return res;
}
