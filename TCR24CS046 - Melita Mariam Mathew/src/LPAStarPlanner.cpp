#include "LPAStarPlanner.hpp"
#include <iostream>
#include <limits>

constexpr double INF_COST = 1e12;

LPAStarPlanner::LPAStarPlanner(double a, double b, double gVal, double d, double sRadius)
    : alpha(a), beta(b), gamma(gVal), delta(d), safetyRadius(sRadius),
      exploredStatesCount(0), isInitialized(false) {}

double LPAStarPlanner::heuristic(uint64_t fromId, uint64_t toId) const {
    auto itFrom = stateMap.find(fromId);
    auto itTo = stateMap.find(toId);
    if (itFrom == stateMap.end() || itTo == stateMap.end()) return 0.0;
    return itFrom->second.euclideanDistance(itTo->second);
}

double LPAStarPlanner::effectiveTransitionCost(const Transition& t) const {
    if (!t.available) return INF_COST;
    if (badStateSet.find(t.to) != badStateSet.end() || badStateSet.find(t.from) != badStateSet.end()) {
        return INF_COST;
    }

    double baseC = std::max(0.0001, t.cost) * beta;
    double relPenalty = delta * (1.0 - std::clamp(t.reliability, 0.0, 1.0));

    // Safety clearance penalty
    double safetyPenalty = 0.0;
    auto itToState = stateMap.find(t.to);
    if (itToState != stateMap.end() && !badStateSet.empty()) {
        double minDist = INF_COST;
        for (uint64_t bId : badStateSet) {
            auto itB = stateMap.find(bId);
            if (itB != stateMap.end()) {
                double dist = itToState->second.euclideanDistance(itB->second);
                if (dist < minDist) minDist = dist;
            }
        }

        if (minDist < safetyRadius) {
            double margin = safetyRadius - minDist;
            safetyPenalty = gamma * (margin * margin + (1.0 / (minDist + 0.1)));
        }
    }

    return baseC + relPenalty + safetyPenalty;
}

LPAKey LPAStarPlanner::calculateKey(uint64_t stateId) const {
    double gVal = getGValue(stateId);
    double rhsVal = getRhsValue(stateId);
    double m = std::min(gVal, rhsVal);
    double hVal = heuristic(stateId, currentProblem.goalState);
    return LPAKey{ m + hVal, m };
}

double LPAStarPlanner::getGValue(uint64_t stateId) const {
    auto it = g.find(stateId);
    return (it != g.end()) ? it->second : INF_COST;
}

double LPAStarPlanner::getRhsValue(uint64_t stateId) const {
    auto it = rhs.find(stateId);
    return (it != rhs.end()) ? it->second : INF_COST;
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != currentProblem.initialState) {
        double minRhs = INF_COST;
        auto inIt = inEdges.find(u);
        if (inIt != inEdges.end()) {
            for (uint64_t tid : inIt->second) {
                const auto& trans = transitionMap[tid];
                double gPred = getGValue(trans.from);
                double cEff = effectiveTransitionCost(trans);
                if (gPred < INF_COST / 2.0 && cEff < INF_COST / 2.0) {
                    double val = gPred + cEff;
                    if (val < minRhs) minRhs = val;
                }
            }
        }
        rhs[u] = minRhs;
    }

    openSetKeys.erase(u);

    double gVal = getGValue(u);
    double rhsVal = getRhsValue(u);
    if (std::abs(gVal - rhsVal) > 1e-9) {
        LPAKey k = calculateKey(u);
        openSetKeys[u] = k;
        openQueue.push(QueueNode{u, k});
    }
}

void LPAStarPlanner::initProblem(const PlanningProblem& problem) {
    currentProblem = problem;
    stateMap.clear();
    transitionMap.clear();
    badStateSet.clear();
    outEdges.clear();
    inEdges.clear();
    g.clear();
    rhs.clear();
    while (!openQueue.empty()) openQueue.pop();
    openSetKeys.clear();
    exploredStatesCount = 0;

    for (const auto& s : problem.states) {
        stateMap[s.id] = s;
        g[s.id] = INF_COST;
        rhs[s.id] = INF_COST;
    }
    for (uint64_t b : problem.badStates) {
        badStateSet.insert(b);
    }
    for (const auto& t : problem.transitions) {
        transitionMap[t.id] = t;
        outEdges[t.from].push_back(t.id);
        inEdges[t.to].push_back(t.id);
    }

    // Set rhs of start state to 0
    rhs[problem.initialState] = 0.0;
    LPAKey kStart = calculateKey(problem.initialState);
    openSetKeys[problem.initialState] = kStart;
    openQueue.push(QueueNode{problem.initialState, kStart});

    isInitialized = true;
}

PlanningResult LPAStarPlanner::computeShortestPath() {
    auto startTime = std::chrono::high_resolution_clock::now();

    while (!openQueue.empty()) {
        QueueNode top = openQueue.top();
        uint64_t u = top.id;
        LPAKey keyTop = top.key;

        // Lazy deletion check: if key is outdated, discard
        auto keyIt = openSetKeys.find(u);
        if (keyIt == openSetKeys.end() || !(keyIt->second <= keyTop && keyTop <= keyIt->second)) {
            openQueue.pop();
            continue;
        }

        LPAKey keyGoal = calculateKey(currentProblem.goalState);
        double gGoal = getGValue(currentProblem.goalState);
        double rhsGoal = getRhsValue(currentProblem.goalState);

        // Termination condition: top key >= key(goal) and goal is consistent
        if (!(keyTop < keyGoal) && std::abs(gGoal - rhsGoal) <= 1e-9) {
            break;
        }

        openQueue.pop();
        openSetKeys.erase(u);
        exploredStatesCount++;

        double gVal = getGValue(u);
        double rhsVal = getRhsValue(u);

        if (gVal > rhsVal) {
            // Overconsistent
            g[u] = rhsVal;
            auto outIt = outEdges.find(u);
            if (outIt != outEdges.end()) {
                for (uint64_t tid : outIt->second) {
                    updateVertex(transitionMap[tid].to);
                }
            }
        } else {
            // Underconsistent
            g[u] = INF_COST;
            updateVertex(u);
            auto outIt = outEdges.find(u);
            if (outIt != outEdges.end()) {
                for (uint64_t tid : outIt->second) {
                    updateVertex(transitionMap[tid].to);
                }
            }
        }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double durationMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();

    PlanningResult res = extractPath();
    res.planningTimeMs = durationMs;
    res.exploredStates = exploredStatesCount;
    return res;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    initProblem(problem);
    return computeShortestPath();
}

void LPAStarPlanner::updateTransition(uint64_t transitionId, bool available, double newCost) {
    if (!isInitialized) return;
    auto it = transitionMap.find(transitionId);
    if (it == transitionMap.end()) return;

    it->second.available = available;
    if (newCost >= 0.0) {
        it->second.cost = newCost;
    }
    updateVertex(it->second.to);
}

void LPAStarPlanner::addTransition(const Transition& t) {
    if (!isInitialized) return;
    transitionMap[t.id] = t;
    outEdges[t.from].push_back(t.id);
    inEdges[t.to].push_back(t.id);
    updateVertex(t.to);
}

void LPAStarPlanner::removeTransition(uint64_t transitionId) {
    if (!isInitialized) return;
    auto it = transitionMap.find(transitionId);
    if (it == transitionMap.end()) return;

    it->second.available = false;
    updateVertex(it->second.to);
}

void LPAStarPlanner::updateBadStates(const std::vector<uint64_t>& bads) {
    if (!isInitialized) return;
    badStateSet.clear();
    for (uint64_t b : bads) {
        badStateSet.insert(b);
    }
    // Update all vertices whose effective costs may have changed
    for (const auto& kv : stateMap) {
        updateVertex(kv.first);
    }
}

void LPAStarPlanner::updateGoal(uint64_t newGoal) {
    if (!isInitialized) return;
    currentProblem.goalState = newGoal;
    // Re-insert all vertices currently in the open set with their updated keys
    std::vector<uint64_t> activeNodes;
    for (const auto& kv : openSetKeys) {
        activeNodes.push_back(kv.first);
    }
    while (!openQueue.empty()) openQueue.pop();
    openSetKeys.clear();

    for (uint64_t u : activeNodes) {
        LPAKey k = calculateKey(u);
        openSetKeys[u] = k;
        openQueue.push(QueueNode{u, k});
    }
}

PlanningResult LPAStarPlanner::extractPath() {
    PlanningResult res;
    double gGoal = getGValue(currentProblem.goalState);

    if (gGoal >= INF_COST / 2.0) {
        res.success = false;
        res.totalCost = INF_COST;
        res.safetyScore = 0.0;
        res.minSafetyDistance = 0.0;
        res.cumulativeReliability = 0.0;
        res.compositeScore = 0.0;
        res.badStatesVisited = 0;
        return res;
    }

    // Reconstruct path forwards from start to goal using greedy predecessor back-pointers or backward search
    std::vector<uint64_t> pathStates;
    std::vector<uint64_t> pathTransitions;

    uint64_t curr = currentProblem.goalState;
    pathStates.push_back(curr);
    std::unordered_set<uint64_t> visited;
    visited.insert(curr);

    while (curr != currentProblem.initialState) {
        double bestPredVal = INF_COST;
        uint64_t bestPred = curr;
        uint64_t bestTid = 0;

        auto inIt = inEdges.find(curr);
        if (inIt != inEdges.end()) {
            for (uint64_t tid : inIt->second) {
                const auto& trans = transitionMap[tid];
                double gPred = getGValue(trans.from);
                double cEff = effectiveTransitionCost(trans);
                if (gPred < INF_COST / 2.0 && cEff < INF_COST / 2.0) {
                    double val = gPred + cEff;
                    if (val < bestPredVal) {
                        bestPredVal = val;
                        bestPred = trans.from;
                        bestTid = tid;
                    }
                }
            }
        }

        if (bestPred == curr || visited.find(bestPred) != visited.end()) {
            // Cycle or unreachable predecessor
            break;
        }

        pathStates.push_back(bestPred);
        pathTransitions.push_back(bestTid);
        visited.insert(bestPred);
        curr = bestPred;
    }

    if (curr != currentProblem.initialState) {
        res.success = false;
        return res;
    }

    std::reverse(pathStates.begin(), pathStates.end());
    std::reverse(pathTransitions.begin(), pathTransitions.end());

    res.success = true;
    res.statePath = pathStates;
    res.transitionPath = pathTransitions;

    // Compute exact metrics
    double totalRawCost = 0.0;
    double cumReliability = 1.0;
    double minSafeDist = INF_COST;
    uint64_t badVisits = 0;

    for (uint64_t tid : pathTransitions) {
        const auto& t = transitionMap[tid];
        totalRawCost += t.cost;
        cumReliability *= std::clamp(t.reliability, 0.0, 1.0);
    }

    for (uint64_t sid : pathStates) {
        if (badStateSet.find(sid) != badStateSet.end()) {
            badVisits++;
        }
        auto sIt = stateMap.find(sid);
        if (sIt != stateMap.end() && !badStateSet.empty()) {
            for (uint64_t bid : badStateSet) {
                auto bIt = stateMap.find(bid);
                if (bIt != stateMap.end()) {
                    double d = sIt->second.euclideanDistance(bIt->second);
                    if (d < minSafeDist) minSafeDist = d;
                }
            }
        }
    }

    if (minSafeDist >= INF_COST / 2.0) minSafeDist = 100.0;

    res.totalCost = totalRawCost;
    res.cumulativeReliability = cumReliability;
    res.minSafetyDistance = minSafeDist;
    res.badStatesVisited = badVisits;

    double goalComp = res.success ? 1.0 : 0.0;
    res.safetyScore = minSafeDist;
    // Score(P) = alpha*G - beta*C + gamma*D + delta*R
    res.compositeScore = (alpha * goalComp) - (beta * totalRawCost) + (gamma * minSafeDist) + (delta * cumReliability);

    return res;
}
