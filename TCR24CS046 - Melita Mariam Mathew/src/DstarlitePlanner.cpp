#include "DstarlitePlanner.hpp"
#include <iostream>

constexpr double DSTAR_INF = 1e12;

DstarlitePlanner::DstarlitePlanner(double a, double b, double gVal, double d, double sRadius)
    : alpha(a), beta(b), gamma(gVal), delta(d), safetyRadius(sRadius),
      km(0.0), lastStart(0), exploredStatesCount(0), isInitialized(false) {}

double DstarlitePlanner::heuristic(uint64_t aId, uint64_t bId) const {
    auto itA = stateMap.find(aId);
    auto itB = stateMap.find(bId);
    if (itA == stateMap.end() || itB == stateMap.end()) return 0.0;
    return itA->second.euclideanDistance(itB->second);
}

double DstarlitePlanner::effectiveCost(const Transition& t) const {
    if (!t.available) return DSTAR_INF;
    if (badStateSet.find(t.to) != badStateSet.end() || badStateSet.find(t.from) != badStateSet.end()) {
        return DSTAR_INF;
    }

    double baseC = std::max(0.0001, t.cost) * beta;
    double relPenalty = delta * (1.0 - std::clamp(t.reliability, 0.0, 1.0));

    double safetyPenalty = 0.0;
    auto itTo = stateMap.find(t.to);
    if (itTo != stateMap.end() && !badStateSet.empty()) {
        double minDist = DSTAR_INF;
        for (uint64_t bId : badStateSet) {
            auto itB = stateMap.find(bId);
            if (itB != stateMap.end()) {
                double dist = itTo->second.euclideanDistance(itB->second);
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

double DstarlitePlanner::getG(uint64_t stateId) const {
    auto it = g.find(stateId);
    return (it != g.end()) ? it->second : DSTAR_INF;
}

double DstarlitePlanner::getRhs(uint64_t stateId) const {
    auto it = rhs.find(stateId);
    return (it != rhs.end()) ? it->second : DSTAR_INF;
}

DStarKey DstarlitePlanner::calculateKey(uint64_t stateId) const {
    double gVal = getG(stateId);
    double rhsVal = getRhs(stateId);
    double m = std::min(gVal, rhsVal);
    double hVal = heuristic(currentProblem.initialState, stateId);
    return DStarKey{ m + hVal + km, m };
}

void DstarlitePlanner::updateVertex(uint64_t u) {
    if (u != currentProblem.goalState) {
        double minRhs = DSTAR_INF;
        auto outIt = outEdges.find(u);
        if (outIt != outEdges.end()) {
            for (uint64_t tid : outIt->second) {
                const auto& trans = transitionMap[tid];
                double gSucc = getG(trans.to);
                double cEff = effectiveCost(trans);
                if (gSucc < DSTAR_INF / 2.0 && cEff < DSTAR_INF / 2.0) {
                    double val = cEff + gSucc;
                    if (val < minRhs) minRhs = val;
                }
            }
        }
        rhs[u] = minRhs;
    }

    openSetKeys.erase(u);

    double gVal = getG(u);
    double rhsVal = getRhs(u);
    if (std::abs(gVal - rhsVal) > 1e-9) {
        DStarKey k = calculateKey(u);
        openSetKeys[u] = k;
        openQueue.push(QueueNode{u, k});
    }
}

void DstarlitePlanner::initProblem(const PlanningProblem& problem) {
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
    km = 0.0;
    lastStart = problem.initialState;
    exploredStatesCount = 0;

    for (const auto& s : problem.states) {
        stateMap[s.id] = s;
        g[s.id] = DSTAR_INF;
        rhs[s.id] = DSTAR_INF;
    }
    for (uint64_t b : problem.badStates) {
        badStateSet.insert(b);
    }
    for (const auto& t : problem.transitions) {
        transitionMap[t.id] = t;
        outEdges[t.from].push_back(t.id);
        inEdges[t.to].push_back(t.id);
    }

    rhs[problem.goalState] = 0.0;
    DStarKey kGoal = calculateKey(problem.goalState);
    openSetKeys[problem.goalState] = kGoal;
    openQueue.push(QueueNode{problem.goalState, kGoal});

    isInitialized = true;
}

PlanningResult DstarlitePlanner::computeShortestPath() {
    auto startTime = std::chrono::high_resolution_clock::now();

    while (!openQueue.empty()) {
        QueueNode top = openQueue.top();
        uint64_t u = top.id;
        DStarKey keyTop = top.key;

        auto keyIt = openSetKeys.find(u);
        if (keyIt == openSetKeys.end() || !(keyIt->second <= keyTop && keyTop <= keyIt->second)) {
            openQueue.pop();
            continue;
        }

        DStarKey keyStart = calculateKey(currentProblem.initialState);
        double gStart = getG(currentProblem.initialState);
        double rhsStart = getRhs(currentProblem.initialState);

        if (!(keyTop < keyStart) && std::abs(gStart - rhsStart) <= 1e-9) {
            break;
        }

        DStarKey keyNew = calculateKey(u);
        if (keyTop < keyNew) {
            // Outdated key due to km shift
            openQueue.pop();
            openSetKeys[u] = keyNew;
            openQueue.push(QueueNode{u, keyNew});
            continue;
        }

        openQueue.pop();
        openSetKeys.erase(u);
        exploredStatesCount++;

        double gVal = getG(u);
        double rhsVal = getRhs(u);

        if (gVal > rhsVal) {
            g[u] = rhsVal;
            auto inIt = inEdges.find(u);
            if (inIt != inEdges.end()) {
                for (uint64_t tid : inIt->second) {
                    updateVertex(transitionMap[tid].from);
                }
            }
        } else {
            g[u] = DSTAR_INF;
            updateVertex(u);
            auto inIt = inEdges.find(u);
            if (inIt != inEdges.end()) {
                for (uint64_t tid : inIt->second) {
                    updateVertex(transitionMap[tid].from);
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

PlanningResult DstarlitePlanner::plan(const PlanningProblem& problem) {
    initProblem(problem);
    return computeShortestPath();
}

void DstarlitePlanner::updateTransition(uint64_t transitionId, bool available, double newCost) {
    if (!isInitialized) return;
    auto it = transitionMap.find(transitionId);
    if (it == transitionMap.end()) return;

    it->second.available = available;
    if (newCost >= 0.0) {
        it->second.cost = newCost;
    }
    updateVertex(it->second.from);
}

void DstarlitePlanner::addTransition(const Transition& t) {
    if (!isInitialized) return;
    transitionMap[t.id] = t;
    outEdges[t.from].push_back(t.id);
    inEdges[t.to].push_back(t.id);
    updateVertex(t.from);
}

void DstarlitePlanner::removeTransition(uint64_t transitionId) {
    if (!isInitialized) return;
    auto it = transitionMap.find(transitionId);
    if (it == transitionMap.end()) return;

    it->second.available = false;
    updateVertex(it->second.from);
}

void DstarlitePlanner::updateBadStates(const std::vector<uint64_t>& bads) {
    if (!isInitialized) return;
    badStateSet.clear();
    for (uint64_t b : bads) badStateSet.insert(b);
    for (const auto& kv : stateMap) {
        updateVertex(kv.first);
    }
}

void DstarlitePlanner::updateStart(uint64_t newStart) {
    if (!isInitialized) return;
    km += heuristic(lastStart, newStart);
    lastStart = newStart;
    currentProblem.initialState = newStart;
}

void DstarlitePlanner::updateGoal(uint64_t newGoal) {
    if (!isInitialized) return;
    currentProblem.goalState = newGoal;
    rhs[newGoal] = 0.0;
    for (const auto& kv : stateMap) {
        updateVertex(kv.first);
    }
}

PlanningResult DstarlitePlanner::extractPath() {
    PlanningResult res;
    double gStart = getG(currentProblem.initialState);

    if (gStart >= DSTAR_INF / 2.0) {
        res.success = false;
        res.totalCost = DSTAR_INF;
        return res;
    }

    std::vector<uint64_t> pathStates;
    std::vector<uint64_t> pathTransitions;

    uint64_t curr = currentProblem.initialState;
    pathStates.push_back(curr);
    std::unordered_set<uint64_t> visited;
    visited.insert(curr);

    while (curr != currentProblem.goalState) {
        double bestCost = DSTAR_INF;
        uint64_t bestNext = curr;
        uint64_t bestTid = 0;

        auto outIt = outEdges.find(curr);
        if (outIt != outEdges.end()) {
            for (uint64_t tid : outIt->second) {
                const auto& trans = transitionMap[tid];
                double cEff = effectiveCost(trans);
                double gSucc = getG(trans.to);
                if (cEff < DSTAR_INF / 2.0 && gSucc < DSTAR_INF / 2.0) {
                    double val = cEff + gSucc;
                    if (val < bestCost) {
                        bestCost = val;
                        bestNext = trans.to;
                        bestTid = tid;
                    }
                }
            }
        }

        if (bestNext == curr || visited.find(bestNext) != visited.end()) {
            break;
        }

        pathStates.push_back(bestNext);
        pathTransitions.push_back(bestTid);
        visited.insert(bestNext);
        curr = bestNext;
    }

    if (curr != currentProblem.goalState) {
        res.success = false;
        return res;
    }

    res.success = true;
    res.statePath = pathStates;
    res.transitionPath = pathTransitions;

    double totalRawCost = 0.0;
    double cumReliability = 1.0;
    double minSafeDist = DSTAR_INF;
    uint64_t badVisits = 0;

    for (uint64_t tid : pathTransitions) {
        const auto& t = transitionMap[tid];
        totalRawCost += t.cost;
        cumReliability *= std::clamp(t.reliability, 0.0, 1.0);
    }

    for (uint64_t sid : pathStates) {
        if (badStateSet.find(sid) != badStateSet.end()) badVisits++;
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

    if (minSafeDist >= DSTAR_INF / 2.0) minSafeDist = 100.0;

    res.totalCost = totalRawCost;
    res.cumulativeReliability = cumReliability;
    res.minSafetyDistance = minSafeDist;
    res.badStatesVisited = badVisits;
    res.safetyScore = minSafeDist;
    res.compositeScore = (alpha * 1.0) - (beta * totalRawCost) + (gamma * minSafeDist) + (delta * cumReliability);

    return res;
}
