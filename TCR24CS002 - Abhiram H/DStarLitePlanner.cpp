#include "DStarLitePlanner.hpp"
#include <algorithm>
#include <chrono>

DStarLitePlanner::DStarLitePlanner() : weights_(Weights()) {}
DStarLitePlanner::DStarLitePlanner(Weights weights) : weights_(weights) {}

double DStarLitePlanner::edgeCost(const Transition& t) const {
    if (!t.available) return INF;
    if (badStates_.count(t.to)) return INF; // never traverse into a bad state

    double c = t.cost;
    c += weights_.safety * (1.0 - t.safety);
    c += weights_.reliability * (1.0 - t.reliability);

    auto it = stateOf_.find(t.to);
    if (it != stateOf_.end() && !badStates_.empty()) {
        double best = INF;
        for (uint64_t b : badStates_) {
            auto bs = stateOf_.find(b);
            if (bs == stateOf_.end()) continue;
            best = std::min(best, State::euclidean(it->second, bs->second));
        }
        if (best < INF) {
            c += weights_.proximity / (best + weights_.proximityEps);
        }
    }
    return c;
}

double DStarLitePlanner::heuristic(uint64_t s) const {
    auto sIt = stateOf_.find(s);
    auto startIt = stateOf_.find(start_);
    if (sIt == stateOf_.end() || startIt == stateOf_.end()) return 0.0;
    return minCostRate_ * State::euclidean(sIt->second, startIt->second);
}

double DStarLitePlanner::gOf(uint64_t s) const {
    auto it = g_.find(s);
    return it == g_.end() ? INF : it->second;
}

double DStarLitePlanner::rhsOf(uint64_t s) const {
    if (s == goal_) return 0.0;
    auto it = rhs_.find(s);
    return it == rhs_.end() ? INF : it->second;
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(uint64_t s) const {
    double m = std::min(gOf(s), rhsOf(s));
    return Key{m + heuristic(s) + km_, m};
}

void DStarLitePlanner::insertOrUpdateOpen(uint64_t s, const Key& k) {
    removeFromOpen(s);
    openSet_.insert({k, s});
    openKeyOf_[s] = k;
}

void DStarLitePlanner::removeFromOpen(uint64_t s) {
    auto it = openKeyOf_.find(s);
    if (it != openKeyOf_.end()) {
        openSet_.erase({it->second, s});
        openKeyOf_.erase(it);
    }
}

void DStarLitePlanner::recomputeMinCostRate() {
    double best = INF;
    for (const auto& [id, t] : transitionOf_) {
        auto a = stateOf_.find(t.from), b = stateOf_.find(t.to);
        if (a == stateOf_.end() || b == stateOf_.end()) continue;
        double d = State::euclidean(a->second, b->second);
        if (d > 1e-9) {
            best = std::min(best, t.cost / d);
        }
    }
    minCostRate_ = (best == INF) ? 0.0 : best; // 0.0 keeps heuristic admissible (falls back to Dijkstra)
}

void DStarLitePlanner::initialize(const PlanningProblem& problem) {
    stateOf_.clear();
    transitionOf_.clear();
    outEdges_.clear();
    inEdges_.clear();
    badStates_.clear();
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    openKeyOf_.clear();
    km_ = 0.0;
    exploredCount_ = 0;

    for (const auto& s : problem.states) stateOf_[s.id] = s;
    for (const auto& t : problem.transitions) {
        transitionOf_[t.id] = t;
        outEdges_[t.from].push_back(t.id);
        inEdges_[t.to].push_back(t.id);
    }
    for (uint64_t b : problem.badStates) badStates_.insert(b);

    start_ = problem.initialState;
    goal_ = problem.goalState;
    lastStartForHeuristic_ = start_;

    recomputeMinCostRate();

    rhs_[goal_] = 0.0;
    insertOrUpdateOpen(goal_, calculateKey(goal_));
}

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (u != goal_) {
        double best = INF;
        for (uint64_t tid : outEdges_[u]) {
            const Transition& t = transitionOf_[tid];
            double c = edgeCost(t);
            if (c >= INF) continue;
            best = std::min(best, c + gOf(t.to));
        }
        rhs_[u] = best;
    }
    removeFromOpen(u);
    if (gOf(u) != rhsOf(u)) {
        insertOrUpdateOpen(u, calculateKey(u));
    }
}

void DStarLitePlanner::computeShortestPath() {
    while (!openSet_.empty()) {
        auto topIt = openSet_.begin();
        Key kOld = topIt->first;
        uint64_t u = topIt->second;
        Key kStart = calculateKey(start_);

        bool startNotConsistent = (gOf(start_) != rhsOf(start_));
        if (!(kOld < kStart) && !startNotConsistent) break;

        exploredCount_++;
        Key kNew = calculateKey(u);
        if (kOld < kNew) {
            insertOrUpdateOpen(u, kNew);
            continue;
        }

        if (gOf(u) > rhsOf(u)) {
            g_[u] = rhsOf(u);
            removeFromOpen(u);
            for (uint64_t tid : inEdges_[u]) {
                updateVertex(transitionOf_[tid].from);
            }
        } else {
            g_[u] = INF;
            for (uint64_t tid : inEdges_[u]) {
                updateVertex(transitionOf_[tid].from);
            }
            updateVertex(u);
        }
    }
}

PlanningResult DStarLitePlanner::extractPath() {
    PlanningResult result;
    if (gOf(start_) >= INF) {
        result.success = false;
        result.exploredStates = exploredCount_;
        return result;
    }

    result.success = true;
    result.statePath.push_back(start_);
    double totalCost = 0.0, minDistToBad = INF, cumulativeReliability = 0.0;

    uint64_t current = start_;
    std::unordered_set<uint64_t> visited;
    int guard = 0;
    int maxSteps = (int)stateOf_.size() + 5;

    while (current != goal_ && guard++ < maxSteps) {
        if (visited.count(current)) { result.success = false; break; }
        visited.insert(current);

        double bestCost = INF;
        uint64_t bestTid = 0, bestNext = 0;
        for (uint64_t tid : outEdges_[current]) {
            const Transition& t = transitionOf_[tid];
            double c = edgeCost(t);
            if (c >= INF) continue;
            double total = c + gOf(t.to);
            if (total < bestCost) {
                bestCost = total;
                bestTid = tid;
                bestNext = t.to;
            }
        }
        if (bestCost >= INF) { result.success = false; break; }

        const Transition& chosen = transitionOf_[bestTid];
        totalCost += chosen.cost; // report raw cost, not the safety-augmented search cost
        cumulativeReliability += chosen.reliability;
        result.transitionPath.push_back(bestTid);
        result.statePath.push_back(bestNext);

        auto it = stateOf_.find(bestNext);
        if (it != stateOf_.end() && !badStates_.empty()) {
            for (uint64_t b : badStates_) {
                auto bs = stateOf_.find(b);
                if (bs == stateOf_.end()) continue;
                minDistToBad = std::min(minDistToBad, State::euclidean(it->second, bs->second));
            }
        }
        current = bestNext;
    }

    if (current != goal_) result.success = false;

    result.totalCost = totalCost;
    result.safetyScore = (minDistToBad == INF) ? 0.0 : minDistToBad;
    result.exploredStates = exploredCount_;
    result.cumulativeReliability = cumulativeReliability;
    return result;
}

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    auto t0 = std::chrono::high_resolution_clock::now();
    initialize(problem);
    computeShortestPath();
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLitePlanner::replan() {
    auto t0 = std::chrono::high_resolution_clock::now();
    computeShortestPath();
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

void DStarLitePlanner::setEdgeAvailability(uint64_t transitionId, bool available) {
    auto it = transitionOf_.find(transitionId);
    if (it == transitionOf_.end()) return;
    it->second.available = available;
    km_ += heuristic(lastStartForHeuristic_);
    lastStartForHeuristic_ = start_;
    updateVertex(it->second.from);
}

void DStarLitePlanner::addTransition(const Transition& t) {
    transitionOf_[t.id] = t;
    outEdges_[t.from].push_back(t.id);
    inEdges_[t.to].push_back(t.id);
    km_ += heuristic(lastStartForHeuristic_);
    lastStartForHeuristic_ = start_;
    updateVertex(t.from);
}

void DStarLitePlanner::removeTransition(uint64_t transitionId) {
    auto it = transitionOf_.find(transitionId);
    if (it == transitionOf_.end()) return;
    uint64_t from = it->second.from, to = it->second.to;
    auto eraseFrom = [&](std::vector<uint64_t>& v) {
        v.erase(std::remove(v.begin(), v.end(), transitionId), v.end());
    };
    eraseFrom(outEdges_[from]);
    eraseFrom(inEdges_[to]);
    transitionOf_.erase(it);
    km_ += heuristic(lastStartForHeuristic_);
    lastStartForHeuristic_ = start_;
    updateVertex(from);
}

void DStarLitePlanner::addBadState(uint64_t stateId) {
    badStates_.insert(stateId);
    km_ += heuristic(lastStartForHeuristic_);
    lastStartForHeuristic_ = start_;
    // Every predecessor of the newly-bad state may now need a cheaper route.
    for (uint64_t tid : inEdges_[stateId]) {
        updateVertex(transitionOf_[tid].from);
    }
    updateVertex(stateId);
}

void DStarLitePlanner::removeBadState(uint64_t stateId) {
    badStates_.erase(stateId);
    km_ += heuristic(lastStartForHeuristic_);
    lastStartForHeuristic_ = start_;
    for (uint64_t tid : inEdges_[stateId]) {
        updateVertex(transitionOf_[tid].from);
    }
    updateVertex(stateId);
}

void DStarLitePlanner::updateStart(uint64_t newStartId) {
    km_ += heuristic(newStartId); // heuristic(newStart) measured w.r.t. OLD start, per D* Lite
    start_ = newStartId;
    lastStartForHeuristic_ = start_;
}

void DStarLitePlanner::updateGoal(uint64_t newGoalId) {
    // Classical D* Lite anchors rhs at a fixed goal; changing the goal
    // invalidates that anchor, so we perform a bounded reinitialization:
    // keep the graph and bad-state set, but reset g/rhs and reopen from the
    // new goal. This is still far cheaper than rebuilding the graph itself
    // (no re-parsing, no re-allocation of adjacency lists) — see
    // docs/DesignReport.md, Section "Replanning after goal changes".
    goal_ = newGoalId;
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    openKeyOf_.clear();
    km_ = 0.0;
    lastStartForHeuristic_ = start_;
    rhs_[goal_] = 0.0;
    insertOrUpdateOpen(goal_, calculateKey(goal_));
}
