#include "DStarLitePlanner.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>

DStarLitePlanner::DStarLitePlanner(double costWeight, double safetyWeight,
                                     double marginWeight, double heuristicWeight)
    : costWeight_(costWeight), safetyWeight_(safetyWeight),
      marginWeight_(marginWeight), heuristicWeight_(heuristicWeight) {}

// ---------------------------------------------------------------------------
// Loading / adjacency bookkeeping
// ---------------------------------------------------------------------------
void DStarLitePlanner::loadProblem(const PlanningProblem& problem) {
    states_.clear();
    transitions_.clear();
    transitionIndexById_.clear();
    outEdges_.clear();
    inEdges_.clear();
    badStates_.clear();
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    keyInOpenSet_.clear();
    statesExpandedTotal_ = 0;

    for (const auto& s : problem.states) states_[s.id] = s;
    for (auto b : problem.badStates) badStates_.insert(b);
    start_ = problem.initialState;
    goal_ = problem.goalState;
    transitions_ = problem.transitions;

    rebuildAdjacency();
    loaded_ = true;
    initialize();
}

void DStarLitePlanner::rebuildAdjacency() {
    outEdges_.clear();
    inEdges_.clear();
    transitionIndexById_.clear();
    for (size_t i = 0; i < transitions_.size(); ++i) {
        const auto& t = transitions_[i];
        transitionIndexById_[t.id] = i;
        outEdges_[t.from].push_back(i);
        inEdges_[t.to].push_back(i);
    }
}

// ---------------------------------------------------------------------------
// Cost model: this is where the assignment's multi-objective Score(P) gets
// folded into a single scalar the graph search can minimize.
//   - unavailable transitions and any edge touching a bad state -> infinite
//     cost (bad states are a HARD constraint, never a soft penalty)
//   - low reliability multiplicatively inflates cost (an edge that only
//     "works" 50% of the time is modeled as twice as costly to rely on)
//   - low edge-local safety adds an additive penalty
//   - a bonus (marginWeight_) rewards landing on a state that is far from
//     every bad state, which is what drives Test Case 3's cost/safety
//     tradeoff
// ---------------------------------------------------------------------------
double DStarLitePlanner::edgeCost(const Transition& t) const {
    if (!t.available) return INF;
    if (badStates_.count(t.from) || badStates_.count(t.to)) return INF;

    double reliability = std::max(t.reliability, 1e-6);
    double base = (costWeight_ * t.cost + safetyWeight_ * (1.0 - t.safety)) / reliability;

    // Penalize *closeness* to bad states with an inverse-distance term.
    // This is always positive (no clamping needed, unlike a subtracted
    // "bonus for being far" which can blow past zero and erase the signal
    // once marginWeight_ gets large -- that was an earlier bug here).
    double margin = distanceToNearestBadState(t.to);
    const double eps = 0.15; // avoids division blowing up as margin -> 0
    double marginPenalty = marginWeight_ / (margin + eps);

    return base + marginPenalty;
}

double DStarLitePlanner::euclidean(uint64_t a, uint64_t b) const {
    const auto& ea = states_.at(a).embedding;
    const auto& eb = states_.at(b).embedding;
    double sum = 0.0;
    for (size_t i = 0; i < ea.size() && i < eb.size(); ++i) {
        double d = ea[i] - eb[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double DStarLitePlanner::distanceToNearestBadState(uint64_t s) const {
    if (badStates_.empty()) return 5.0; // "infinitely safe" capped at the same 5.0 ceiling used above
    double best = INF;
    for (auto b : badStates_) {
        if (!states_.count(b)) continue;
        best = std::min(best, euclidean(s, b));
    }
    return best;
}

double DStarLitePlanner::heuristic(uint64_t s) const {
    if (heuristicWeight_ == 0.0) return 0.0; // reduces to plain (incremental) Dijkstra
    return heuristicWeight_ * euclidean(s, start_);
}

// ---------------------------------------------------------------------------
// Core D* Lite machinery
// ---------------------------------------------------------------------------
DStarLitePlanner::Key DStarLitePlanner::calculateKey(uint64_t s) const {
    double gs = g_.count(s) ? g_.at(s) : INF;
    double rhss = rhs_.count(s) ? rhs_.at(s) : INF;
    double m = std::min(gs, rhss);
    return Key{ m + heuristic(s), m };
}

void DStarLitePlanner::initialize() {
    for (auto& kv : states_) {
        g_[kv.first] = INF;
        rhs_[kv.first] = INF;
    }
    rhs_[goal_] = 0.0;
    openSet_.clear();
    keyInOpenSet_.clear();
    Key k = calculateKey(goal_);
    openSet_.insert({k, goal_});
    keyInOpenSet_[goal_] = k;
}

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (u != goal_) {
        double best = INF;
        auto it = outEdges_.find(u);
        if (it != outEdges_.end()) {
            for (size_t idx : it->second) {
                const auto& t = transitions_[idx];
                double c = edgeCost(t);
                if (c >= INF) continue;
                double gTo = g_.count(t.to) ? g_.at(t.to) : INF;
                if (gTo >= INF) continue;
                best = std::min(best, c + gTo);
            }
        }
        rhs_[u] = best;
    }

    auto qit = keyInOpenSet_.find(u);
    if (qit != keyInOpenSet_.end()) {
        openSet_.erase({qit->second, u});
        keyInOpenSet_.erase(qit);
    }
    if (g_[u] != rhs_[u]) {
        Key k = calculateKey(u);
        openSet_.insert({k, u});
        keyInOpenSet_[u] = k;
    }
}

void DStarLitePlanner::computeShortestPath(int& expandedThisCall, size_t& peakQueueThisCall) {
    expandedThisCall = 0;
    peakQueueThisCall = openSet_.size();

    while (!openSet_.empty()) {
        Key topKey = openSet_.begin()->first;
        Key startKey = calculateKey(start_);
        bool startInconsistent = (g_[start_] != rhs_[start_]);
        if (!(topKey < startKey) && !startInconsistent) break;

        auto it = openSet_.begin();
        Key kOld = it->first;
        uint64_t u = it->second;
        openSet_.erase(it);
        keyInOpenSet_.erase(u);
        expandedThisCall++;
        statesExpandedTotal_++;

        Key kNew = calculateKey(u);
        if (kOld < kNew) {
            openSet_.insert({kNew, u});
            keyInOpenSet_[u] = kNew;
        } else if (g_[u] > rhs_[u]) {
            g_[u] = rhs_[u];
            auto pit = inEdges_.find(u);
            if (pit != inEdges_.end())
                for (size_t idx : pit->second) updateVertex(transitions_[idx].from);
        } else {
            g_[u] = INF;
            updateVertex(u);
            auto pit = inEdges_.find(u);
            if (pit != inEdges_.end())
                for (size_t idx : pit->second) updateVertex(transitions_[idx].from);
        }
        peakQueueThisCall = std::max(peakQueueThisCall, openSet_.size());
    }
}

// ---------------------------------------------------------------------------
// Greedily walk the g-value gradient from start to goal to read off the path.
// ---------------------------------------------------------------------------
PlanningResult DStarLitePlanner::extractPath() {
    PlanningResult result;
    if (!g_.count(start_) || g_[start_] >= INF) {
        result.success = false;
        return result;
    }

    uint64_t current = start_;
    result.statePath.push_back(current);
    result.safetyScore = distanceToNearestBadState(current);
    if (badStates_.count(current)) result.badStatesVisited++;

    std::unordered_set<uint64_t> visited;
    visited.insert(current);

    int guard = 0;
    const int maxSteps = static_cast<int>(states_.size()) + 5;
    while (current != goal_ && guard++ < maxSteps) {
        double bestVal = INF;
        int bestIdx = -1;
        auto it = outEdges_.find(current);
        if (it != outEdges_.end()) {
            for (size_t idx : it->second) {
                const auto& t = transitions_[idx];
                double c = edgeCost(t);
                if (c >= INF) continue;
                double gTo = g_.count(t.to) ? g_.at(t.to) : INF;
                if (gTo >= INF) continue;
                double val = c + gTo;
                if (val < bestVal) { bestVal = val; bestIdx = static_cast<int>(idx); }
            }
        }
        if (bestIdx < 0) { result.success = false; return result; }

        const auto& chosen = transitions_[bestIdx];
        result.transitionPath.push_back(chosen.id);
        result.totalCost += chosen.cost;
        current = chosen.to;

        if (visited.count(current)) { result.success = false; return result; } // safety net, shouldn't happen
        visited.insert(current);

        result.statePath.push_back(current);
        result.safetyScore = std::min(result.safetyScore, distanceToNearestBadState(current));
        if (badStates_.count(current)) result.badStatesVisited++;
    }

    result.success = (current == goal_);
    return result;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    auto t0 = std::chrono::high_resolution_clock::now();
    loadProblem(problem);
    int expanded; size_t peak;
    computeShortestPath(expanded, peak);
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();

    result.statesExpanded = expanded;
    result.peakOpenSetSize = peak;
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLitePlanner::updateGoal(uint64_t newGoal) {
    // Honest limitation (see REPORT.md): D* Lite's g-values are all defined
    // relative to a fixed goal, so changing the goal invalidates essentially
    // every g-value. The efficient part of D* Lite is replanning after EDGE
    // or AVAILABILITY changes (below), not goal changes -- that's why the
    // literature pairs D* Lite with a *moving robot, fixed goal* setup.
    auto t0 = std::chrono::high_resolution_clock::now();
    goal_ = newGoal;
    initialize();
    int expanded; size_t peak;
    computeShortestPath(expanded, peak);
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.statesExpanded = expanded;
    result.peakOpenSetSize = peak;
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLitePlanner::setTransitionAvailability(uint64_t transitionId, bool available) {
    auto t0 = std::chrono::high_resolution_clock::now();
    auto it = transitionIndexById_.find(transitionId);
    if (it != transitionIndexById_.end()) {
        transitions_[it->second].available = available;
        updateVertex(transitions_[it->second].from);
    }
    int expanded; size_t peak;
    computeShortestPath(expanded, peak);
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.statesExpanded = expanded;
    result.peakOpenSetSize = peak;
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLitePlanner::addTransition(const Transition& t) {
    auto t0 = std::chrono::high_resolution_clock::now();
    transitions_.push_back(t);
    size_t idx = transitions_.size() - 1;
    transitionIndexById_[t.id] = idx;
    outEdges_[t.from].push_back(idx);
    inEdges_[t.to].push_back(idx);
    if (!g_.count(t.from)) { g_[t.from] = INF; rhs_[t.from] = INF; }
    if (!g_.count(t.to)) { g_[t.to] = INF; rhs_[t.to] = INF; }
    updateVertex(t.from);
    int expanded; size_t peak;
    computeShortestPath(expanded, peak);
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.statesExpanded = expanded;
    result.peakOpenSetSize = peak;
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLitePlanner::removeTransition(uint64_t transitionId) {
    // Modeled as a permanent availability=false; the adjacency lists keep
    // the (now-dead) index around, which is fine since edgeCost() will
    // always return infinity for it.
    return setTransitionAvailability(transitionId, false);
}

PlanningResult DStarLitePlanner::addBadState(uint64_t stateId) {
    auto t0 = std::chrono::high_resolution_clock::now();
    badStates_.insert(stateId);
    g_[stateId] = INF;
    rhs_[stateId] = INF;
    auto qit = keyInOpenSet_.find(stateId);
    if (qit != keyInOpenSet_.end()) {
        openSet_.erase({qit->second, stateId});
        keyInOpenSet_.erase(qit);
    }
    // Every predecessor of the newly-bad state may lose its best edge.
    auto pit = inEdges_.find(stateId);
    if (pit != inEdges_.end())
        for (size_t idx : pit->second) updateVertex(transitions_[idx].from);
    // Every successor of the newly-bad state may also need re-checking
    // (their inbound edge from a bad state is now void, but more
    // importantly nodes reached only through here need requeuing).
    auto oit = outEdges_.find(stateId);
    if (oit != outEdges_.end())
        for (size_t idx : oit->second) updateVertex(transitions_[idx].to);

    int expanded; size_t peak;
    computeShortestPath(expanded, peak);
    PlanningResult result = extractPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.statesExpanded = expanded;
    result.peakOpenSetSize = peak;
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}
