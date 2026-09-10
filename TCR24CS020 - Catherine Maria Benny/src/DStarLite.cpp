#include "DStarLite.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

DStarLite::DStarLite() {}

double DStarLite::gOf(uint64_t s) const {
    auto it = g_.find(s);
    return it == g_.end() ? INF : it->second;
}
double DStarLite::rhsOf(uint64_t s) const {
    auto it = rhs_.find(s);
    return it == rhs_.end() ? INF : it->second;
}

double DStarLite::heuristic(uint64_t a, uint64_t b) const {
    auto ia = states_.find(a);
    auto ib = states_.find(b);
    if (ia == states_.end() || ib == states_.end()) return 0.0;
    // Scaled by beta because the search minimizes beta*cost + (non-negative
    // gamma/delta penalty terms) rather than raw cost. Admissibility (h must
    // never OVERestimate the true remaining composite cost) requires
    // cost(u,v) >= EuclideanDistance(u,v) for every edge -- an invariant the
    // graph generator and the hand-built test scenarios both enforce (see
    // GraphGenerator.cpp and TestScenarios.cpp). Because gamma/delta terms
    // are always >= 0, they only ever make the true cost larger, so scaling
    // by beta alone keeps h(s,start) a valid lower bound. See report.md
    // section 9 for the full admissibility argument.
    return weights.beta * metrics::euclideanDistance(ia->second.embedding, ib->second.embedding);
}

void DStarLite::recomputeNearestBadDistances() {
    nearestBadDist_.clear();
    if (badStates_.empty()) {
        for (const auto& kv : states_) nearestBadDist_[kv.first] = INF;
        return;
    }
    for (const auto& kv : states_) {
        double best = INF;
        for (uint64_t b : badStates_) {
            auto ib = states_.find(b);
            if (ib == states_.end()) continue;
            double d = metrics::euclideanDistance(kv.second.embedding, ib->second.embedding);
            if (d < best) best = d;
        }
        nearestBadDist_[kv.first] = best;
    }
}

double DStarLite::edgeCost(uint64_t u, uint64_t v, uint64_t* usedTransitionId) const {
    // Bad states (other than the goal, which is validated to never be bad)
    // make every incoming edge cost +inf: a HARD constraint, not a penalty.
    // No amount of gamma/delta weighting below can ever make this "cheap
    // enough" -- it is categorically excluded, which is what section 5 of
    // the assignment ("Safety is a hard constraint") requires.
    if (badStates_.count(v) && v != goalState_) return INF;
    auto it = outTransitionIds_.find(u);
    if (it == outTransitionIds_.end()) return INF;
    double best = INF;
    uint64_t bestId = 0;
    static constexpr double kEps = 1e-6;
    for (uint64_t tid : it->second) {
        const Transition& t = transitions_.at(tid);
        if (t.to != v) continue;
        if (!t.available) continue;

        double proximityPenalty = 0.0; // soft: how close v is to the nearest bad state
        auto nd = nearestBadDist_.find(v);
        if (nd != nearestBadDist_.end() && std::isfinite(nd->second)) {
            proximityPenalty = 1.0 / (kEps + nd->second);
        }
        double edgeSafetyPenalty = (1.0 - t.safety) + proximityPenalty;
        double reliabilityPenalty = -std::log(std::max(t.reliability, 1e-9));

        double composite = weights.beta * t.cost +
                            weights.gamma * edgeSafetyPenalty +
                            weights.delta * reliabilityPenalty;
        if (composite < best) { best = composite; bestId = tid; }
    }
    if (usedTransitionId) *usedTransitionId = bestId;
    return best;
}

DStarLite::Key DStarLite::calculateKey(uint64_t s) const {
    double m = std::min(gOf(s), rhsOf(s));
    return Key{ m + heuristic(s, startState_) + km_, m };
}

void DStarLite::queueInsertOrUpdate(uint64_t s, const Key& k) {
    openKey_[s] = k;
    open_.push(QueueEntry{k, s});
}
void DStarLite::queueRemove(uint64_t s) { openKey_.erase(s); }

void DStarLite::cleanStaleTop() {
    while (!open_.empty()) {
        const QueueEntry& top = open_.top();
        auto it = openKey_.find(top.id);
        if (it == openKey_.end() || !(it->second == top.key)) {
            open_.pop(); // stale entry (superseded or removed)
        } else {
            break;
        }
    }
}
bool DStarLite::queueEmpty() { cleanStaleTop(); return open_.empty(); }
DStarLite::Key DStarLite::queueTopKey() { cleanStaleTop(); return open_.top().key; }
uint64_t DStarLite::queuePeekTopId() { cleanStaleTop(); return open_.top().id; }

void DStarLite::updateVertex(uint64_t u) {
    if (u != goalState_) {
        double best = INF;
        auto it = outTransitionIds_.find(u);
        if (it != outTransitionIds_.end()) {
            // group by destination to reuse edgeCost's "best parallel edge" logic
            std::unordered_set<uint64_t> destinationsSeen;
            for (uint64_t tid : it->second) {
                uint64_t v = transitions_.at(tid).to;
                if (destinationsSeen.count(v)) continue;
                destinationsSeen.insert(v);
                double c = edgeCost(u, v);
                if (c == INF) continue;
                double val = c + gOf(v);
                if (val < best) best = val;
            }
        }
        rhs_[u] = best;
    }
    queueRemove(u);
    if (gOf(u) != rhsOf(u)) {
        queueInsertOrUpdate(u, calculateKey(u));
    }
}

void DStarLite::computeShortestPath() {
    int guard = 0;
    int maxIterations = static_cast<int>(states_.size()) * 50 + 1000; // safety bound
    while (!queueEmpty() &&
           (queueTopKey() < calculateKey(startState_) || rhsOf(startState_) != gOf(startState_))) {
        if (++guard > maxIterations) break; // defensive; should not trigger on a correct graph
        uint64_t u = queuePeekTopId();
        Key kOld = openKey_[u];
        Key kNew = calculateKey(u);
        if (kOld < kNew) {
            queueInsertOrUpdate(u, kNew);
        } else if (gOf(u) > rhsOf(u)) {
            g_[u] = rhsOf(u);
            queueRemove(u);
            exploredStatesThisCall_++;
            auto it = inTransitionIds_.find(u);
            if (it != inTransitionIds_.end()) {
                std::unordered_set<uint64_t> preds;
                for (uint64_t tid : it->second) preds.insert(transitions_.at(tid).from);
                for (uint64_t p : preds) updateVertex(p);
            }
        } else {
            g_[u] = INF;
            exploredStatesThisCall_++;
            auto it = inTransitionIds_.find(u);
            std::unordered_set<uint64_t> preds;
            if (it != inTransitionIds_.end())
                for (uint64_t tid : it->second) preds.insert(transitions_.at(tid).from);
            preds.insert(u);
            for (uint64_t p : preds) updateVertex(p);
        }
    }
}

void DStarLite::rebuildAdjacency() {
    outTransitionIds_.clear();
    inTransitionIds_.clear();
    for (const auto& kv : transitions_) {
        const Transition& t = kv.second;
        outTransitionIds_[t.from].push_back(t.id);
        inTransitionIds_[t.to].push_back(t.id);
    }
}

void DStarLite::initializeSearch() {
    g_.clear();
    rhs_.clear();
    while (!open_.empty()) open_.pop();
    openKey_.clear();
    km_ = 0.0;
    lastStartForKm_ = startState_;
    rhs_[goalState_] = 0.0;
    queueInsertOrUpdate(goalState_, calculateKey(goalState_));
}

PlanningResult DStarLite::reconstructAndValidate() {
    PlanningResult result;

    if (badStates_.count(startState_)) {
        result.success = false;
        result.errorMessage = "initial state is a bad state";
        return result;
    }
    if (badStates_.count(goalState_)) {
        result.success = false;
        result.errorMessage = "goal state is a bad state";
        return result;
    }
    if (states_.find(startState_) == states_.end()) {
        result.success = false;
        result.errorMessage = "initial state does not exist";
        return result;
    }
    if (states_.find(goalState_) == states_.end()) {
        result.success = false;
        result.errorMessage = "goal state does not exist";
        return result;
    }

    if (startState_ == goalState_) {
        result.success = true;
        result.statePath = {startState_};
        result.transitionPath = {};
        result.totalCost = 0.0;
        result.badStatesVisited = 0;
    } else {
        if (gOf(startState_) == INF) {
            result.success = false;
            result.errorMessage = "goal is unreachable from the initial state";
            return result;
        }
        std::vector<uint64_t> statePath{startState_};
        std::vector<uint64_t> transitionPath;
        uint64_t current = startState_;
        std::unordered_set<uint64_t> visited{current};
        size_t maxSteps = states_.size() + 1;
        bool reachedGoal = false;
        for (size_t step = 0; step < maxSteps; ++step) {
            if (current == goalState_) { reachedGoal = true; break; }
            auto it = outTransitionIds_.find(current);
            if (it == outTransitionIds_.end()) break;
            double bestVal = INF;
            uint64_t bestNext = 0, bestTid = 0;
            std::unordered_set<uint64_t> destinationsSeen;
            for (uint64_t tid : it->second) {
                uint64_t v = transitions_.at(tid).to;
                if (destinationsSeen.count(v)) continue;
                destinationsSeen.insert(v);
                uint64_t usedTid = 0;
                double c = edgeCost(current, v, &usedTid);
                if (c == INF) continue;
                double val = c + gOf(v);
                if (val < bestVal) { bestVal = val; bestNext = v; bestTid = usedTid; }
            }
            if (bestVal == INF) break; // no viable successor
            statePath.push_back(bestNext);
            transitionPath.push_back(bestTid);
            if (visited.count(bestNext)) break; // defensive: avoid infinite loop on a bug
            visited.insert(bestNext);
            current = bestNext;
        }
        if (!reachedGoal && current != goalState_) {
            result.success = false;
            result.errorMessage = "path reconstruction failed to reach the goal";
            return result;
        }
        result.success = true;
        result.statePath = statePath;
        result.transitionPath = transitionPath;
    }

    // Independent validation, per the assignment's requirement (section 30 /
    // 39): re-derive metrics purely from problem data, not from g/rhs.
    PlanningProblem problem;
    problem.initialState = startState_;
    problem.goalState = goalState_;
    problem.badStates.assign(badStates_.begin(), badStates_.end());
    for (const auto& kv : states_) problem.states.push_back(kv.second);
    for (const auto& kv : transitions_) problem.transitions.push_back(kv.second);

    std::string err = metrics::validatePath(problem, result.statePath, result.transitionPath);
    if (!err.empty()) {
        result.success = false;
        result.errorMessage = "path validation failed: " + err;
        return result;
    }

    result.badStatesVisited = 0;
    for (uint64_t sid : result.statePath) {
        if (badStates_.count(sid)) result.badStatesVisited++;
    }

    result.totalCost = metrics::totalCost(result.transitionPath, transitions_);
    result.reliability = metrics::cumulativeReliability(result.transitionPath, transitions_);
    result.safetyScore = metrics::minimumSafetyDistance(result.statePath, problem.badStates, states_);
    result.objectiveScore = metrics::objectiveScore(result.success, result.totalCost,
                                                      result.safetyScore, result.reliability, weights);
    result.exploredStates = exploredStatesThisCall_;
    result.memoryBytesEstimate =
        states_.size() * sizeof(State) + transitions_.size() * sizeof(Transition) +
        g_.size() * (sizeof(uint64_t) + sizeof(double)) * 2 + open_.size() * sizeof(QueueEntry);
    return result;
}

PlanningResult DStarLite::plan(const PlanningProblem& problem) {
    states_.clear();
    transitions_.clear();
    badStates_.clear();
    for (const auto& s : problem.states) states_[s.id] = s;
    for (const auto& t : problem.transitions) transitions_[t.id] = t;
    for (auto b : problem.badStates) badStates_.insert(b);
    startState_ = problem.initialState;
    goalState_ = problem.goalState;
    rebuildAdjacency();
    recomputeNearestBadDistances();

    exploredStatesThisCall_ = 0;
    auto t0 = std::chrono::high_resolution_clock::now();

    if (badStates_.count(startState_) || badStates_.count(goalState_) ||
        states_.find(startState_) == states_.end() || states_.find(goalState_) == states_.end()) {
        initialized_ = true; // so subsequent dynamic calls don't crash
        auto res = reconstructAndValidate();
        auto t1 = std::chrono::high_resolution_clock::now();
        res.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
        return res;
    }

    initializeSearch();
    computeShortestPath();
    initialized_ = true;

    auto result = reconstructAndValidate();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLite::replan() {
    exploredStatesThisCall_ = 0;
    auto t0 = std::chrono::high_resolution_clock::now();
    if (!badStates_.count(startState_) && !badStates_.count(goalState_) &&
        states_.count(startState_) && states_.count(goalState_)) {
        computeShortestPath();
    }
    auto result = reconstructAndValidate();
    auto t1 = std::chrono::high_resolution_clock::now();
    result.planningTimeMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult DStarLite::setTransitionAvailability(uint64_t transitionId, bool available) {
    auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) {
        PlanningResult r;
        r.success = false;
        r.errorMessage = "transition does not exist";
        return r;
    }
    it->second.available = available;
    updateVertex(it->second.from); // only the tail's rhs is directly affected
    return replan();
}

PlanningResult DStarLite::addTransition(const Transition& t) {
    transitions_[t.id] = t;
    outTransitionIds_[t.from].push_back(t.id);
    inTransitionIds_[t.to].push_back(t.id);
    updateVertex(t.from);
    return replan();
}

PlanningResult DStarLite::removeTransition(uint64_t transitionId) {
    return setTransitionAvailability(transitionId, false);
}

PlanningResult DStarLite::addBadState(uint64_t stateId) {
    badStates_.insert(stateId);
    // Because edgeCost's soft proximity-penalty term depends on distance to
    // the NEAREST bad state, adding one bad state can change the composite
    // cost of edges throughout a neighborhood, not just edges directly
    // entering it. We recompute the (cheap, O(|S|*|B|)) distance table and
    // then call UpdateVertex on every vertex so ComputeShortestPath's
    // incremental propagation re-settles exactly the vertices whose rhs
    // actually changed -- this is still much cheaper than initializeSearch()
    // because it reuses every g-value that turns out not to have changed as
    // a warm start, rather than resetting the whole search from scratch.
    recomputeNearestBadDistances();
    for (const auto& kv : states_) updateVertex(kv.first);
    return replan();
}

PlanningResult DStarLite::removeBadState(uint64_t stateId) {
    badStates_.erase(stateId);
    recomputeNearestBadDistances();
    for (const auto& kv : states_) updateVertex(kv.first);
    return replan();
}

PlanningResult DStarLite::updateGoal(uint64_t newGoal) {
    uint64_t oldGoal = goalState_;
    goalState_ = newGoal;
    rhs_[newGoal] = 0.0;
    queueRemove(newGoal);
    if (gOf(newGoal) != 0.0) queueInsertOrUpdate(newGoal, calculateKey(newGoal));
    updateVertex(oldGoal); // oldGoal is now an ordinary node; recompute its rhs normally
    return replan();
}

PlanningResult DStarLite::moveStart(uint64_t newStart) {
    km_ += heuristic(lastStartForKm_, newStart);
    lastStartForKm_ = newStart;
    startState_ = newStart;
    return replan();
}
