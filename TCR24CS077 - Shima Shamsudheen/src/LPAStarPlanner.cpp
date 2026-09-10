#include "LPAStarPlanner.h"
#include <cmath>
#include <algorithm>
#include <chrono>

LPAStarPlanner::LPAStarPlanner() {}

double LPAStarPlanner::euclidean(uint64_t a, uint64_t b) const {
    const auto& ea = states_.at(a).embedding;
    const auto& eb = states_.at(b).embedding;
    double sum = 0.0;
    size_t d = std::min(ea.size(), eb.size());
    for (size_t i = 0; i < d; ++i) {
        double diff = ea[i] - eb[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double LPAStarPlanner::gOf(uint64_t s) const {
    auto it = g_.find(s);
    return it == g_.end() ? INF : it->second;
}

double LPAStarPlanner::rhsOf(uint64_t s) const {
    auto it = rhs_.find(s);
    return it == rhs_.end() ? INF : it->second;
}

void LPAStarPlanner::recomputeClearances() {
    clearance_.clear();
    for (const auto& kv : states_) {
        uint64_t sid = kv.first;
        if (badStates_.empty()) {
            clearance_[sid] = INF;
            continue;
        }
        double best = INF;
        for (uint64_t b : badStates_) {
            if (states_.count(b) == 0) continue; // bad "state" id not in state set - ignore defensively
            best = std::min(best, euclidean(sid, b));
        }
        clearance_[sid] = best;
    }
}

double LPAStarPlanner::heuristic(uint64_t s) const {
    if (HEURISTIC_WEIGHT <= 0.0) return 0.0;
    return HEURISTIC_WEIGHT * euclidean(s, goalState_) * W_COST;
}

double LPAStarPlanner::edgeWeight(const Transition& t) const {
    if (!t.available) return INF;
    if (currentTime_ < t.availableFrom || currentTime_ >= t.availableUntil) return INF; // bonus: time-dependent availability
    if (badStates_.count(t.to)) return INF; // hard constraint: never route into a bad state

    double clearance = clearance_.count(t.to) ? clearance_.at(t.to) : INF;
    double safetyPenalty = 0.0;
    if (clearance < SAFETY_RADIUS) {
        safetyPenalty = (SAFETY_RADIUS - clearance); // linear potential-field penalty
    }
    double reliabilityPenalty = std::max(0.0, 1.0 - t.reliability);

    return W_COST * t.cost + W_SAFETY * safetyPenalty + W_RELIABILITY * reliabilityPenalty;
}

LPAStarPlanner::Key LPAStarPlanner::calculateKey(uint64_t s) const {
    double g = gOf(s), rhs = rhsOf(s);
    double m = std::min(g, rhs);
    return Key{ m == INF ? INF : m + heuristic(s), m };
}

void LPAStarPlanner::initialize(const PlanningProblem& problem) {
    states_.clear();
    transitionsById_.clear();
    outgoing_.clear();
    incoming_.clear();
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    inQueueKey_.clear();
    statesExplored_ = 0;

    for (const auto& s : problem.states) states_[s.id] = s;
    for (const auto& t : problem.transitions) {
        transitionsById_[t.id] = t;
        outgoing_[t.from].push_back(t.id);
        incoming_[t.to].push_back(t.id);
    }
    badStates_ = std::set<uint64_t>(problem.badStates.begin(), problem.badStates.end());
    startState_ = problem.initialState;
    goalState_ = problem.goalState;

    recomputeClearances();

    for (const auto& kv : states_) {
        g_[kv.first] = INF;
        rhs_[kv.first] = INF;
    }
    rhs_[startState_] = 0.0;

    Key k = calculateKey(startState_);
    openSet_.insert(QueueEntry{k, startState_});
    inQueueKey_[startState_] = k;

    initialized_ = true;
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != startState_) {
        double best = INF;
        auto it = incoming_.find(u);
        if (it != incoming_.end()) {
            for (uint64_t tid : it->second) {
                const Transition& t = transitionsById_.at(tid);
                double w = edgeWeight(t);
                if (w == INF) continue;
                double cand = gOf(t.from) + w;
                if (cand < best) best = cand;
            }
        }
        rhs_[u] = best;
    }

    auto qit = inQueueKey_.find(u);
    if (qit != inQueueKey_.end()) {
        openSet_.erase(QueueEntry{qit->second, u});
        inQueueKey_.erase(qit);
    }

    if (gOf(u) != rhsOf(u)) {
        Key k = calculateKey(u);
        openSet_.insert(QueueEntry{k, u});
        inQueueKey_[u] = k;
    }
}

void LPAStarPlanner::computeShortestPath() {
    while (!openSet_.empty()) {
        Key topKey = openSet_.begin()->key;
        Key goalKey = calculateKey(goalState_);
        bool topLessThanGoal = topKey < goalKey;
        if (!topLessThanGoal && gOf(goalState_) == rhsOf(goalState_)) break;

        uint64_t u = openSet_.begin()->id;
        openSet_.erase(openSet_.begin());
        inQueueKey_.erase(u);
        statesExplored_++;

        if (gOf(u) > rhsOf(u)) {
            g_[u] = rhsOf(u);
            auto it = outgoing_.find(u);
            if (it != outgoing_.end()) {
                for (uint64_t tid : it->second) {
                    updateVertex(transitionsById_.at(tid).to);
                }
            }
        } else {
            g_[u] = INF;
            updateVertex(u);
            auto it = outgoing_.find(u);
            if (it != outgoing_.end()) {
                for (uint64_t tid : it->second) {
                    updateVertex(transitionsById_.at(tid).to);
                }
            }
        }
    }
}

PlanningResult LPAStarPlanner::extractPath(double elapsedMs, bool isReplan) {
    PlanningResult result;
    result.statesExplored = statesExplored_;
    if (isReplan) result.replanTimeMs = elapsedMs; else result.planningTimeMs = elapsedMs;

    if (gOf(goalState_) == INF) {
        result.success = false;
        return result;
    }

    // Walk backwards from goal to start by greedily choosing the predecessor
    // that achieves g(u) = g(pred) + w(pred,u). Ties broken by lowest id for determinism.
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    uint64_t cur = goalState_;
    statePath.push_back(cur);
    double totalCost = 0.0;
    double minClearance = clearance_.count(cur) ? clearance_.at(cur) : INF;
    double cumulativeReliability = 1.0;

    int guard = 0;
    while (cur != startState_) {
        if (++guard > (int)states_.size() + 5) { result.success = false; return result; } // safety guard vs. cycles

        auto it = incoming_.find(cur);
        bool found = false;
        double bestW = INF;
        uint64_t bestPred = 0, bestTid = 0;
        if (it != incoming_.end()) {
            for (uint64_t tid : it->second) {
                const Transition& t = transitionsById_.at(tid);
                double w = edgeWeight(t);
                if (w == INF) continue;
                if (std::abs((gOf(t.from) + w) - gOf(cur)) < 1e-9) {
                    if (w < bestW || (w == bestW && tid < bestTid) || !found) {
                        bestW = w; bestPred = t.from; bestTid = tid; found = true;
                    }
                }
            }
        }
        if (!found) { result.success = false; return result; }

        const Transition& chosen = transitionsById_.at(bestTid);
        totalCost += chosen.cost;
        cumulativeReliability *= chosen.reliability;
        double predClearance = clearance_.count(bestPred) ? clearance_.at(bestPred) : INF;
        minClearance = std::min(minClearance, predClearance);

        statePath.push_back(bestPred);
        transitionPath.push_back(bestTid);
        cur = bestPred;
    }

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    result.success = true;
    result.statePath = statePath;
    result.transitionPath = transitionPath;
    result.totalCost = totalCost;
    result.safetyScore = minClearance;
    result.cumulativeReliability = cumulativeReliability;
    return result;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    auto t0 = std::chrono::high_resolution_clock::now();
    initialize(problem);
    computeShortestPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return extractPath(ms, false);
}

PlanningResult LPAStarPlanner::replan() {
    auto t0 = std::chrono::high_resolution_clock::now();
    computeShortestPath();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return extractPath(ms, true);
}

// --- Dynamic environment API -------------------------------------------------

void LPAStarPlanner::setGoal(uint64_t newGoalState) {
    goalState_ = newGoalState;
    // With HEURISTIC_WEIGHT == 0 the key ordering does not depend on the
    // goal at all, so no re-keying is required: every previously computed
    // g()/rhs() value stays valid and computeShortestPath() simply keeps
    // expanding (or, if the new goal was already settled, does nothing).
    // If a heuristic is enabled we must re-key every queued vertex because
    // h(s) changed for all of them.
    if (HEURISTIC_WEIGHT > 0.0) {
        std::vector<uint64_t> queued;
        queued.reserve(inQueueKey_.size());
        for (auto& kv : inQueueKey_) queued.push_back(kv.first);
        for (uint64_t s : queued) {
            openSet_.erase(QueueEntry{inQueueKey_.at(s), s});
            inQueueKey_.erase(s);
        }
        for (uint64_t s : queued) {
            if (gOf(s) != rhsOf(s)) {
                Key k = calculateKey(s);
                openSet_.insert(QueueEntry{k, s});
                inQueueKey_[s] = k;
            }
        }
    }
}

void LPAStarPlanner::setTransitionAvailability(uint64_t transitionId, bool available) {
    auto it = transitionsById_.find(transitionId);
    if (it == transitionsById_.end()) return;
    it->second.available = available;
    updateVertex(it->second.to);
}

void LPAStarPlanner::addTransition(const Transition& t) {
    transitionsById_[t.id] = t;
    outgoing_[t.from].push_back(t.id);
    incoming_[t.to].push_back(t.id);
    updateVertex(t.to);
}

void LPAStarPlanner::removeTransition(uint64_t transitionId) {
    auto it = transitionsById_.find(transitionId);
    if (it == transitionsById_.end()) return;
    uint64_t to = it->second.to;
    uint64_t from = it->second.from;

    auto& outs = outgoing_[from];
    outs.erase(std::remove(outs.begin(), outs.end(), transitionId), outs.end());
    auto& ins = incoming_[to];
    ins.erase(std::remove(ins.begin(), ins.end(), transitionId), ins.end());
    transitionsById_.erase(it);

    updateVertex(to);
}

void LPAStarPlanner::setBadStates(const std::vector<uint64_t>& badStates) {
    badStates_ = std::set<uint64_t>(badStates.begin(), badStates.end());
    recomputeClearances();
    // Every vertex's incoming edge weights may have changed (safety penalty
    // depends on the destination's clearance), so every vertex must be
    // re-evaluated. This is the one update that is NOT cheap in the current
    // design -- see the report's discussion of this limitation and the
    // "affected region only" optimization proposed as a bonus extension.
    for (auto& kv : states_) updateVertex(kv.first);
}

// --- Bonus: time-dependent transition availability --------------------------

void LPAStarPlanner::setCurrentTime(double newTime) {
    currentTime_ = newTime;
    // Only transitions whose window actually straddles a boundary can have
    // changed effective availability; re-check every transition's
    // destination cheaply (window comparisons are O(1)) rather than
    // maintaining a separate time-indexed structure. For graphs where most
    // transitions are always-available (the default window), this is close
    // to free in practice since edgeWeight() is only re-derived, not searched.
    for (auto& kv : transitionsById_) {
        updateVertex(kv.second.to);
    }
}

// --- Bonus: multi-goal planning ----------------------------------------------

PlanningResult LPAStarPlanner::planMultiGoal(const PlanningProblem& problem, const std::vector<uint64_t>& goalCandidates) {
    PlanningProblem augmented = problem;

    augmented.states.push_back(State{ META_GOAL_ID, {} });

    uint64_t nextTid = 1;
    for (const auto& t : augmented.transitions) nextTid = std::max(nextTid, t.id + 1);
    for (uint64_t g : goalCandidates) {
        augmented.transitions.push_back(Transition{ nextTid++, g, META_GOAL_ID, 0.0, 1.0, 1.0, true });
    }
    augmented.goalState = META_GOAL_ID;

    PlanningResult result = plan(augmented);
    if (result.success && !result.statePath.empty() && result.statePath.back() == META_GOAL_ID) {
        result.statePath.pop_back();
        if (!result.transitionPath.empty()) result.transitionPath.pop_back();
    }
    return result;
}
