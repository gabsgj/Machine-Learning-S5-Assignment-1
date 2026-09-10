#include "planner.h"

#include <algorithm>
#include <chrono>

// =====================================================================
// Geometry helpers
// =====================================================================

double DStarLitePlanner::euclidean(const std::vector<double>& a,
                                   const std::vector<double>& b) {
    double sum = 0;
    size_t dim = std::min(a.size(), b.size());
    for (size_t i = 0; i < dim; ++i) {
        double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double DStarLitePlanner::heuristic(uint64_t a, uint64_t b) const {
    auto ia = stateMap_.find(a);
    auto ib = stateMap_.find(b);
    if (ia == stateMap_.end() || ib == stateMap_.end()) return 0;
    return euclidean(ia->second.embedding, ib->second.embedding);
}

double DStarLitePlanner::minDistToBad(uint64_t sid) const {
    if (badPos_.empty()) return INF_COST;
    auto it = stateMap_.find(sid);
    if (it == stateMap_.end()) return INF_COST;
    double best = INF_COST;
    for (const auto& bp : badPos_)
        best = std::min(best, euclidean(it->second.embedding, bp));
    return best;
}

// =====================================================================
// Cost model
// =====================================================================

double DStarLitePlanner::effectiveCost(const Transition& t) const {
    if (!t.available)           return INF_COST;
    if (badSet_.count(t.to))    return INF_COST;

    double base = BETA * t.cost;

    double dist = minDistToBad(t.to);
    double safetyPen = (dist < SAFETY_THRESHOLD)
                           ? GAMMA * (SAFETY_THRESHOLD - dist)
                           : 0.0;

    double relPen = DELTA * (1.0 - t.reliability);

    return base + safetyPen + relPen;
}

// =====================================================================
// Graph construction
// =====================================================================

void DStarLitePlanner::buildGraph() {
    stateMap_.clear();
    succAdj_.clear();
    predAdj_.clear();
    badSet_.clear();
    badPos_.clear();
    transMap_.clear();

    for (auto& s : problem_.states) stateMap_[s.id] = s;

    for (auto b : problem_.badStates) {
        badSet_.insert(b);
        auto it = stateMap_.find(b);
        if (it != stateMap_.end()) badPos_.push_back(it->second.embedding);
    }

    // Copy transitions into the stable map, then build adjacency from pointers
    for (auto& t : problem_.transitions) transMap_[t.id] = t;

    for (auto& [id, t] : transMap_) {
        succAdj_[t.from].push_back(&t);
        predAdj_[t.to].push_back(&t);
    }
}

// =====================================================================
// D* Lite core
// =====================================================================

void DStarLitePlanner::removeFromOpen(uint64_t s) {
    auto it = openKeys_.find(s);
    if (it != openKeys_.end()) {
        open_.erase({it->second, s});
        openKeys_.erase(it);
    }
}

void DStarLitePlanner::insertToOpen(uint64_t s, Key k) {
    open_.insert({k, s});
    openKeys_[s] = k;
}

Key DStarLitePlanner::calcKey(uint64_t s) {
    double m = std::min(getG(s), getRhs(s));
    return {m + heuristic(problem_.initialState, s) + km_, m};
}

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (u != problem_.goalState) {
        double best = INF_COST;
        for (const auto* t : succAdj_[u]) {
            double v = effectiveCost(*t) + getG(t->to);
            best = std::min(best, v);
        }
        rhs_[u] = best;
    }
    removeFromOpen(u);
    if (getG(u) != getRhs(u)) insertToOpen(u, calcKey(u));
}

void DStarLitePlanner::initialize() {
    g_.clear();
    rhs_.clear();
    open_.clear();
    openKeys_.clear();
    km_            = 0;
    exploredCount_ = 0;

    rhs_[problem_.goalState] = 0;
    insertToOpen(problem_.goalState, calcKey(problem_.goalState));
}

bool DStarLitePlanner::computeShortestPath() {
    while (!open_.empty()) {
        auto topIt  = open_.begin();
        Key topKey  = topIt->key;
        Key startK  = calcKey(problem_.initialState);

        bool startConsistent =
            (getRhs(problem_.initialState) == getG(problem_.initialState));
        if (!(topKey < startK) && startConsistent) break;

        uint64_t u = topIt->state;
        Key kOld   = topIt->key;
        Key kNew   = calcKey(u);

        ++exploredCount_;

        if (kOld < kNew) {
            // Outdated key — reinsert with correct key
            open_.erase(topIt);
            openKeys_.erase(u);
            insertToOpen(u, kNew);
        } else if (getG(u) > getRhs(u)) {
            // Over-consistent: make consistent
            g_[u] = getRhs(u);
            open_.erase(topIt);
            openKeys_.erase(u);
            for (const auto* t : predAdj_[u]) updateVertex(t->from);
        } else {
            // Under-consistent
            g_[u] = INF_COST;
            open_.erase(topIt);
            openKeys_.erase(u);
            updateVertex(u);
            for (const auto* t : predAdj_[u]) updateVertex(t->from);
        }
    }
    return getG(problem_.initialState) < INF_COST;
}

// =====================================================================
// Path extraction & result building
// =====================================================================

PlanningResult DStarLitePlanner::extractResult() {
    PlanningResult r{};
    r.exploredStates   = exploredCount_;
    r.totalCost        = 0;
    r.totalReliability = 0;
    r.safetyScore      = INF_COST;

    if (getG(problem_.initialState) >= INF_COST) {
        r.success = false;
        return r;
    }

    r.success = true;
    uint64_t cur = problem_.initialState;
    r.statePath.push_back(cur);

    int budget = static_cast<int>(stateMap_.size()) + 1;
    while (cur != problem_.goalState && budget-- > 0) {
        double bestVal  = INF_COST;
        uint64_t bestTo = cur;
        const Transition* bestT = nullptr;

        for (const auto* t : succAdj_[cur]) {
            if (!t->available || badSet_.count(t->to)) continue;
            double v = effectiveCost(*t) + getG(t->to);
            if (v < bestVal) { bestVal = v; bestTo = t->to; bestT = t; }
        }

        if (bestTo == cur) { r.success = false; break; }

        r.statePath.push_back(bestTo);
        r.transitionPath.push_back(bestT->id);
        r.totalCost        += bestT->cost;
        r.totalReliability += bestT->reliability;
        r.safetyScore = std::min(r.safetyScore, minDistToBad(bestTo));

        cur = bestTo;
    }

    r.safetyScore = std::min(r.safetyScore, minDistToBad(problem_.initialState));
    if (cur != problem_.goalState) r.success = false;

    // Approximate memory footprint
    r.memoryBytes = (g_.size() + rhs_.size()) * (sizeof(uint64_t) + sizeof(double))
                  + open_.size()     * sizeof(PQEntry)
                  + stateMap_.size() * (sizeof(uint64_t) + sizeof(State))
                  + transMap_.size() * (sizeof(uint64_t) + sizeof(Transition));
    return r;
}

// =====================================================================
// Public API — initial plan
// =====================================================================

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    problem_ = problem;
    auto t0  = std::chrono::high_resolution_clock::now();

    buildGraph();

    if (badSet_.count(problem_.initialState) ||
        badSet_.count(problem_.goalState)) {
        PlanningResult r{};
        r.success = false;
        return r;
    }

    initialize();
    computeShortestPath();

    PlanningResult r = extractResult();
    auto t1 = std::chrono::high_resolution_clock::now();
    r.planningTimeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

// =====================================================================
// Bonus #1 — incremental replanning
// =====================================================================

PlanningResult DStarLitePlanner::replan(
    const std::vector<EnvironmentChange>& changes) {

    auto t0 = std::chrono::high_resolution_clock::now();
    bool needFull = false;

    for (const auto& ch : changes) {
        switch (ch.type) {

        case ChangeType::TRANSITION_UNAVAILABLE: {
            auto it = transMap_.find(ch.transitionId);
            if (it != transMap_.end()) {
                it->second.available = false;
                updateVertex(it->second.from);
            }
            break;
        }
        case ChangeType::TRANSITION_AVAILABLE: {
            auto it = transMap_.find(ch.transitionId);
            if (it != transMap_.end()) {
                it->second.available = true;
                updateVertex(it->second.from);
            }
            break;
        }
        case ChangeType::TRANSITION_ADDED: {
            Transition t = ch.newTransition;
            transMap_[t.id] = t;
            // Rebuild adjacency for affected nodes
            succAdj_[t.from].push_back(&transMap_[t.id]);
            predAdj_[t.to].push_back(&transMap_[t.id]);
            updateVertex(t.from);
            break;
        }
        case ChangeType::TRANSITION_REMOVED: {
            auto it = transMap_.find(ch.transitionId);
            if (it != transMap_.end()) {
                uint64_t src = it->second.from;
                uint64_t dst = it->second.to;
                // Remove from adjacency
                auto& sv = succAdj_[src];
                sv.erase(std::remove_if(sv.begin(), sv.end(),
                             [&](const Transition* p) { return p->id == ch.transitionId; }),
                         sv.end());
                auto& pv = predAdj_[dst];
                pv.erase(std::remove_if(pv.begin(), pv.end(),
                             [&](const Transition* p) { return p->id == ch.transitionId; }),
                         pv.end());
                transMap_.erase(it);
                updateVertex(src);
            }
            break;
        }
        case ChangeType::BAD_STATE_ADDED: {
            badSet_.insert(ch.stateId);
            auto sit = stateMap_.find(ch.stateId);
            if (sit != stateMap_.end()) badPos_.push_back(sit->second.embedding);
            // Every edge leading TO this state now has infinite cost
            for (const auto* t : predAdj_[ch.stateId]) updateVertex(t->from);
            // Edges FROM neighbouring states may also have changed safety penalty
            for (auto& [sid, _] : stateMap_) {
                for (const auto* t : succAdj_[sid]) {
                    if (minDistToBad(t->to) < SAFETY_THRESHOLD)
                        updateVertex(sid);
                }
            }
            break;
        }
        case ChangeType::BAD_STATE_REMOVED: {
            badSet_.erase(ch.stateId);
            // Rebuild badPos_
            badPos_.clear();
            for (auto b : badSet_) {
                auto sit = stateMap_.find(b);
                if (sit != stateMap_.end())
                    badPos_.push_back(sit->second.embedding);
            }
            // Edges to the formerly-bad state are now viable
            for (const auto* t : predAdj_[ch.stateId]) updateVertex(t->from);
            break;
        }
        case ChangeType::GOAL_CHANGED:
            problem_.goalState = ch.stateId;
            needFull = true;
            break;
        }
    }

    if (needFull) {
        // Goal change requires full re-initialisation in D* Lite
        initialize();
    }

    km_ += 1.0; // increment to keep keys monotone
    computeShortestPath();

    PlanningResult r = extractResult();
    auto t1 = std::chrono::high_resolution_clock::now();
    r.planningTimeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return r;
}

// =====================================================================
// Bonus #2 — multi-goal planning
// =====================================================================

PlanningResult DStarLitePlanner::planMultiGoal(
    PlanningProblem problem, const std::vector<uint64_t>& goals) {

    PlanningResult best{};
    best.success   = false;
    best.totalCost = INF_COST;

    for (uint64_t g : goals) {
        problem.goalState = g;
        PlanningResult r = plan(problem);
        if (!r.success) continue;

        // Score: higher is better  →  pick lowest cost among successes
        double score = ALPHA - BETA * r.totalCost
                     + GAMMA * r.safetyScore
                     + DELTA * r.totalReliability;
        double bestScore = best.success
            ? (ALPHA - BETA * best.totalCost
               + GAMMA * best.safetyScore
               + DELTA * best.totalReliability)
            : -INF_COST;

        if (score > bestScore) best = r;
    }
    return best;
}
