#include "d_star_lite_planner.h"
#include "safety_utils.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

// ═════════════════════════════════════════════════════════════════════════
// Construction
// ═════════════════════════════════════════════════════════════════════════

DStarLitePlanner::DStarLitePlanner()
    : km_(0.0), beta_(1.0), gamma_(0.5), delta_(0.3), exploredCount_(0) {}

// ═════════════════════════════════════════════════════════════════════════
// Weight Configuration
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::setWeights(double beta, double gamma, double delta) {
    beta_  = beta;
    gamma_ = gamma;
    delta_ = delta;
}

// ═════════════════════════════════════════════════════════════════════════
// Graph Building
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::buildAdjacencyLists() {
    adjForward_.clear();
    adjReverse_.clear();

    for (size_t i = 0; i < problem_.transitions.size(); ++i) {
        const auto& t = problem_.transitions[i];
        adjForward_[t.from].push_back(i);
        adjReverse_[t.to].push_back(i);
    }
}

void DStarLitePlanner::buildStateLookup() {
    stateLookup_.clear();
    for (const auto& s : problem_.states) {
        stateLookup_[s.id] = &s;
    }
}

// ═════════════════════════════════════════════════════════════════════════
// Open-Set Helpers (ordered set acting as priority queue)
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::insertOpen(uint64_t s, KeyPair key) {
    removeOpen(s);  // Ensure no duplicates
    openSet_.insert({key, s});
    openKeys_[s] = key;
}

void DStarLitePlanner::removeOpen(uint64_t s) {
    auto it = openKeys_.find(s);
    if (it != openKeys_.end()) {
        openSet_.erase({it->second, s});
        openKeys_.erase(it);
    }
}

bool DStarLitePlanner::isInOpen(uint64_t s) const {
    return openKeys_.find(s) != openKeys_.end();
}

KeyPair DStarLitePlanner::topKey() const {
    if (openSet_.empty()) return {INF, INF};
    return openSet_.begin()->first;
}

uint64_t DStarLitePlanner::topState() const {
    if (openSet_.empty()) return UINT64_MAX;
    return openSet_.begin()->second;
}

void DStarLitePlanner::popTop() {
    if (openSet_.empty()) return;
    auto it = openSet_.begin();
    uint64_t s = it->second;
    openSet_.erase(it);
    openKeys_.erase(s);
}

// ═════════════════════════════════════════════════════════════════════════
// G / RHS Accessors
// ═════════════════════════════════════════════════════════════════════════

double DStarLitePlanner::getG(uint64_t s) const {
    auto it = g_.find(s);
    return (it != g_.end()) ? it->second : INF;
}

double DStarLitePlanner::getRhs(uint64_t s) const {
    auto it = rhs_.find(s);
    return (it != rhs_.end()) ? it->second : INF;
}

void DStarLitePlanner::setG(uint64_t s, double val) {
    g_[s] = val;
}

void DStarLitePlanner::setRhs(uint64_t s, double val) {
    rhs_[s] = val;
}

// ═════════════════════════════════════════════════════════════════════════
// Heuristic: Euclidean distance in R^d
// ═════════════════════════════════════════════════════════════════════════

double DStarLitePlanner::heuristic(uint64_t a, uint64_t b) const {
    auto itA = stateLookup_.find(a);
    auto itB = stateLookup_.find(b);
    if (itA == stateLookup_.end() || itB == stateLookup_.end()) return 0.0;
    // Scale by beta_ to maintain admissibility with composite cost function.
    // Effective cost = beta*rawCost + penalties, so minimum cost per unit
    // distance is beta * euclideanDist. Without scaling, the heuristic
    // overestimates when beta < 1.0, causing suboptimal paths.
    return beta_ * safety::euclideanDistance(itA->second->embedding, itB->second->embedding);
}

// ═════════════════════════════════════════════════════════════════════════
// Composite Cost
// ═════════════════════════════════════════════════════════════════════════

double DStarLitePlanner::effectiveCost(const Transition& t) const {
    // Unavailable transitions have infinite cost
    if (!t.available) return INF;

    // Bad states are strictly forbidden
    if (badStateSet_.count(t.to)) return INF;

    // Safety penalty: closer to bad states = higher cost
    // Uses 1/distance so nearby bad states create large penalties
    double safetyPenalty = 0.0;
    auto it = safetyMap_.find(t.to);
    if (it != safetyMap_.end() && it->second != INF && it->second > 1e-9) {
        safetyPenalty = 1.0 / it->second;  // Close = high penalty
    }

    // Reliability penalty: less reliable = higher cost
    double reliabilityPenalty = 1.0 - t.reliability;  // 0 for perfect, 1 for zero reliability

    // effective = beta*cost + gamma*(1/safetyDist) + delta*(1 - reliability)
    // All terms are ADDITIVE penalties, so no clamping issues
    double raw = beta_ * t.cost + gamma_ * safetyPenalty + delta_ * reliabilityPenalty;
    return std::max(raw, 1e-6);
}

// ═════════════════════════════════════════════════════════════════════════
// Predecessor / Successor Queries
// ═════════════════════════════════════════════════════════════════════════

std::vector<uint64_t> DStarLitePlanner::getPredecessors(uint64_t s) const {
    // Predecessors of s = states u such that transition (u → s) exists
    std::vector<uint64_t> preds;
    auto it = adjReverse_.find(s);
    if (it != adjReverse_.end()) {
        for (size_t idx : it->second) {
            const auto& t = problem_.transitions[idx];
            if (t.available && !badStateSet_.count(t.from)) {
                preds.push_back(t.from);
            }
        }
    }
    return preds;
}

std::vector<uint64_t> DStarLitePlanner::getSuccessors(uint64_t s) const {
    // Successors of s = states v such that transition (s → v) exists
    std::vector<uint64_t> succs;
    auto it = adjForward_.find(s);
    if (it != adjForward_.end()) {
        for (size_t idx : it->second) {
            const auto& t = problem_.transitions[idx];
            if (t.available && !badStateSet_.count(t.to)) {
                succs.push_back(t.to);
            }
        }
    }
    return succs;
}

const Transition* DStarLitePlanner::findTransition(uint64_t from, uint64_t to) const {
    auto it = adjForward_.find(from);
    if (it != adjForward_.end()) {
        for (size_t idx : it->second) {
            const auto& t = problem_.transitions[idx];
            if (t.to == to && t.available) {
                return &t;
            }
        }
    }
    return nullptr;
}

// ═════════════════════════════════════════════════════════════════════════
// D* Lite: Calculate Key
// ═════════════════════════════════════════════════════════════════════════

KeyPair DStarLitePlanner::calculateKey(uint64_t s) const {
    double gVal   = getG(s);
    double rhsVal = getRhs(s);
    double minGRhs = std::min(gVal, rhsVal);
    return {
        minGRhs + heuristic(problem_.initialState, s) + km_,
        minGRhs
    };
}

// ═════════════════════════════════════════════════════════════════════════
// D* Lite: Initialize
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::initialize() {
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    openKeys_.clear();
    km_ = 0.0;
    exploredCount_ = 0;

    // All states start with g = rhs = infinity
    for (const auto& s : problem_.states) {
        setG(s.id, INF);
        setRhs(s.id, INF);
    }

    // Goal state: rhs = 0
    setRhs(problem_.goalState, 0.0);
    insertOpen(problem_.goalState, calculateKey(problem_.goalState));
}

// ═════════════════════════════════════════════════════════════════════════
// D* Lite: Update Vertex
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (u != problem_.goalState) {
        // rhs(u) = min over successors v of { c(u,v) + g(v) }
        double minRhs = INF;
        auto itFwd = adjForward_.find(u);
        if (itFwd != adjForward_.end()) {
            for (size_t idx : itFwd->second) {
                const auto& t = problem_.transitions[idx];
                double ec = effectiveCost(t);
                if (ec < INF) {
                    double candidate = ec + getG(t.to);
                    if (candidate < minRhs) {
                        minRhs = candidate;
                    }
                }
            }
        }
        setRhs(u, minRhs);
    }

    // Update priority queue membership
    removeOpen(u);
    if (getG(u) != getRhs(u)) {
        insertOpen(u, calculateKey(u));
    }
}

// ═════════════════════════════════════════════════════════════════════════
// D* Lite: Compute Shortest Path
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::computeShortestPath() {
    KeyPair startKey = calculateKey(problem_.initialState);

    while (!openSet_.empty() &&
           (topKey() < startKey ||
            getRhs(problem_.initialState) != getG(problem_.initialState))) {

        uint64_t u   = topState();
        KeyPair kOld = topKey();
        KeyPair kNew = calculateKey(u);

        if (kOld < kNew) {
            // Reinsert with updated key
            removeOpen(u);
            insertOpen(u, kNew);
        } else if (getG(u) > getRhs(u)) {
            // Overconsistent — make consistent
            setG(u, getRhs(u));
            removeOpen(u);
            exploredCount_++;

            // Update all predecessors
            for (uint64_t pred : getPredecessors(u)) {
                updateVertex(pred);
            }
        } else {
            // Underconsistent — reset g to infinity
            setG(u, INF);
            exploredCount_++;

            // Update u and all predecessors
            updateVertex(u);
            for (uint64_t pred : getPredecessors(u)) {
                updateVertex(pred);
            }
        }

        // Recompute start key for termination check
        startKey = calculateKey(problem_.initialState);
    }
}

// ═════════════════════════════════════════════════════════════════════════
// Safety Map
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::recomputeSafetyMap() {
    safetyMap_ = safety::computeSafetyMap(problem_.states, problem_.badStates);
}

// ═════════════════════════════════════════════════════════════════════════
// Path Extraction
// ═════════════════════════════════════════════════════════════════════════

PlanningResult DStarLitePlanner::extractPath() const {
    PlanningResult result;
    result.success   = false;
    result.totalCost = 0.0;
    result.safetyScore = INF;

    // Check if a path exists
    if (getG(problem_.initialState) >= INF) {
        return result;
    }

    result.success = true;
    uint64_t current = problem_.initialState;
    result.statePath.push_back(current);

    // Greedy forward trace: at each state, pick successor minimizing c(u,v)+g(v)
    size_t maxSteps = problem_.states.size() + 1;  // Prevent infinite loops
    while (current != problem_.goalState && maxSteps-- > 0) {
        double bestCost = INF;
        uint64_t bestNext = UINT64_MAX;
        const Transition* bestTrans = nullptr;

        auto itFwd = adjForward_.find(current);
        if (itFwd != adjForward_.end()) {
            for (size_t idx : itFwd->second) {
                const auto& t = problem_.transitions[idx];
                double ec = effectiveCost(t);
                if (ec < INF) {
                    double total = ec + getG(t.to);
                    if (total < bestCost) {
                        bestCost  = total;
                        bestNext  = t.to;
                        bestTrans = &t;
                    }
                }
            }
        }

        if (bestNext == UINT64_MAX) {
            result.success = false;
            break;
        }

        result.statePath.push_back(bestNext);
        result.transitionPath.push_back(bestTrans->id);
        result.totalCost += bestTrans->cost;  // Raw cost, not composite

        // Track minimum safety distance along path
        auto safetyIt = safetyMap_.find(bestNext);
        if (safetyIt != safetyMap_.end()) {
            result.safetyScore = std::min(result.safetyScore, safetyIt->second);
        }

        current = bestNext;
    }

    if (current != problem_.goalState) {
        result.success = false;
    }

    // Also include initial state safety
    if (result.success) {
        auto safetyIt = safetyMap_.find(problem_.initialState);
        if (safetyIt != safetyMap_.end()) {
            result.safetyScore = std::min(result.safetyScore, safetyIt->second);
        }
    }

    // If safety score is still infinity (no bad states), set to a large value
    if (result.safetyScore == INF) {
        result.safetyScore = std::numeric_limits<double>::max();
    }

    return result;
}

// ═════════════════════════════════════════════════════════════════════════
// Public: plan()
// ═════════════════════════════════════════════════════════════════════════

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    problem_ = problem;

    // Build bad state set
    badStateSet_.clear();
    for (uint64_t b : problem_.badStates) {
        badStateSet_.insert(b);
    }

    // Build data structures
    buildStateLookup();
    buildAdjacencyLists();
    recomputeSafetyMap();

    // D* Lite initialization and search
    initialize();
    computeShortestPath();

    return extractPath();
}

// ═════════════════════════════════════════════════════════════════════════
// Incremental Updates
// ═════════════════════════════════════════════════════════════════════════

void DStarLitePlanner::updateGoal(uint64_t newGoal) {
    // Changing the goal is a fundamental change to the backward search root.
    // The cleanest and most correct approach is to re-initialize the entire
    // D* Lite search from the new goal. This is still efficient because
    // we reuse existing data structures (adjacency lists, safety map, etc.)
    // and only reset the g/rhs search state.
    problem_.goalState = newGoal;

    // Full re-initialization of D* Lite search state
    initialize();
    computeShortestPath();
}

void DStarLitePlanner::addBadState(uint64_t stateId) {
    problem_.badStates.push_back(stateId);
    badStateSet_.insert(stateId);

    // Recompute safety map
    recomputeSafetyMap();

    // Invalidate the bad state itself
    setG(stateId, INF);
    setRhs(stateId, INF);
    removeOpen(stateId);

    // Update all states — safety distances changed globally and
    // paths through the new bad state are now forbidden
    for (const auto& s : problem_.states) {
        if (!badStateSet_.count(s.id)) {
            updateVertex(s.id);
        }
    }

    computeShortestPath();
}

void DStarLitePlanner::removeBadState(uint64_t stateId) {
    badStateSet_.erase(stateId);
    auto it = std::find(problem_.badStates.begin(), problem_.badStates.end(), stateId);
    if (it != problem_.badStates.end()) {
        problem_.badStates.erase(it);
    }

    // Recompute safety map
    recomputeSafetyMap();

    // Update all states since safety distances changed and
    // paths through the removed bad state are now allowed
    for (const auto& s : problem_.states) {
        updateVertex(s.id);
    }

    computeShortestPath();
}

void DStarLitePlanner::setTransitionAvailability(uint64_t transId, bool available) {
    for (auto& t : problem_.transitions) {
        if (t.id == transId) {
            t.available = available;
            break;
        }
    }

    // Re-initialize the search from scratch after edge changes.
    // This is necessary because D* Lite's incremental update may not
    // correctly propagate cost increases through graph regions that
    // were never explored in the initial search (those nodes have g=INF,
    // making it impossible to discover alternative paths incrementally).
    // Re-initialization reuses adjacency lists and safety map.
    initialize();
    computeShortestPath();
}

void DStarLitePlanner::addTransition(const Transition& t) {
    size_t newIdx = problem_.transitions.size();
    problem_.transitions.push_back(t);

    // Update adjacency lists
    adjForward_[t.from].push_back(newIdx);
    adjReverse_[t.to].push_back(newIdx);

    // The source state may now have a better path
    updateVertex(t.from);

    computeShortestPath();
}

PlanningResult DStarLitePlanner::replan() {
    computeShortestPath();
    return extractPath();
}
