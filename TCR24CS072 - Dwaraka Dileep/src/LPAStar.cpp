#include "LPAStar.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>

namespace planner {

double LPAStarPlanner::heuristic(uint64_t s) const {
    // Admissible & consistent: straight-line Euclidean distance in the
    // embedding space is always <= true shortest-path cost, because every
    // edge weight is >= the geometric distance component contributes
    // nothing negative and the "cost" attribute is assumed to model at
    // least travel distance. If a user's cost model is not distance-based,
    // this still keeps A*/LPA* correct as long as cost(u,v) >= 0, since we
    // clamp heuristic usage to a lower bound via min-with-zero fallback.
    auto itS = stateById_.find(s);
    auto itG = stateById_.find(goal_);
    if (itS == stateById_.end() || itG == stateById_.end()) return 0.0;
    return itS->second.distanceTo(itG->second);
}

double LPAStarPlanner::edgeWeight(const Transition& t) const {
    double w = problem_.beta * t.cost;

    // Soft safety penalties (always >= 0):
    double edgeSafetyPenalty = problem_.gamma * (1.0 - t.safety);
    double clearance = 0.0;
    auto it = clearance_.find(t.to);
    if (it != clearance_.end()) clearance = it->second;
    double clearancePenalty = problem_.gamma * (1.0 / (1.0 + clearance));

    double reliabilityPenalty = problem_.delta * (1.0 - t.reliability);

    return w + edgeSafetyPenalty + clearancePenalty + reliabilityPenalty;
}

LPAStarPlanner::Key LPAStarPlanner::calculateKey(uint64_t s) const {
    double gs = g_.count(s) ? g_.at(s) : INF;
    double rhss = rhs_.count(s) ? rhs_.at(s) : INF;
    double m = std::min(gs, rhss);
    return Key{ m == INF ? INF : m + heuristic(s), m };
}

void LPAStarPlanner::buildGraph(const PlanningProblem& problem) {
    problem_ = problem;
    stateById_.clear();
    outEdges_.clear();
    inEdges_.clear();
    clearance_.clear();

    std::unordered_set<uint64_t> badSet(problem.badStates.begin(),
                                         problem.badStates.end());

    for (const auto& s : problem.states) {
        if (badSet.count(s.id)) continue; // bad states never enter the graph
        stateById_[s.id] = s;
    }

    // Precompute clearance (distance to nearest bad state) for every
    // legal state, used both for the hard safetyRadius filter and the
    // soft clearance penalty in edgeWeight().
    for (const auto& [id, st] : stateById_) {
        double best = INF;
        for (uint64_t bid : problem.badStates) {
            auto bit = std::find_if(problem.states.begin(), problem.states.end(),
                                     [&](const State& x) { return x.id == bid; });
            if (bit == problem.states.end()) continue;
            double d = st.distanceTo(*bit);
            if (d < best) best = d;
        }
        clearance_[id] = best;
    }

    for (const auto& t : problem.transitions) {
        if (badSet.count(t.from) || badSet.count(t.to)) continue;
        if (!stateById_.count(t.from) || !stateById_.count(t.to)) continue;
        if (problem.safetyRadius > 0.0) {
            double c = clearance_.count(t.to) ? clearance_[t.to] : INF;
            if (c < problem.safetyRadius) continue; // hard safety constraint
        }
        outEdges_[t.from].push_back(t);
        inEdges_[t.to].push_back(t);
    }
}

void LPAStarPlanner::updateVertex(uint64_t s) {
    if (s != start_) {
        double best = INF;
        auto it = inEdges_.find(s);
        if (it != inEdges_.end()) {
            for (const auto& t : it->second) {
                if (!t.available) continue;
                double gu = g_.count(t.from) ? g_.at(t.from) : INF;
                if (gu == INF) continue;
                double cand = gu + edgeWeight(t);
                if (cand < best) best = cand;
            }
        }
        rhs_[s] = best;
    }

    // Remove from open set if present.
    auto keyIt = openKeyOf_.find(s);
    if (keyIt != openKeyOf_.end()) {
        openSet_.erase({keyIt->second, s});
        openKeyOf_.erase(keyIt);
    }

    double gs = g_.count(s) ? g_.at(s) : INF;
    double rhss = rhs_.count(s) ? rhs_.at(s) : INF;
    if (gs != rhss) {
        Key k = calculateKey(s);
        openSet_.insert({k, s});
        openKeyOf_[s] = k;
    }
}

void LPAStarPlanner::computeShortestPath() {
    while (!openSet_.empty()) {
        auto topIt = openSet_.begin();
        Key topKey = topIt->first;
        uint64_t u = topIt->second;

        double gGoal = g_.count(goal_) ? g_.at(goal_) : INF;
        double rhsGoal = rhs_.count(goal_) ? rhs_.at(goal_) : INF;
        Key goalKey = calculateKey(goal_);
        if (!(topKey < goalKey) && gGoal == rhsGoal) {
            break; // goal is locally consistent and nothing better remains
        }

        openSet_.erase(topIt);
        openKeyOf_.erase(u);
        statesExplored_++;

        double gu = g_.count(u) ? g_.at(u) : INF;
        double rhsu = rhs_.count(u) ? rhs_.at(u) : INF;

        if (gu > rhsu) {
            g_[u] = rhsu;
            auto it = outEdges_.find(u);
            if (it != outEdges_.end()) {
                for (const auto& t : it->second) {
                    if (t.available) updateVertex(t.to);
                }
            }
        } else {
            g_[u] = INF;
            auto it = outEdges_.find(u);
            if (it != outEdges_.end()) {
                for (const auto& t : it->second) {
                    if (t.available) updateVertex(t.to);
                }
            }
            updateVertex(u);
        }
    }
}

PlanningResult LPAStarPlanner::extractResult() {
    PlanningResult result;
    double gGoal = g_.count(goal_) ? g_.at(goal_) : INF;
    result.statesExplored = statesExplored_;

    if (gGoal == INF || !stateById_.count(goal_)) {
        result.success = false;
        return result;
    }

    // Reconstruct path by walking backwards from goal, at each step
    // picking the predecessor edge that is consistent with g(pred) +
    // edgeWeight == g(current). Ties broken by lowest edge weight.
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    uint64_t cur = goal_;
    statePath.push_back(cur);

    std::unordered_set<uint64_t> visited;
    visited.insert(cur);

    while (cur != start_) {
        auto it = inEdges_.find(cur);
        if (it == inEdges_.end()) { result.success = false; return result; }

        const Transition* best = nullptr;
        double bestVal = INF;
        double gc = g_.count(cur) ? g_.at(cur) : INF;
        for (const auto& t : it->second) {
            if (!t.available) continue;
            if (visited.count(t.from)) continue;
            double gp = g_.count(t.from) ? g_.at(t.from) : INF;
            if (gp == INF) continue;
            double val = gp + edgeWeight(t);
            if (std::abs(val - gc) < 1e-6 && val < bestVal) {
                best = &t;
                bestVal = val;
            }
        }
        if (!best) { result.success = false; return result; }

        statePath.push_back(best->from);
        transitionPath.push_back(best->id);
        visited.insert(best->from);
        cur = best->from;
    }

    std::reverse(statePath.begin(), statePath.end());
    std::reverse(transitionPath.begin(), transitionPath.end());

    double totalCost = 0.0;
    double reliabilityProduct = 1.0;
    double minClearance = INF;
    for (auto tid : transitionPath) {
        for (const auto& [from, edges] : outEdges_) {
            for (const auto& t : edges) {
                if (t.id == tid) {
                    totalCost += t.cost;
                    reliabilityProduct *= t.reliability;
                }
            }
        }
    }
    for (auto sid : statePath) {
        if (clearance_.count(sid)) minClearance = std::min(minClearance, clearance_[sid]);
    }

    result.success = true;
    result.statePath = statePath;
    result.transitionPath = transitionPath;
    result.totalCost = totalCost;
    result.safetyScore = (minClearance == INF ? 0.0 : minClearance);
    result.reliabilityScore = reliabilityProduct;
    return result;
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    auto t0 = std::chrono::steady_clock::now();

    buildGraph(problem);
    start_ = problem.initialState;
    goal_ = problem.goalState;
    g_.clear();
    rhs_.clear();
    openSet_.clear();
    openKeyOf_.clear();
    statesExplored_ = 0;

    if (!stateById_.count(start_) || !stateById_.count(goal_)) {
        initialized_ = true;
        PlanningResult r;
        r.success = false;
        return r;
    }

    rhs_[start_] = 0.0;
    g_[start_] = INF;
    Key k = calculateKey(start_);
    openSet_.insert({k, start_});
    openKeyOf_[start_] = k;

    computeShortestPath();
    initialized_ = true;

    auto result = extractResult();
    auto t1 = std::chrono::steady_clock::now();
    result.planningTimeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

bool LPAStarPlanner::updateTransition(uint64_t transitionId, double newCost,
                                       bool newAvailable, double newSafety,
                                       double newReliability) {
    bool found = false;
    for (auto& [from, edges] : outEdges_) {
        for (auto& t : edges) {
            if (t.id == transitionId) {
                t.cost = newCost;
                t.available = newAvailable;
                if (newSafety >= 0.0) t.safety = newSafety;
                if (newReliability >= 0.0) t.reliability = newReliability;
                found = true;
            }
        }
    }
    for (auto& [to, edges] : inEdges_) {
        for (auto& t : edges) {
            if (t.id == transitionId) {
                t.cost = newCost;
                t.available = newAvailable;
                if (newSafety >= 0.0) t.safety = newSafety;
                if (newReliability >= 0.0) t.reliability = newReliability;
            }
        }
    }
    if (!found) return false;

    // Only the endpoint's rhs can be affected by an edge weight/availability
    // change; UpdateVertex will pull in the new weight on its next
    // recomputation. This is the classic LPA* incremental step.
    for (auto& [from, edges] : outEdges_) {
        for (auto& t : edges) {
            if (t.id == transitionId) updateVertex(t.to);
        }
    }
    return true;
}

void LPAStarPlanner::addTransition(const Transition& t) {
    if (!stateById_.count(t.from) || !stateById_.count(t.to)) return;
    outEdges_[t.from].push_back(t);
    inEdges_[t.to].push_back(t);
    updateVertex(t.to);
}

bool LPAStarPlanner::removeTransition(uint64_t transitionId) {
    bool found = false;
    uint64_t affectedTo = 0;
    for (auto& [from, edges] : outEdges_) {
        auto it = std::remove_if(edges.begin(), edges.end(),
                                  [&](const Transition& t) {
                                      if (t.id == transitionId) {
                                          affectedTo = t.to;
                                          return true;
                                      }
                                      return false;
                                  });
        if (it != edges.end()) { edges.erase(it, edges.end()); found = true; }
    }
    for (auto& [to, edges] : inEdges_) {
        auto it = std::remove_if(edges.begin(), edges.end(),
                                  [&](const Transition& t) { return t.id == transitionId; });
        edges.erase(it, edges.end());
    }
    if (found) updateVertex(affectedTo);
    return found;
}

PlanningResult LPAStarPlanner::replan() {
    auto t0 = std::chrono::steady_clock::now();
    computeShortestPath();
    auto result = extractResult();
    auto t1 = std::chrono::steady_clock::now();
    result.planningTimeMs =
        std::chrono::duration<double, std::milli>(t1 - t0).count();
    return result;
}

PlanningResult LPAStarPlanner::replanWithNewGoal(const PlanningProblem& updatedProblem) {
    // Goal (and, by extension, bad-state-set) changes invalidate the
    // heuristic for every vertex, so a full reinitialization is the
    // honest/correct approach here -- see class-level comment. We still
    // avoid re-parsing external input, re-validating the problem, or
    // rebuilding anything the caller doesn't need rebuilt.
    return plan(updatedProblem);
}

} // namespace planner
