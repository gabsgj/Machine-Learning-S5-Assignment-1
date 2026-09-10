#include "planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <unordered_set>

namespace safeplanner {

namespace {
constexpr double EPS = 1e-9;
}

DStarLitePlanner::DStarLitePlanner(double safetyWeight, double reliabilityWeight)
    : safetyWeight_(safetyWeight), reliabilityWeight_(reliabilityWeight) {}

bool DStarLitePlanner::Compare::operator()(const QueueEntry& a,
                                            const QueueEntry& b) const {
    if (std::fabs(a.key.k1 - b.key.k1) > EPS)
        return a.key.k1 > b.key.k1;
    if (std::fabs(a.key.k2 - b.key.k2) > EPS)
        return a.key.k2 > b.key.k2;
    return a.serial > b.serial;
}

double DStarLitePlanner::inf() const {
    return std::numeric_limits<double>::infinity();
}

bool DStarLitePlanner::validState(std::uint64_t state) const {
    return states_.find(state) != states_.end();
}

bool DStarLitePlanner::isBad(std::uint64_t state) const {
    return bad_.find(state) != bad_.end();
}

double DStarLitePlanner::heuristic(std::uint64_t a, std::uint64_t b) const {
    const auto ia = states_.find(a);
    const auto ib = states_.find(b);
    if (ia == states_.end() || ib == states_.end()) return inf();

    const auto& x = ia->second.embedding;
    const auto& y = ib->second.embedding;
    const std::size_t n = std::min(x.size(), y.size());

    double sum = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = x[i] - y[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double DStarLitePlanner::distanceToBad(std::uint64_t state) const {
    if (!validState(state) || bad_.empty()) return inf();

    double best = inf();
    for (const auto b : bad_) {
        if (validState(b))
            best = std::min(best, heuristic(state, b));
    }
    return best;
}

double DStarLitePlanner::edgeWeight(const Transition& t) const {
    if (!t.available || !validState(t.from) || !validState(t.to) ||
        isBad(t.from) || isBad(t.to)) {
        return inf();
    }

    const double distance = distanceToBad(t.to);
    const double safetyPenalty =
        std::isfinite(distance) ? safetyWeight_ / std::max(distance, EPS) : 0.0;

    const double reliability = std::clamp(t.reliability, 0.0, 1.0);
    const double reliabilityPenalty = reliabilityWeight_ * (1.0 - reliability);

    return std::max(0.0, t.cost) + safetyPenalty + reliabilityPenalty;
}

double DStarLitePlanner::g(std::uint64_t s) const {
    const auto it = g_.find(s);
    return it == g_.end() ? inf() : it->second;
}

double DStarLitePlanner::rhs(std::uint64_t s) const {
    const auto it = rhs_.find(s);
    return it == rhs_.end() ? inf() : it->second;
}

void DStarLitePlanner::setG(std::uint64_t s, double value) {
    g_[s] = value;
}

void DStarLitePlanner::setRhs(std::uint64_t s, double value) {
    rhs_[s] = value;
}

DStarLitePlanner::Key DStarLitePlanner::calculateKey(std::uint64_t s) const {
    const double best = std::min(g(s), rhs(s));
    return {best + heuristic(start_, s) + km_, best};
}

bool DStarLitePlanner::keyLess(const Key& a, const Key& b) const {
    if (a.k1 < b.k1 - EPS) return true;
    if (a.k1 > b.k1 + EPS) return false;
    return a.k2 < b.k2 - EPS;
}

void DStarLitePlanner::pushOpen(std::uint64_t s) {
    open_.push(QueueEntry{calculateKey(s), s, ++serial_});
}

void DStarLitePlanner::updateVertex(std::uint64_t u) {
    if (!validState(u)) return;

    if (u != goal_) {
        double best = inf();
        const auto outIt = outgoing_.find(u);
        if (outIt != outgoing_.end()) {
            for (const auto tid : outIt->second) {
                const auto trIt = transitions_.find(tid);
                if (trIt == transitions_.end()) continue;
                const auto& t = trIt->second;
                const double w = edgeWeight(t);
                if (std::isfinite(w))
                    best = std::min(best, w + g(t.to));
            }
        }
        setRhs(u, best);
    }

    if (std::fabs(g(u) - rhs(u)) > EPS)
        pushOpen(u);
}

void DStarLitePlanner::computeShortestPath(std::size_t& explored) {
    while (!open_.empty()) {
        const auto top = open_.top();
        const Key startKey = calculateKey(start_);

        if (!keyLess(top.key, startKey) &&
            std::fabs(rhs(start_) - g(start_)) <= EPS) {
            break;
        }

        open_.pop();

        const Key currentKey = calculateKey(top.state);
        if (keyLess(currentKey, top.key) || keyLess(top.key, currentKey)) {
            pushOpen(top.state);
            continue;
        }

        ++explored;
        const auto u = top.state;

        if (g(u) > rhs(u)) {
            setG(u, rhs(u));
            const auto inIt = incoming_.find(u);
            if (inIt != incoming_.end()) {
                for (const auto tid : inIt->second) {
                    const auto trIt = transitions_.find(tid);
                    if (trIt != transitions_.end())
                        updateVertex(trIt->second.from);
                }
            }
        } else {
            setG(u, inf());
            updateVertex(u);
            const auto inIt = incoming_.find(u);
            if (inIt != incoming_.end()) {
                for (const auto tid : inIt->second) {
                    const auto trIt = transitions_.find(tid);
                    if (trIt != transitions_.end())
                        updateVertex(trIt->second.from);
                }
            }
        }
    }
}

void DStarLitePlanner::rebuildGraph() {
    outgoing_.clear();
    incoming_.clear();

    for (const auto& [id, t] : transitions_) {
        if (!validState(t.from) || !validState(t.to)) continue;
        outgoing_[t.from].push_back(id);
        incoming_[t.to].push_back(id);
    }
}

void DStarLitePlanner::resetSearch() {
    while (!open_.empty()) open_.pop();
    g_.clear();
    rhs_.clear();
    serial_ = 0;
    km_ = 0.0;

    for (const auto& [id, _] : states_) {
        g_[id] = inf();
        rhs_[id] = inf();
    }

    if (validState(goal_) && !isBad(goal_)) {
        rhs_[goal_] = 0.0;
        pushOpen(goal_);
    }
}

PlanningResult DStarLitePlanner::buildResult(std::size_t explored,
                                             double elapsedMs,
                                             double replanningMs) const {
    PlanningResult result;
    result.exploredStates = explored;
    result.planningTimeMs = elapsedMs;
    result.replanningTimeMs = replanningMs;

    if (!validState(start_) || !validState(goal_) ||
        isBad(start_) || isBad(goal_) ||
        !std::isfinite(g(start_))) {
        return result;
    }

    std::uint64_t current = start_;
    std::unordered_set<std::uint64_t> seen;
    result.statePath.push_back(current);

    double rawCost = 0.0;
    double reliabilityProduct = 1.0;
    double minSafety = inf();

    while (current != goal_) {
        if (!seen.insert(current).second) {
            result.statePath.clear();
            result.transitionPath.clear();
            return result;
        }

        double best = inf();
        const Transition* bestTransition = nullptr;

        const auto outIt = outgoing_.find(current);
        if (outIt == outgoing_.end()) break;

        for (const auto tid : outIt->second) {
            const auto trIt = transitions_.find(tid);
            if (trIt == transitions_.end()) continue;
            const auto& t = trIt->second;
            const double w = edgeWeight(t);
            if (!std::isfinite(w) || !std::isfinite(g(t.to))) continue;

            const double value = w + g(t.to);
            if (value < best - EPS ||
                (std::fabs(value - best) <= EPS &&
                 (bestTransition == nullptr ||
                  t.reliability > bestTransition->reliability))) {
                best = value;
                bestTransition = &t;
            }
        }

        if (!bestTransition) break;

        result.transitionPath.push_back(bestTransition->id);
        result.statePath.push_back(bestTransition->to);
        rawCost += std::max(0.0, bestTransition->cost);
        reliabilityProduct *= std::clamp(bestTransition->reliability, 0.0, 1.0);
        minSafety = std::min(minSafety, distanceToBad(bestTransition->to));

        current = bestTransition->to;
        if (result.statePath.size() > states_.size() + 1) {
            result.statePath.clear();
            result.transitionPath.clear();
            return result;
        }
    }

    if (current != goal_) {
        result.statePath.clear();
        result.transitionPath.clear();
        return result;
    }

    minSafety = std::min(minSafety, distanceToBad(start_));
    if (!std::isfinite(minSafety)) minSafety = 0.0;

    result.success = true;
    result.totalCost = rawCost;
    result.safetyScore = minSafety;
    result.cumulativeReliability = reliabilityProduct;
    return result;
}

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    const auto t0 = std::chrono::steady_clock::now();

    states_.clear();
    transitions_.clear();
    bad_.clear();

    for (const auto& s : problem.states) states_[s.id] = s;
    for (const auto& t : problem.transitions) transitions_[t.id] = t;
    bad_.insert(problem.badStates.begin(), problem.badStates.end());

    start_ = problem.initialState;
    goal_ = problem.goalState;

    rebuildGraph();
    resetSearch();

    std::size_t explored = 0;
    computeShortestPath(explored);

    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    return buildResult(explored, ms);
}

void DStarLitePlanner::setGoal(std::uint64_t goal) {
    if (goal_ == goal) return;
    goal_ = goal;
    resetSearch();
}

void DStarLitePlanner::setBadStates(const std::vector<std::uint64_t>& badStates) {
    bad_.clear();
    bad_.insert(badStates.begin(), badStates.end());
    rebuildGraph();
    resetSearch();
}

void DStarLitePlanner::setTransitionAvailable(std::uint64_t transitionId,
                                               bool available) {
    const auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;

    it->second.available = available;
    const auto from = it->second.from;
    const auto to = it->second.to;

    updateVertex(from);
    updateVertex(to);
}

void DStarLitePlanner::addTransition(const Transition& transition) {
    transitions_[transition.id] = transition;
    if (validState(transition.from) && validState(transition.to)) {
        outgoing_[transition.from].push_back(transition.id);
        incoming_[transition.to].push_back(transition.id);
        updateVertex(transition.from);
    }
}

void DStarLitePlanner::removeTransition(std::uint64_t transitionId) {
    const auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;

    const auto from = it->second.from;
    const auto to = it->second.to;
    transitions_.erase(it);

    auto removeId = [transitionId](std::vector<std::uint64_t>& ids) {
        ids.erase(std::remove(ids.begin(), ids.end(), transitionId), ids.end());
    };

    removeId(outgoing_[from]);
    removeId(incoming_[to]);
    updateVertex(from);
}

PlanningResult DStarLitePlanner::replan(std::uint64_t start) {
    const auto t0 = std::chrono::steady_clock::now();

    if (start_ != start) {
        km_ += heuristic(start_, start);
        start_ = start;
    }

    std::size_t explored = 0;
    computeShortestPath(explored);

    const auto t1 = std::chrono::steady_clock::now();
    const double ms =
        std::chrono::duration<double, std::milli>(t1 - t0).count();

    return buildResult(explored, ms, ms);
}

} // namespace safeplanner
