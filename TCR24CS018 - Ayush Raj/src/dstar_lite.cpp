#include "dstar_lite.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>

#if defined(__linux__)
#include <sys/resource.h>
#endif

namespace {

double processPeakMemoryKB() {
#if defined(__linux__)
    rusage usage{};
    if (getrusage(RUSAGE_SELF, &usage) == 0) {
        return static_cast<double>(usage.ru_maxrss) * 1.024;
    }
#endif
    return 0;
}

}  // namespace

bool PriorityQueue::LaterKey::operator()(const Entry& a, const Entry& b) const {
    if (a.key.first != b.key.first) {
        return a.key.first > b.key.first;
    }
    if (a.key.second != b.key.second) {
        return a.key.second > b.key.second;
    }
    return a.state > b.state;
}
void PriorityQueue::push(uint64_t state, Key key) {
    heap_.push({state, key});
}
bool PriorityQueue::empty() const {
    return heap_.empty();
}
const PriorityQueue::Entry& PriorityQueue::top() const {
    return heap_.top();
}
void PriorityQueue::pop() {
    heap_.pop();
}
void PriorityQueue::clear() {
    heap_ = {};
}

DStarLitePlanner::DStarLitePlanner(double alpha, double beta) {
    setQualityWeights(alpha, beta);
}

void DStarLitePlanner::setQualityWeights(double alpha, double beta) {
    if (alpha < 0.0 || beta < 0.0 || std::abs(alpha + beta - 1.0) > kEpsilon) {
        throw std::invalid_argument("quality weights must be non-negative and sum to one");
    }
    alpha_ = alpha;
    beta_ = beta;
}

bool DStarLitePlanner::equal(double a, double b) {
    return std::abs(a - b) <= kEpsilon * std::max({1.0, std::abs(a), std::abs(b)});
}
bool DStarLitePlanner::lessKey(const Key& a, const Key& b) {
    return a.first < b.first - kEpsilon ||
           (equal(a.first, b.first) && a.second < b.second - kEpsilon);
}

bool DStarLitePlanner::buildGraph() {
    states_.clear();
    outgoing_.clear();
    incoming_.clear();
    bad_.clear();
    clearance_.clear();
    safety_.clear();
    continuationSafety_.clear();
    reliability_.clear();

    for (const State& state : problem_.states) {
        if (!states_.emplace(state.id, &state).second) {
            return false;
        }
    }
    if (!states_.count(start_) || !states_.count(goal_)) {
        return false;
    }

    for (uint64_t id : problem_.badStates) {
        if (!states_.count(id)) {
            return false;
        }
        bad_.insert(id);
    }

    for (const Transition& t : problem_.transitions) {
        if (!states_.count(t.from) || !states_.count(t.to) || !std::isfinite(t.cost) ||
            t.cost < 0.0 || !std::isfinite(t.reliability) || t.reliability < 0.0 ||
            t.reliability > 1.0) {
            return false;
        }
        outgoing_[t.from].push_back(&t);
        incoming_[t.to].push_back(&t);
    }

    maxClearance_ = 0.0;
    for (const auto& [id, state] : states_) {
        if (bad_.count(id)) {
            continue;
        }
        double distance = kInfinity;
        if (bad_.empty()) {
            distance = 1.0;
        }
        for (uint64_t badId : bad_) {
            const auto& a = state->embedding;
            const auto& b = states_.at(badId)->embedding;
            if (a.size() != b.size()) {
                return false;
            }
            double sum = 0.0;
            for (std::size_t i = 0; i < a.size(); ++i) {
                const double difference = a[i] - b[i];
                sum += difference * difference;
            }
            distance = std::min(distance, std::sqrt(sum));
        }
        clearance_[id] = distance;
        maxClearance_ = std::max(maxClearance_, distance);
    }
    for (const auto& [id, state] : states_) {
        if (bad_.count(id)) {
            continue;
        }
        if (bad_.empty()) {
            safety_[id] = 1.0;
        } else if (maxClearance_ > 0.0) {
            safety_[id] = clearance_[id] / maxClearance_;
        } else {
            safety_[id] = 0.0;
        }
    }
    return true;
}

void DStarLitePlanner::initialize(const PlanningProblem& problem) {
    problem_ = problem;
    buildGraph();
    g_.clear();
    rhs_.clear();
    open_.clear();
    km_ = 0.0;

    for (const auto& [id, state] : states_) {
        g_[id] = kInfinity;
        rhs_[id] = kInfinity;
        reliability_[id] = 0.0;
        continuationSafety_[id] = 0.0;
    }
    rhs_[goal_] = 0.0;
    reliability_[goal_] = 1.0;
    continuationSafety_[goal_] = safety_[goal_];
    open_.push(goal_, calculateKey(goal_));
}

void DStarLitePlanner::applyChanges(const PlanningProblem& problem) {
    const double savedKm = km_;
    const uint64_t oldStart = start_;
    const uint64_t oldGoal = goal_;
    std::unordered_set<uint64_t> affected;

    for (const Transition& oldTransition : problem_.transitions) {
        auto found = std::find_if(problem.transitions.begin(), problem.transitions.end(),
            [&oldTransition](const Transition& transition) {
                return transition.id == oldTransition.id;
            });
        if (found == problem.transitions.end() ||
            found->from != oldTransition.from || found->to != oldTransition.to ||
            found->cost != oldTransition.cost ||
            found->reliability != oldTransition.reliability ||
            found->available != oldTransition.available) {
            affected.insert(oldTransition.from);
            if (found != problem.transitions.end()) affected.insert(found->from);
        }
    }
    for (const Transition& transition : problem.transitions) {
        auto found = std::find_if(problem_.transitions.begin(), problem_.transitions.end(),
            [&transition](const Transition& oldTransition) {
                return transition.id == oldTransition.id;
            });
        if (found == problem_.transitions.end()) affected.insert(transition.from);
    }

    const bool badChanged = problem_.badStates != problem.badStates;
    problem_ = problem;
    start_ = problem_.initialState;
    goal_ = problem_.goalState;
    if (start_ != oldStart) km_ += heuristic(oldStart, start_);
    buildGraph();

    for (const auto& item : states_) {
        if (!g_.count(item.first)) {
            g_[item.first] = kInfinity;
            rhs_[item.first] = kInfinity;
            reliability_[item.first] = 0.0;
            continuationSafety_[item.first] = 0.0;
        }
    }
    if (goal_ != oldGoal) {
        rhs_[oldGoal] = kInfinity;
        rhs_[goal_] = 0.0;
        reliability_[goal_] = 1.0;
        continuationSafety_[goal_] = safety_[goal_];
        affected.insert(oldGoal);
        affected.insert(goal_);
    }
    if (badChanged) {
        for (const auto& item : states_) affected.insert(item.first);
    }
    for (uint64_t vertex : affected) {
        if (states_.count(vertex) && !bad_.count(vertex)) {
            updateVertex(vertex);
        }
    }
    if (!open_.empty() && g_[start_] >= kInfinity / 2.0) {
        initialize(problem_);
        km_ = savedKm;
    }
}

Key DStarLitePlanner::calculateKey(uint64_t u) const {
    const double base = std::min(g_.at(u), rhs_.at(u));
    const double q = calculateQuality(u);
    return {base + heuristic(start_, u) + km_, q > 0.0 ? base / q : kInfinity};
}

std::vector<const Transition*> DStarLitePlanner::successors(uint64_t u) const {
    std::vector<const Transition*> result;
    if (bad_.count(u)) {
        return result;
    }
    auto it = outgoing_.find(u);
    if (it == outgoing_.end()) {
        return result;
    }
    for (const Transition* t : it->second) {
        if (t->available && !bad_.count(t->to)) {
            result.push_back(t);
        }
    }
    return result;
}
std::vector<const Transition*> DStarLitePlanner::predecessors(uint64_t u) const {
    std::vector<const Transition*> result;
    if (bad_.count(u)) {
        return result;
    }
    auto it = incoming_.find(u);
    if (it == incoming_.end()) {
        return result;
    }
    for (const Transition* t : it->second) {
        if (t->available && !bad_.count(t->from)) {
            result.push_back(t);
        }
    }
    return result;
}
const Transition* DStarLitePlanner::getTransition(uint64_t from, uint64_t to) const {
    const Transition* best = nullptr;
    for (const Transition* t : successors(from)) {
        if (t->to == to && (!best || t->cost < best->cost)) {
            best = t;
        }
    }
    return best;
}
double DStarLitePlanner::heuristic(uint64_t, uint64_t) const { return 0.0; }
double DStarLitePlanner::calculateQuality(uint64_t u) const {
    return alpha_ * reliability_.at(u) + beta_ * continuationSafety_.at(u);
}

void DStarLitePlanner::updateVertex(uint64_t u) {
    if (bad_.count(u)) {
        return;
    }
    if (u != goal_) {
        rhs_[u] = kInfinity;
        reliability_[u] = 0.0;
        continuationSafety_[u] = 0.0;
        for (const Transition* t : successors(u)) {
            if (g_[t->to] >= kInfinity / 2.0) {
                continue;
            }
            const double candidate = t->cost + g_[t->to];
            const double candidateReliability = t->reliability * reliability_[t->to];
            const double candidateSafety = std::min(safety_[u], continuationSafety_[t->to]);
            const double candidateQuality = alpha_ * candidateReliability + beta_ * candidateSafety;
            const double currentQuality = alpha_ * reliability_[u] + beta_ * continuationSafety_[u];
            if (candidate < rhs_[u] - kEpsilon ||
                (equal(candidate, rhs_[u]) && candidateQuality > currentQuality + kEpsilon)) {
                rhs_[u] = candidate;
                reliability_[u] = candidateReliability;
                continuationSafety_[u] = candidateSafety;
            }
        }
    }
    if (!equal(g_[u], rhs_[u])) {
        open_.push(u, calculateKey(u));
    }
}

void DStarLitePlanner::computeShortestPath() {
    while (!open_.empty()) {
        const auto entry = open_.top();
        const Key current = calculateKey(entry.state);
        if (lessKey(entry.key, current)) {
            open_.pop();
            open_.push(entry.state, current);
            continue;
        }
        if (lessKey(current, entry.key)) {
            open_.pop();
            open_.push(entry.state, current);
            continue;
        }
        open_.pop();
        if (equal(g_[entry.state], rhs_[entry.state])) {
            continue;
        }
        ++exploredStates_;
        if (g_[entry.state] > rhs_[entry.state]) {
            g_[entry.state] = rhs_[entry.state];
            for (const Transition* t : predecessors(entry.state)) {
                updateVertex(t->from);
            }
        } else {
            g_[entry.state] = kInfinity;
            updateVertex(entry.state);
            for (const Transition* t : predecessors(entry.state)) {
                updateVertex(t->from);
            }
        }
    }
}

bool DStarLitePlanner::extractPath(
    std::vector<uint64_t>& states,
    std::vector<uint64_t>& transitions
) const {
    uint64_t current = start_;
    std::unordered_set<uint64_t> visited;
    states.push_back(current);
    while (current != goal_) {
        if (!visited.insert(current).second) {
            return false;
        }
        const Transition* best = nullptr;
        double bestScore = kInfinity;
        for (const Transition* t : successors(current)) {
            if (g_.at(t->to) >= kInfinity / 2.0) {
                continue;
            }
            const double continuationReliability = t->reliability * reliability_.at(t->to);
            const double candidateSafety = std::min(safety_.at(current), continuationSafety_.at(t->to));
            const double quality = alpha_ * continuationReliability + beta_ * candidateSafety;
            if (quality <= 0.0) {
                continue;
            }
            const double score = (t->cost + g_.at(t->to)) / quality;
            if (!best || score < bestScore - kEpsilon ||
                (equal(score, bestScore) && t->id < best->id)) {
                best = t;
                bestScore = score;
            }
        }
        if (!best) {
            return false;
        }
        transitions.push_back(best->id);
        current = best->to;
        states.push_back(current);
    }
    return true;
}

double DStarLitePlanner::calculatePathReliability(const std::vector<uint64_t>& transitionPath) const {
    double value = 1.0;
    for (uint64_t id : transitionPath) {
        auto it = std::find_if(problem_.transitions.begin(), problem_.transitions.end(),
                               [id](const Transition& t) { return t.id == id; });
        if (it == problem_.transitions.end()) {
            return 0.0;
        }
        value *= it->reliability;
    }
    return value;
}
double DStarLitePlanner::calculatePathSafety(const std::vector<uint64_t>& statePath) const {
    if (statePath.empty()) {
        return 0.0;
    }
    double value = 1.0;
    for (uint64_t id : statePath) {
        value = std::min(value, safety_.at(id));
    }
    return value;
}

PlanningResult DStarLitePlanner::plan(const PlanningProblem& problem) {
    const auto startedAt = std::chrono::steady_clock::now();
    const bool isReplan = initialized_;
    PlanningResult result;
    exploredStates_ = 0;
    const auto finish = [this, startedAt, isReplan](PlanningResult value) {
        const auto elapsed = std::chrono::steady_clock::now() - startedAt;
        value.exploredStates = exploredStates_;
        value.planningTimeMs = std::chrono::duration<double, std::milli>(elapsed).count();
        value.replanningTimeMs = isReplan ? value.planningTimeMs : 0.0;
        value.peakMemoryKB = processPeakMemoryKB();
        return value;
    };
    if (!initialized_) {
        start_ = problem.initialState;
        goal_ = problem.goalState;
        initialize(problem);
        initialized_ = true;
    } else {
        applyChanges(problem);
    }
    if (bad_.count(start_) || bad_.count(goal_)) {
        return finish(result);
    }
    computeShortestPath();
    if (g_[start_] >= kInfinity / 2.0) {
        initialize(problem_);
        computeShortestPath();
        if (g_[start_] >= kInfinity / 2.0) {
            return finish(result);
        }
    }
    if (!extractPath(result.statePath, result.transitionPath)) {
        result.statePath.clear();
        result.transitionPath.clear();
        initialize(problem_);
        computeShortestPath();
        if (!extractPath(result.statePath, result.transitionPath)) {
            result.statePath.clear();
            result.transitionPath.clear();
            return finish(result);
        }
    }

    result.totalCost = 0.0;
    for (uint64_t id : result.transitionPath) {
        auto it = std::find_if(problem_.transitions.begin(), problem_.transitions.end(),
                               [id](const Transition& t) { return t.id == id; });
        result.totalCost += it->cost;
    }
    (void)calculatePathReliability(result.transitionPath);
    result.safetyScore = calculatePathSafety(result.statePath);
    result.success = true;
    return finish(result);
}
