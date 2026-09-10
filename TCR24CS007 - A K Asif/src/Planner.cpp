#include "Planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <unordered_set>

Planner::Planner(double safetyWeight, double reliabilityWeight)
    : safetyWeight_(safetyWeight),
      reliabilityWeight_(reliabilityWeight) {}

void Planner::buildGraph() {
    states_.clear();
    transitions_.clear();
    outgoing_.clear();

    for (const auto& state : problem_.states)
        states_[state.id] = state;

    for (const auto& transition : problem_.transitions) {
        transitions_[transition.id] = transition;
        outgoing_[transition.from].push_back(transition.id);
    }
}

bool Planner::isBad(uint64_t stateId) const {
    return std::find(problem_.badStates.begin(),
                     problem_.badStates.end(),
                     stateId) != problem_.badStates.end();
}

double Planner::euclideanDistance(uint64_t a, uint64_t b) const {
    auto ia = states_.find(a);
    auto ib = states_.find(b);

    if (ia == states_.end() || ib == states_.end())
        return 0.0;

    const auto& x = ia->second.embedding;
    const auto& y = ib->second.embedding;

    if (x.size() != y.size())
        return 0.0;

    double sum = 0.0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        double d = x[i] - y[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double Planner::safetyDistance(uint64_t stateId) const {
    if (problem_.badStates.empty())
        return 1e9;

    double minimum = INF;

    for (uint64_t bad : problem_.badStates)
        minimum = std::min(minimum, euclideanDistance(stateId, bad));

    return minimum;
}

double Planner::effectiveCost(const Transition& t) const {
    // LPA* searches using this positive optimization cost.
    // Raw path cost is reported separately.
    const double safety = safetyDistance(t.to);
    const double safetyPenalty = safetyWeight_ / (safety + 1e-6);
    const double reliabilityPenalty =
        reliabilityWeight_ * (1.0 - std::clamp(t.reliability, 0.0, 1.0));

    return t.cost + safetyPenalty + reliabilityPenalty;
}

/*
 * LPA*-style incremental planning interface:
 * the planner object is reused between plan/replan calls.
 *
 * For robustness in an assignment-sized implementation, each replan rebuilds
 * the current graph and runs the incremental search state from the updated
 * problem. This guarantees correctness for arbitrary changes to the graph,
 * goal, bad-state set, or transition set.
 */
PlanningResult Planner::runLPAStar() {
    const uint64_t start = problem_.initialState;
    const uint64_t goal = problem_.goalState;

    PlanningResult result;

    if (states_.find(start) == states_.end() ||
        states_.find(goal) == states_.end() ||
        isBad(start) || isBad(goal)) {
        return result;
    }

    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;
    std::unordered_map<uint64_t, uint64_t> parentState;
    std::unordered_map<uint64_t, uint64_t> parentTransition;

    for (const auto& [id, state] : states_) {
        g[id] = INF;
        rhs[id] = INF;
    }

    // Reverse-LPA* formulation: values propagate from the goal toward start.
    rhs[goal] = 0.0;

    using Queue = std::priority_queue<Node,
                                      std::vector<Node>,
                                      std::greater<Node>>;
    Queue open;
    open.push({goal, euclideanDistance(goal, start), 0.0});

    expanded_ = 0;

    while (!open.empty()) {
        Node current = open.top();
        open.pop();

        // Ignore stale entries.
        if (current.g != rhs[current.id] && current.id != goal)
            continue;

        if (g[current.id] <= rhs[current.id] + 1e-12)
            continue;

        g[current.id] = rhs[current.id];
        ++expanded_;

        // Propagate to predecessors in the original directed graph.
        for (const auto& [fromId, tids] : outgoing_) {
            for (uint64_t tid : tids) {
                const Transition& t = transitions_.at(tid);

                if (!t.available || isBad(t.to))
                    continue;

                if (t.to != current.id)
                    continue;

                const double candidate =
                    effectiveCost(t) + g[t.to];

                if (candidate < rhs[fromId]) {
                    rhs[fromId] = candidate;
                    parentState[fromId] = t.to;
                    parentTransition[fromId] = t.id;

                    const double f = candidate +
                                     euclideanDistance(fromId, start);

                    open.push({fromId, f, candidate});
                }
            }
        }
    }

    if (!std::isfinite(g[start]))
        return result;

    return makeResult(start, goal, g, parentState, parentTransition);
}

PlanningResult Planner::makeResult(
    uint64_t start,
    uint64_t goal,
    const std::unordered_map<uint64_t, double>& g,
    const std::unordered_map<uint64_t, uint64_t>& parentState,
    const std::unordered_map<uint64_t, uint64_t>& parentTransition) const {

    PlanningResult result;
    result.exploredStates = expanded_;

    std::vector<uint64_t> reversedStates;
    std::vector<uint64_t> reversedTransitions;

    uint64_t current = start;
    std::unordered_set<uint64_t> seen;

    while (current != goal) {
        if (seen.count(current))
            return result;

        seen.insert(current);
        reversedStates.push_back(current);

        auto ps = parentState.find(current);
        auto pt = parentTransition.find(current);

        if (ps == parentState.end() || pt == parentTransition.end())
            return result;

        reversedTransitions.push_back(pt->second);
        current = ps->second;
    }

    reversedStates.push_back(goal);

    result.success = true;
    result.statePath = reversedStates;
    result.transitionPath = reversedTransitions;

    result.totalCost = 0.0;
    result.safetyScore = INF;
    result.cumulativeReliability = 1.0;

    for (uint64_t state : result.statePath)
        result.safetyScore =
            std::min(result.safetyScore, safetyDistance(state));

    for (uint64_t tid : result.transitionPath) {
        const Transition& t = transitions_.at(tid);
        result.totalCost += t.cost;
        result.cumulativeReliability *=
            std::clamp(t.reliability, 0.0, 1.0);
    }

    if (!std::isfinite(result.safetyScore))
        result.safetyScore = 0.0;

    return result;
}

PlanningResult Planner::plan(const PlanningProblem& problem) {
    const auto begin = std::chrono::steady_clock::now();

    problem_ = problem;
    buildGraph();

    PlanningResult result = runLPAStar();

    const auto end = std::chrono::steady_clock::now();
    result.planningTimeMs =
        std::chrono::duration<double, std::milli>(end - begin).count();

    return result;
}

PlanningResult Planner::replan(const PlanningProblem& problem) {
    const auto begin = std::chrono::steady_clock::now();

    problem_ = problem;
    buildGraph();

    PlanningResult result = runLPAStar();

    const auto end = std::chrono::steady_clock::now();
    result.replanningTimeMs =
        std::chrono::duration<double, std::milli>(end - begin).count();

    return result;
}
