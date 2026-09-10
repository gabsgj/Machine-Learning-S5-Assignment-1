#include "LPAStarPlanner.h"
#include <queue>
#include <limits>
#include <cmath>
#include <unordered_set>
#include <iostream>
#include <algorithm>
#include <vector>
#include <functional>
#include <chrono>

using NodeId = uint64_t;

double LPAStarPlanner::euclidean(const std::vector<double>& a, const std::vector<double>& b) const {
    double s=0.0;
    for (size_t i=0;i<a.size() && i<b.size();++i) { double d = a[i]-b[i]; s += d*d; }
    return std::sqrt(s);
}

double LPAStarPlanner::safetyOfState(uint64_t sid, const PlanningProblem& problem) const {
    const State* s_ptr = nullptr;
    for (auto &s : problem.states) if (s.tid==sid) { s_ptr=&s; break; }
    if (!s_ptr) return 0.0;
    double minD = std::numeric_limits<double>::infinity();
    for (auto b : problem.badStates) {
        const State* b_ptr = nullptr;
        for (auto &bs : problem.states) if (bs.tid==b) { b_ptr=&bs; break; }
        if (!b_ptr) continue;
        double d = euclidean(s_ptr->embedding, b_ptr->embedding);
        if (d < minD) minD = d;
    }
    if (problem.badStates.empty()) return std::numeric_limits<double>::infinity();
    return minD;
}

static bool isBadState(uint64_t stateId, const PlanningProblem& problem) {
    return std::find(problem.badStates.begin(), problem.badStates.end(), stateId) != problem.badStates.end();
}

std::size_t LPAStarPlanner::transitionSignature(const PlanningProblem& problem) const {
    std::size_t signature = problem.transitions.size();
    for (const auto& transition : problem.transitions) {
        signature ^= std::hash<uint64_t>{}(transition.tid) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<uint64_t>{}(transition.from) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<uint64_t>{}(transition.to) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<double>{}(transition.cost) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<double>{}(transition.safety) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<double>{}(transition.reliability) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
        signature ^= std::hash<bool>{}(transition.available) + 0x9e3779b9 + (signature << 6) + (signature >> 2);
    }
    return signature;
}

struct LpaKey {
    double first;
    double second;
    NodeId id;
};

struct LpaKeyComp {
    bool operator()(const LpaKey& left, const LpaKey& right) const {
        if (left.first != right.first) return left.first > right.first;
        if (left.second != right.second) return left.second > right.second;
        return left.id > right.id;
    }
};

PlanningResult LPAStarPlanner::plan(const PlanningProblem& problem) {
    const auto startTime = std::chrono::steady_clock::now();
    PlanningResult result;
    PlanningProblem prob = problem;
    const std::size_t currentSignature = transitionSignature(problem);
    bool reusedGraph = false;
    if (!adjacencyCacheValid || adjacencySignature != currentSignature) {
        prob.buildAdjacency();
        cachedAdjacency = prob.adj;
        adjacencySignature = currentSignature;
        adjacencyCacheValid = true;
    } else {
        prob.adj = cachedAdjacency;
        reusedGraph = true;
    }
    const auto finish = [&](PlanningResult output) {
        const auto endTime = std::chrono::steady_clock::now();
        output.planningTimeMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        output.replanningTimeMs = reusedGraph ? output.planningTimeMs : 0.0;
        for (const auto& state : output.statePath) {
            if (isBadState(state, prob)) ++output.badStatesVisited;
        }
        output.memoryUsageBytes = sizeof(prob) + prob.states.capacity() * sizeof(State) +
                                  prob.transitions.capacity() * sizeof(Transition);
        for (const auto& state : prob.states) output.memoryUsageBytes += state.embedding.capacity() * sizeof(double);
        for (const auto& entry : prob.adj) output.memoryUsageBytes += entry.second.capacity() * sizeof(Transition);
        output.reusedCachedGraph = reusedGraph;
        return output;
    };

    const NodeId start = prob.initialState;
    const NodeId goal = prob.goalState;
    if (isBadState(start, prob) || isBadState(goal, prob)) return finish(result);

    std::unordered_map<NodeId, std::vector<Transition>> incoming;
    for (const auto& entry : prob.adj) {
        for (const auto& transition : entry.second) {
            if (!isBadState(transition.to, prob)) incoming[transition.to].push_back(transition);
        }
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::unordered_map<NodeId, double> g;
    std::unordered_map<NodeId, double> rhs;
    std::priority_queue<LpaKey, std::vector<LpaKey>, LpaKeyComp> queue;

    const auto value = [&](NodeId id, const std::unordered_map<NodeId, double>& values) {
        auto found = values.find(id);
        return found == values.end() ? infinity : found->second;
    };
    const auto heuristic = [&](NodeId from) {
        const State* fromState = nullptr;
        const State* goalState = nullptr;
        for (const auto& state : prob.states) {
            if (state.tid == from) fromState = &state;
            if (state.tid == goal) goalState = &state;
        }
        return fromState && goalState ? heurW * euclidean(fromState->embedding, goalState->embedding) : 0.0;
    };
    const auto edgeCost = [&](const Transition& transition) {
        const double safety = safetyOfState(transition.to, prob) + std::max(0.0, transition.safety);
         const double safetyPenalty = safety < infinity ? safetyW / (1.0 + safety) : 0.0;
         const double reliability = std::clamp(transition.reliability, 0.0, 1.0);
         return costW * std::max(0.0, transition.cost) + safetyPenalty +
             reliabilityW * (1.0 - reliability);
    };
    const auto key = [&](NodeId id) {
        const double best = std::min(value(id, g), value(id, rhs));
        return LpaKey{best + heuristic(id), best, id};
    };
    const auto updateVertex = [&](NodeId id) {
        if (id != start) {
            double best = infinity;
            auto found = incoming.find(id);
            if (found != incoming.end()) {
                for (const auto& transition : found->second) {
                    best = std::min(best, value(transition.from, g) + edgeCost(transition));
                }
            }
            rhs[id] = best;
        }
        if (value(id, g) != value(id, rhs)) queue.push(key(id));
    };

    rhs[start] = 0.0;
    queue.push(key(start));
    while (!queue.empty()) {
        const LpaKey current = queue.top();
        const LpaKey goalKey = key(goal);
        if (!LpaKeyComp{}(goalKey, current) && value(goal, rhs) == value(goal, g)) break;
        queue.pop();
        ++result.exploredStates;
        if (value(current.id, g) > value(current.id, rhs)) {
            g[current.id] = value(current.id, rhs);
        } else {
            g[current.id] = infinity;
            updateVertex(current.id);
        }
        auto outgoing = prob.adj.find(current.id);
        if (outgoing != prob.adj.end()) {
            for (const auto& transition : outgoing->second) updateVertex(transition.to);
        }
    }

    if (value(goal, g) == infinity) return result;
    std::vector<NodeId> reversePath{goal};
    std::unordered_set<NodeId> visited{goal};
    while (reversePath.back() != start) {
        const NodeId current = reversePath.back();
        double best = infinity;
        const Transition* bestTransition = nullptr;
        for (const auto& transition : incoming[current]) {
            const double candidate = value(transition.from, g) + edgeCost(transition);
            if (candidate < best && !visited.count(transition.from)) {
                best = candidate;
                bestTransition = &transition;
            }
        }
        if (!bestTransition) return finish(PlanningResult{});
        reversePath.push_back(bestTransition->from);
        visited.insert(bestTransition->from);
    }

    std::reverse(reversePath.begin(), reversePath.end());
    result.success = true;
    result.statePath = reversePath;
    double minimumSafety = infinity;
    result.reliabilityScore = 1.0;
    for (size_t index = 0; index + 1 < reversePath.size(); ++index) {
        for (const auto& transition : prob.adj[reversePath[index]]) {
            if (transition.to == reversePath[index + 1]) {
                result.transitionPath.push_back(transition.tid);
                result.totalCost += transition.cost;
                minimumSafety = std::min(minimumSafety, safetyOfState(transition.to, prob));
                result.reliabilityScore *= std::clamp(transition.reliability, 0.0, 1.0);
                break;
            }
        }
    }
    for (NodeId state : result.statePath) minimumSafety = std::min(minimumSafety, safetyOfState(state, prob));
    result.safetyScore = minimumSafety;
    return finish(result);
}
