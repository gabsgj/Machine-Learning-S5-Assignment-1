#include "../include/lpa_star.h"
#include "../include/heuristic.h"
#include "../include/safety.h"
#include <limits>
#include <algorithm>

bool LPAStarPlanner::isBadState(uint64_t stateId) {
    for (uint64_t b : problem.badStates) {
        if (b == stateId) return true;
    }
    return false;
}

double LPAStarPlanner::effectiveCost(const Transition& t) {
    double dist = distanceToNearestBadState(stateLookup[t.to], problem.badStates, problem.states);
    double penalty = (dist > 0.0001) ? (1.0 / dist) : 1000.0;
    return t.cost + penalty;
}

void LPAStarPlanner::pushQueue(uint64_t stateId) {
    auto key = calculateKey(stateId);
    queue.push_back({key.first, key.second, stateId});
}

void LPAStarPlanner::removeFromQueue(uint64_t stateId) {
    for (size_t i = 0; i < queue.size(); i++) {
        if (queue[i].stateId == stateId) {
            queue.erase(queue.begin() + i);
            return;
        }
    }
}

std::pair<double,double> LPAStarPlanner::calculateKey(uint64_t stateId) {
    double minVal = std::min(g[stateId], rhs[stateId]);
    double h = heuristic(stateLookup[stateId], stateLookup[problem.goalState]);
    return {minVal + h, minVal};
}

void LPAStarPlanner::initialize() {
    for (const State& s : problem.states) {
        g[s.id] = std::numeric_limits<double>::infinity();
        rhs[s.id] = std::numeric_limits<double>::infinity();
    }
    rhs[problem.initialState] = 0.0;
    pushQueue(problem.initialState);
}

void LPAStarPlanner::updateVertex(uint64_t u) {
    if (u != problem.initialState) {
        double bestRhs = std::numeric_limits<double>::infinity();
        for (const Transition& t : incoming[u]) {
            if (!t.available) continue;
            if (isBadState(t.from)) continue;
            auto it = g.find(t.from);
            if (it == g.end()) continue;
            double candidate = it->second + effectiveCost(t);
            if (candidate < bestRhs) bestRhs = candidate;
        }
        rhs[u] = bestRhs;
    }
    removeFromQueue(u);
    if (g[u] != rhs[u]) {
        pushQueue(u);
    }
}

void LPAStarPlanner::computeShortestPath() {
    while (!queue.empty()) {
        size_t minIndex = 0;
        for (size_t i = 1; i < queue.size(); i++) {
            if (queue[i].k1 < queue[minIndex].k1 ||
                (queue[i].k1 == queue[minIndex].k1 && queue[i].k2 < queue[minIndex].k2)) {
                minIndex = i;
            }
        }
        QueueEntry top = queue[minIndex];
        auto goalKey = calculateKey(problem.goalState);
        bool topLessThanGoal = (top.k1 < goalKey.first) ||
            (top.k1 == goalKey.first && top.k2 < goalKey.second);

        if (!topLessThanGoal && rhs[problem.goalState] == g[problem.goalState]) {
            break;
        }

        queue.erase(queue.begin() + minIndex);
        uint64_t u = top.stateId;
        statesExpanded++;

        if (g[u] > rhs[u]) {
            g[u] = rhs[u];
            for (const Transition& t : graphPtr->getOutgoing(u)) {
                if (t.available) updateVertex(t.to);
            }
        } else {
            g[u] = std::numeric_limits<double>::infinity();
            updateVertex(u);
            for (const Transition& t : graphPtr->getOutgoing(u)) {
                if (t.available) updateVertex(t.to);
            }
        }
    }
}

PlanningResult LPAStarPlanner::plan(const PlanningProblem& inputProblem) {
    problem = inputProblem;
    delete graphPtr;
    graphPtr = new Graph(problem);

    statesExpanded = 0;

    stateLookup.clear();
    for (const State& s : problem.states) stateLookup[s.id] = s;

    incoming.clear();
    for (const Transition& t : problem.transitions) incoming[t.to].push_back(t);

    g.clear();
    rhs.clear();
    queue.clear();

    initialize();
    computeShortestPath();

    return extractResult();
}

PlanningResult LPAStarPlanner::onTransitionUnavailable(uint64_t transitionId, uint64_t fromState, uint64_t toState) {
    statesExpanded = 0;
    graphPtr->setAvailable(transitionId, fromState, false);
    for (auto& t : incoming[toState]) {
        if (t.id == transitionId) t.available = false;
    }
    updateVertex(toState);
    computeShortestPath();
    return extractResult();
}

PlanningResult LPAStarPlanner::onTransitionAdded(const Transition& t) {
    statesExpanded = 0;
    graphPtr->addTransition(t);
    incoming[t.to].push_back(t);
    problem.transitions.push_back(t);
    updateVertex(t.to);
    computeShortestPath();
    return extractResult();
}

PlanningResult LPAStarPlanner::onBadStatesChanged(const std::vector<uint64_t>& newBadStates) {
    statesExpanded = 0;
    problem.badStates = newBadStates;
    for (const State& s : problem.states) {
        updateVertex(s.id);
    }
    computeShortestPath();
    return extractResult();
}

PlanningResult LPAStarPlanner::onGoalChanged(uint64_t newGoal) {
    statesExpanded = 0;
    problem.goalState = newGoal;
    std::vector<QueueEntry> oldQueue = queue;
    queue.clear();
    for (auto& entry : oldQueue) {
        pushQueue(entry.stateId);
    }
    computeShortestPath();
    return extractResult();
}

PlanningResult LPAStarPlanner::extractResult() {
    PlanningResult result;
    result.totalCost = 0;
    result.safetyScore = 0;

    if (g.find(problem.goalState) == g.end() ||
        g[problem.goalState] == std::numeric_limits<double>::infinity()) {
        result.success = false;
        return result;
    }

    std::vector<uint64_t> reversePath;
    uint64_t current = problem.goalState;
    reversePath.push_back(current);

    while (current != problem.initialState) {
        double bestG = std::numeric_limits<double>::infinity();
        uint64_t bestPred = 0;
        bool found = false;
        for (const Transition& t : incoming[current]) {
            if (!t.available || isBadState(t.from)) continue;
            auto it = g.find(t.from);
            if (it == g.end()) continue;
            double candidateG = it->second + effectiveCost(t);
            if (candidateG < bestG) {
                bestG = candidateG;
                bestPred = t.from;
                found = true;
            }
        }
        if (!found) {
            result.success = false;
            return result;
        }
        current = bestPred;
        reversePath.push_back(current);
    }

    std::vector<uint64_t> path(reversePath.rbegin(), reversePath.rend());
    result.success = true;
    result.statePath = path;

    double totalCost = 0;
    double minSafety = std::numeric_limits<double>::infinity();

    for (size_t i = 0; i + 1 < path.size(); i++) {
        for (const Transition& t : graphPtr->getOutgoing(path[i])) {
            if (t.to == path[i+1] && t.available) {
                totalCost += t.cost;
                result.transitionPath.push_back(t.id);
                break;
            }
        }
    }

    for (uint64_t sid : path) {
        double d = distanceToNearestBadState(stateLookup[sid], problem.badStates, problem.states);
        if (d < minSafety) minSafety = d;
    }

    result.totalCost = totalCost;
    result.safetyScore = minSafety;
    return result;
}