#include "planner.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

Planner::Planner() = default;

double Planner::euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return 0.0;
    size_t dim = std::min(a.size(), b.size());
    double sum = 0.0;
    for (size_t i = 0; i < dim; ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double Planner::heuristic(const State& a, const State& b) {
    return euclideanDistance(a.embedding, b.embedding);
}

double Planner::stateSafetyDistance(uint64_t stateId) const {
    if (badStateSet_.empty()) {
        return INF;
    }
    if (badStateSet_.find(stateId) != badStateSet_.end()) {
        return 0.0;
    }
    auto it = stateMap_.find(stateId);
    if (it == stateMap_.end()) return 0.0;

    double minDist = INF;
    for (uint64_t badId : badStateSet_) {
        auto badIt = stateMap_.find(badId);
        if (badIt != stateMap_.end()) {
            double d = heuristic(it->second, badIt->second);
            if (d < minDist) {
                minDist = d;
            }
        }
    }
    return minDist;
}

double Planner::getEffectiveCost(const Transition& t) const {
    if (!t.available) return INF;
    if (badStateSet_.find(t.from) != badStateSet_.end() || badStateSet_.find(t.to) != badStateSet_.end()) {
        return INF;
    }
    double effCost = t.cost;
    if (safetyWeight_ > 0.0) {
        double safetyDist = stateSafetyDistance(t.to);
        if (safetyDist < INF && safetyDist > 1e-6) {
            effCost += safetyWeight_ * (1.0 / safetyDist);
        }
    }
    return effCost;
}

void Planner::setSafetyWeight(double weight) {
    safetyWeight_ = std::max(0.0, weight);
}

double Planner::getSafetyWeight() const {
    return safetyWeight_;
}

DStarKey Planner::calculateKey(uint64_t u) {
    double minVal = std::min(g_[u], rhs_[u]);
    double h = 0.0;
    if (stateMap_.find(startState_) != stateMap_.end() && stateMap_.find(u) != stateMap_.end()) {
        h = heuristic(stateMap_.at(startState_), stateMap_.at(u));
    }
    return DStarKey{minVal + h + km_, minVal};
}

void Planner::updateVertex(uint64_t u) {
    if (badStateSet_.find(u) != badStateSet_.end()) {
        rhs_[u] = INF;
        g_[u] = INF;
        auto it = openMap_.find(u);
        if (it != openMap_.end()) {
            openSet_.erase({it->second, u});
            openMap_.erase(it);
        }
        return;
    }

    if (u != goalState_) {
        double minRhs = INF;
        auto succIt = succTransitions_.find(u);
        if (succIt != succTransitions_.end()) {
            for (uint64_t tId : succIt->second) {
                const auto& t = transitionMap_.at(tId);
                double c = getEffectiveCost(t);
                if (c < INF && g_[t.to] < INF) {
                    minRhs = std::min(minRhs, c + g_[t.to]);
                }
            }
        }
        rhs_[u] = minRhs;
    }

    // Remove from open list if already present
    auto it = openMap_.find(u);
    if (it != openMap_.end()) {
        openSet_.erase({it->second, u});
        openMap_.erase(it);
    }

    // Insert if inconsistent
    if (std::abs(g_[u] - rhs_[u]) > 1e-9) {
        DStarKey key = calculateKey(u);
        openSet_.insert({key, u});
        openMap_[u] = key;
    }
}

void Planner::computeShortestPath() {
    while (!openSet_.empty()) {
        auto topEntry = *openSet_.begin();
        DStarKey topKey = topEntry.first;
        uint64_t u = topEntry.second;
        DStarKey startKey = calculateKey(startState_);

        if (!(topKey < startKey) && std::abs(rhs_[startState_] - g_[startState_]) < 1e-9) {
            break;
        }

        openSet_.erase(openSet_.begin());
        openMap_.erase(u);
        nodesExplored_++;

        DStarKey currentKey = calculateKey(u);
        if (topKey < currentKey) {
            openSet_.insert({currentKey, u});
            openMap_[u] = currentKey;
        } else if (g_[u] > rhs_[u]) { // Overconsistent
            g_[u] = rhs_[u];
            auto predIt = predTransitions_.find(u);
            if (predIt != predTransitions_.end()) {
                for (uint64_t tId : predIt->second) {
                    updateVertex(transitionMap_.at(tId).from);
                }
            }
        } else { // Underconsistent
            g_[u] = INF;
            auto predIt = predTransitions_.find(u);
            if (predIt != predTransitions_.end()) {
                for (uint64_t tId : predIt->second) {
                    updateVertex(transitionMap_.at(tId).from);
                }
            }
            updateVertex(u);
        }
    }
}

PlanningResult Planner::plan(const PlanningProblem& problem) {
    auto startTime = std::chrono::high_resolution_clock::now();

    problem_ = problem;
    startState_ = problem.initialState;
    goalState_ = problem.goalState;
    lastStartState_ = startState_;
    km_ = 0.0;
    nodesExplored_ = 0;

    badStateSet_.clear();
    for (uint64_t bad : problem.badStates) {
        badStateSet_.insert(bad);
    }

    stateMap_.clear();
    succTransitions_.clear();
    predTransitions_.clear();
    for (const auto& s : problem.states) {
        stateMap_[s.id] = s;
        succTransitions_[s.id] = {};
        predTransitions_[s.id] = {};
    }

    transitionMap_.clear();
    for (const auto& t : problem.transitions) {
        transitionMap_[t.id] = t;
        succTransitions_[t.from].push_back(t.id);
        predTransitions_[t.to].push_back(t.id);
    }

    g_.clear();
    rhs_.clear();
    for (const auto& s : problem.states) {
        g_[s.id] = INF;
        rhs_[s.id] = INF;
    }

    openSet_.clear();
    openMap_.clear();

    // Check validity of start and goal
    if (stateMap_.find(startState_) == stateMap_.end() ||
        stateMap_.find(goalState_) == stateMap_.end() ||
        badStateSet_.find(startState_) != badStateSet_.end() ||
        badStateSet_.find(goalState_) != badStateSet_.end()) {
        auto endTime = std::chrono::high_resolution_clock::now();
        PlanningResult res;
        res.success = false;
        res.planningTimeUs = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        res.nodesExplored = 0;
        res.badStatesVisited = 0;
        return res;
    }

    rhs_[goalState_] = 0.0;
    DStarKey goalKey = calculateKey(goalState_);
    openSet_.insert({goalKey, goalState_});
    openMap_[goalState_] = goalKey;

    computeShortestPath();

    PlanningResult res = extractPath();
    auto endTime = std::chrono::high_resolution_clock::now();
    res.planningTimeUs = std::chrono::duration<double, std::micro>(endTime - startTime).count();
    res.nodesExplored = nodesExplored_;
    return res;
}

void Planner::updateGoal(uint64_t goal) {
    if (goal == goalState_) return;
    goalState_ = goal;

    // In D* Lite, changing the goal re-roots the backward search
    for (const auto& pair : stateMap_) {
        g_[pair.first] = INF;
        rhs_[pair.first] = INF;
    }
    openSet_.clear();
    openMap_.clear();
    km_ = 0.0;
    lastStartState_ = startState_;

    if (stateMap_.find(goalState_) != stateMap_.end() && badStateSet_.find(goalState_) == badStateSet_.end()) {
        rhs_[goalState_] = 0.0;
        DStarKey key = calculateKey(goalState_);
        openSet_.insert({key, goalState_});
        openMap_[goalState_] = key;
    }
}

void Planner::updateStart(uint64_t start) {
    if (start == startState_) return;
    if (stateMap_.find(lastStartState_) != stateMap_.end() && stateMap_.find(start) != stateMap_.end()) {
        km_ += heuristic(stateMap_.at(lastStartState_), stateMap_.at(start));
    }
    startState_ = start;
    lastStartState_ = start;
}

void Planner::updateTransition(uint64_t transitionId, bool available) {
    auto it = transitionMap_.find(transitionId);
    if (it == transitionMap_.end()) return;
    it->second.available = available;

    if (startState_ != lastStartState_) {
        if (stateMap_.find(lastStartState_) != stateMap_.end() && stateMap_.find(startState_) != stateMap_.end()) {
            km_ += heuristic(stateMap_.at(lastStartState_), stateMap_.at(startState_));
        }
        lastStartState_ = startState_;
    }

    updateVertex(it->second.from);
}

void Planner::addTransition(const Transition& transition) {
    problem_.transitions.push_back(transition);
    transitionMap_[transition.id] = transition;
    succTransitions_[transition.from].push_back(transition.id);
    predTransitions_[transition.to].push_back(transition.id);

    if (startState_ != lastStartState_) {
        if (stateMap_.find(lastStartState_) != stateMap_.end() && stateMap_.find(startState_) != stateMap_.end()) {
            km_ += heuristic(stateMap_.at(lastStartState_), stateMap_.at(startState_));
        }
        lastStartState_ = startState_;
    }

    updateVertex(transition.from);
}

PlanningResult Planner::replan() {
    auto startTime = std::chrono::high_resolution_clock::now();
    nodesExplored_ = 0;

    computeShortestPath();

    PlanningResult res = extractPath();
    auto endTime = std::chrono::high_resolution_clock::now();
    res.planningTimeUs = std::chrono::duration<double, std::micro>(endTime - startTime).count();
    res.nodesExplored = nodesExplored_;
    return res;
}

PlanningResult Planner::extractPath() {
    PlanningResult res;

    if (badStateSet_.find(startState_) != badStateSet_.end() ||
        badStateSet_.find(goalState_) != badStateSet_.end()) {
        res.success = false;
        return res;
    }

    if (startState_ == goalState_) {
        res.success = true;
        res.statePath = {startState_};
        res.transitionPath = {};
        res.totalCost = 0.0;
        res.safetyScore = stateSafetyDistance(startState_);
        res.badStatesVisited = 0;
        return res;
    }

    if (g_[startState_] >= INF && rhs_[startState_] >= INF) {
        res.success = false;
        return res;
    }

    uint64_t curr = startState_;
    res.statePath.push_back(curr);
    std::unordered_set<uint64_t> visited;
    visited.insert(curr);

    while (curr != goalState_) {
        double minVal = INF;
        uint64_t bestNext = 0;
        uint64_t bestTId = 0;

        auto succIt = succTransitions_.find(curr);
        if (succIt != succTransitions_.end()) {
            for (uint64_t tId : succIt->second) {
                const auto& t = transitionMap_.at(tId);
                double c = getEffectiveCost(t);
                if (c < INF && g_[t.to] < INF) {
                    double val = c + g_[t.to];
                    if (val < minVal) {
                        minVal = val;
                        bestNext = t.to;
                        bestTId = t.id;
                    }
                }
            }
        }

        if (minVal >= INF || visited.find(bestNext) != visited.end()) {
            // No valid forward edge or cycle detected
            res.success = false;
            return res;
        }

        res.transitionPath.push_back(bestTId);
        res.totalCost += transitionMap_.at(bestTId).cost;
        curr = bestNext;
        res.statePath.push_back(curr);
        visited.insert(curr);
    }

    // Safety and bad-state check
    double minSafety = INF;
    size_t badVisited = 0;
    for (uint64_t s : res.statePath) {
        if (badStateSet_.find(s) != badStateSet_.end()) {
            badVisited++;
        }
        double d = stateSafetyDistance(s);
        if (d < minSafety) {
            minSafety = d;
        }
    }

    res.safetyScore = minSafety;
    res.badStatesVisited = badVisited;
    res.success = (badVisited == 0);
    return res;
}
