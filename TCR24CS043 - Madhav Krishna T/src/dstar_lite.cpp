#include "safe_semantic_planner/dstar_lite.hpp"
#include <cmath>
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <chrono>

namespace safe_semantic_planner {

namespace {
// Infinity is a valid D* Lite sentinel.  abs(inf - inf) is NaN, so a
// tolerance-only comparison incorrectly treats two infinite values as unequal.
bool sameValue(double a, double b) {
    return a == b || std::abs(a - b) < 1e-9;
}
}

double DStarLitePlanner::computeEdgeWeight(double cost, double reliability) const {
    return params_.beta * cost - params_.delta * reliability;
}

double DStarLitePlanner::computeHeuristic(const std::string& stateId, const std::string& targetGoalId) const {
    auto itS = states_.find(stateId);
    auto itG = states_.find(targetGoalId);
    if (itS == states_.end() || itG == states_.end()) {
        return 0.0;
    }
    double dist = ProblemLoader::computeEuclideanDistance(itS->second.embedding, itG->second.embedding);
    return cMin_ * dist;
}

DStarKey DStarLitePlanner::calculateKey(const std::string& stateId) const {
    double gVal = getG(stateId);
    double rhsVal = getRHS(stateId);
    double m = std::min(gVal, rhsVal);

    DStarKey key;
    if (m >= std::numeric_limits<double>::infinity() / 2.0) {
        key.k1 = std::numeric_limits<double>::infinity();
        key.k2 = std::numeric_limits<double>::infinity();
        return key;
    }

    key.k1 = m + computeHeuristic(stateId, sGoal_) + km_;
    key.k2 = m;
    return key;
}

void DStarLitePlanner::initialize(const PlanningProblem& problem, const WeightParams& params) {
    problem_ = problem;
    params_ = params;

    sInit_ = problem_.initialState;
    sGoal_ = problem_.goalState;
    sLastGoal_ = sGoal_;
    km_ = 0.0;
    currentCallExpansions_ = 0;
    isInitialSolve_ = true;
    lastPlanningTimeMs_ = 0.0;
    lastReplanningTimeMs_ = 0.0;

    states_.clear();
    transitions_.clear();
    successors_.clear();
    predecessors_.clear();
    edgeLookup_.clear();
    g_.clear();
    rhs_.clear();
    inOpenList_.clear();
    while (!openList_.empty()) {
        openList_.pop();
    }

    // Build KD-Tree over bad states
    std::vector<KdPoint> badPoints;
    std::unordered_set<std::string> badSet(problem_.badStates.begin(), problem_.badStates.end());
    for (const auto& s : problem_.states) {
        if (badSet.find(s.id) != badSet.end()) {
            badPoints.emplace_back(s.id, s.embedding);
        }
    }
    kdTree_.build(badPoints);

    // Initialize states
    for (auto s : problem_.states) {
        s.isBadState = (badSet.find(s.id) != badSet.end());
        if (badPoints.empty()) {
            s.distanceToNearestBadState = std::numeric_limits<double>::infinity();
        } else if (s.isBadState) {
            s.distanceToNearestBadState = 0.0;
        } else {
            s.distanceToNearestBadState = kdTree_.nearestDistance(s.embedding);
        }
        s.isExcluded = (s.distanceToNearestBadState <= params_.r);
        states_[s.id] = s;
        g_[s.id] = std::numeric_limits<double>::infinity();
        rhs_[s.id] = std::numeric_limits<double>::infinity();
    }

    // Process transitions & validate bounds
    double minCost = std::numeric_limits<double>::infinity();
    double maxReliability = 0.0;
    cMin_ = std::numeric_limits<double>::infinity();

    for (const auto& t : problem_.transitions) {
        InternalEdge edge;
        edge.transitionId = t.id;
        edge.from = t.from;
        edge.to = t.to;
        edge.cost = t.cost;
        edge.reliability = t.reliability;
        edge.safetyScore = t.safetyScore;
        edge.available = t.available;
        edge.weight = computeEdgeWeight(t.cost, t.reliability);

        transitions_[t.id] = edge;
        edgeLookup_[t.from + "->" + t.to] = t.id;
        successors_[t.from].push_back(t.to);
        predecessors_[t.to].push_back(t.from);

        const auto& fromState = states_[t.from];
        const auto& toState = states_[t.to];

        if (t.available && !fromState.isExcluded && !toState.isExcluded) {
            minCost = std::min(minCost, t.cost);
            maxReliability = std::max(maxReliability, t.reliability);
            double dist = ProblemLoader::computeEuclideanDistance(fromState.embedding, toState.embedding);
            if (dist > 1e-9) {
                double w = params_.beta * t.cost - params_.delta * t.reliability;
                cMin_ = std::min(cMin_, w / dist);
            }
        }
    }

    if (cMin_ == std::numeric_limits<double>::infinity()) {
        cMin_ = 1.0;
    }

    if (minCost < std::numeric_limits<double>::infinity()) {
        double wMin = params_.beta * minCost - params_.delta * maxReliability;
        if (wMin < 0.0) {
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(4);
            oss << "Load-time validation failed: Non-negative edge weight assumption violated. "
                << "w_min = beta * min(cost) - delta * max(reliability) = "
                << params_.beta << " * " << minCost << " - " << params_.delta << " * " << maxReliability
                << " = " << wMin << " < 0.0. "
                << "Offending bounds: min(cost)=" << minCost << ", max(reliability)=" << maxReliability
                << " with weights (beta=" << params_.beta << ", delta=" << params_.delta << ").";
            throw ValidationException(oss.str());
        }
    }

    // Set initial state rhs = 0.0 (D* Lite transposed goal)
    if (states_.find(sInit_) != states_.end()) {
        if (!states_[sInit_].isExcluded) {
            rhs_[sInit_] = 0.0;
            DStarKey key = calculateKey(sInit_);
            inOpenList_[sInit_] = key;
            openList_.push(QueueEntry{sInit_, key});
        }
    }
}

void DStarLitePlanner::updateVertex(const std::string& u) {
    if (states_.find(u) == states_.end()) return;

    if (states_[u].isExcluded) {
        rhs_[u] = std::numeric_limits<double>::infinity();
        g_[u] = std::numeric_limits<double>::infinity();
        inOpenList_.erase(u);
        return;
    }

    if (u != sInit_) {
        double minRhs = std::numeric_limits<double>::infinity();
        auto itPred = predecessors_.find(u);
        if (itPred != predecessors_.end()) {
            for (const auto& p : itPred->second) {
                if (states_[p].isExcluded) continue;
                auto itEdge = edgeLookup_.find(p + "->" + u);
                if (itEdge == edgeLookup_.end()) continue;
                const auto& edge = transitions_[itEdge->second];
                if (!edge.available) continue;

                double gP = getG(p);
                if (gP < std::numeric_limits<double>::infinity() / 2.0) {
                    double val = gP + edge.weight;
                    if (val < minRhs) {
                        minRhs = val;
                    }
                }
            }
        }
        rhs_[u] = minRhs;
    }

    bool consistent = sameValue(getG(u), getRHS(u));
    if (consistent) {
        inOpenList_.erase(u);
    } else {
        DStarKey key = calculateKey(u);
        inOpenList_[u] = key;
        openList_.push(QueueEntry{u, key});
    }
}

PlanningResult DStarLitePlanner::computeShortestPath() {
    auto startTime = std::chrono::high_resolution_clock::now();
    currentCallExpansions_ = 0;
    attemptCount_++;

    PlanningResult result;

    if (states_.find(sInit_) == states_.end() || states_.find(sGoal_) == states_.end()) {
        result.success = false;
        result.errorMessage = "Initial state or goal state does not exist in graph.";
        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        if (isInitialSolve_) {
            lastPlanningTimeMs_ = elapsedMs;
            isInitialSolve_ = false;
        } else {
            lastReplanningTimeMs_ = elapsedMs;
        }
        return result;
    }

    if (states_[sInit_].isExcluded || states_[sGoal_].isExcluded) {
        result.success = false;
        result.errorMessage = "Initial state or goal state is excluded by safety exclusion radius.";
        auto endTime = std::chrono::high_resolution_clock::now();
        double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
        if (isInitialSolve_) {
            lastPlanningTimeMs_ = elapsedMs;
            isInitialSolve_ = false;
        } else {
            lastReplanningTimeMs_ = elapsedMs;
        }
        return result;
    }

    while (true) {
        // D* Lite defines topKey as infinity when OPEN is empty.  Do not call
        // top()/pop() in that case: an inconsistent goal with OPEN exhausted
        // means the goal is unreachable, not that the queue may be dereferenced.
        const bool openEmpty = openList_.empty();
        const DStarKey topKey = openEmpty
            ? DStarKey{std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()}
            : openList_.top().key;
        DStarKey goalKey = calculateKey(sGoal_);
        double gGoal = getG(sGoal_);
        double rhsGoal = getRHS(sGoal_);

        bool goalConsistent = sameValue(gGoal, rhsGoal);
        bool topLessThanGoal = (topKey < goalKey);

        if (!topLessThanGoal && goalConsistent) {
            break;
        }

        // OPEN is exhausted while the goal remains inconsistent.  Its rhs
        // remains infinite, so extractPath() returns the canonical failure.
        if (openEmpty) {
            break;
        }

        std::string u = openList_.top().stateId;
        openList_.pop();

        // Lazy deletion: check if this entry is valid in inOpenList_
        auto itIn = inOpenList_.find(u);
        if (itIn == inOpenList_.end()) {
            continue; // Node already consistent / deleted from open list
        }
        
        // If key in queue is stale compared to stored key
        if (!sameValue(itIn->second.k1, topKey.k1) || !sameValue(itIn->second.k2, topKey.k2)) {
            continue; // Skip stale duplicate
        }

        DStarKey curKey = calculateKey(u);
        if (topKey < curKey) {
            inOpenList_[u] = curKey;
            openList_.push(QueueEntry{u, curKey});
        } else if (getG(u) > getRHS(u)) {
            // Overconsistent: g(u) > rhs(u)
            inOpenList_.erase(u);
            currentCallExpansions_++;
            g_[u] = rhs_[u];
            auto itSucc = successors_.find(u);
            if (itSucc != successors_.end()) {
                for (const auto& s : itSucc->second) {
                    updateVertex(s);
                }
            }
        } else {
            // Underconsistent: g(u) <= rhs(u)
            currentCallExpansions_++;
            g_[u] = std::numeric_limits<double>::infinity();
            updateVertex(u);
            auto itSucc = successors_.find(u);
            if (itSucc != successors_.end()) {
                for (const auto& s : itSucc->second) {
                    updateVertex(s);
                }
            }
        }
    }

    result = extractPath();
    if (result.success) {
        goalSuccessCount_++;
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    double elapsedMs = std::chrono::duration<double, std::milli>(endTime - startTime).count();
    if (isInitialSolve_) {
        lastPlanningTimeMs_ = elapsedMs;
        isInitialSolve_ = false;
    } else {
        lastReplanningTimeMs_ = elapsedMs;
    }

    return result;
}

PlanningResult DStarLitePlanner::extractPath() const {
    PlanningResult result;

    if (states_.find(sInit_) == states_.end() || states_.find(sGoal_) == states_.end()) {
        result.success = false;
        result.errorMessage = "Invalid initial or goal state.";
        return result;
    }

    if (states_.at(sInit_).isExcluded || states_.at(sGoal_).isExcluded) {
        result.success = false;
        result.errorMessage = "Initial or goal state is excluded by safety exclusion.";
        return result;
    }

    double gGoal = getG(sGoal_);
    if (gGoal >= std::numeric_limits<double>::infinity() / 2.0) {
        result.success = false;
        result.errorMessage = "No valid path found from initial state to goal.";
        return result;
    }

    std::vector<std::string> pathStates;
    std::vector<std::string> pathTransitions;
    std::string curr = sGoal_;
    pathStates.push_back(curr);

    std::unordered_set<std::string> visited;
    visited.insert(curr);

    double totalCost = 0.0;
    double totalSafety = 0.0;

    while (curr != sInit_) {
        std::string bestPred = "";
        std::string bestTransId = "";
        double bestVal = std::numeric_limits<double>::infinity();

        auto itPred = predecessors_.find(curr);
        if (itPred != predecessors_.end()) {
            for (const auto& p : itPred->second) {
                if (states_.at(p).isExcluded) continue;
                auto itEdge = edgeLookup_.find(p + "->" + curr);
                if (itEdge == edgeLookup_.end()) continue;
                const auto& edge = transitions_.at(itEdge->second);
                if (!edge.available) continue;

                double gP = getG(p);
                if (gP < std::numeric_limits<double>::infinity() / 2.0) {
                    double val = gP + edge.weight;
                    if (val < bestVal) {
                        bestVal = val;
                        bestPred = p;
                        bestTransId = edge.transitionId;
                    }
                }
            }
        }

        if (bestPred.empty() || visited.find(bestPred) != visited.end()) {
            result.success = false;
            result.errorMessage = "Disconnected predecessor or loop encountered during path extraction at node " + curr;
            return result;
        }

        const auto& chosenEdge = transitions_.at(bestTransId);
        totalCost += chosenEdge.cost;
        totalSafety += chosenEdge.safetyScore;

        pathTransitions.push_back(bestTransId);
        curr = bestPred;
        pathStates.push_back(curr);
        visited.insert(curr);
    }

    std::reverse(pathStates.begin(), pathStates.end());
    std::reverse(pathTransitions.begin(), pathTransitions.end());

    result.success = true;
    result.statePath = pathStates;
    result.transitionPath = pathTransitions;
    result.totalCost = totalCost;
    result.safetyScore = pathTransitions.empty() ? 1.0 : (totalSafety / pathTransitions.size());
    return result;
}

void DStarLitePlanner::notifyEdgeChanged(const std::string& transitionId, double newCost, bool newAvailability) {
    auto it = transitions_.find(transitionId);
    if (it == transitions_.end()) return;

    it->second.cost = newCost;
    it->second.available = newAvailability;
    it->second.weight = computeEdgeWeight(newCost, it->second.reliability);

    updateVertex(it->second.to);
}

void DStarLitePlanner::notifyGoalChanged(const std::string& newGoalStateId) {
    if (newGoalStateId == sGoal_) return;

    km_ += computeHeuristic(sLastGoal_, newGoalStateId);
    sGoal_ = newGoalStateId;
    sLastGoal_ = sGoal_;

    if (states_.find(sGoal_) != states_.end()) {
        updateVertex(sGoal_);
    }
}

void DStarLitePlanner::notifyBadStatesChanged(const std::vector<std::string>& newBadStates) {
    problem_.badStates = newBadStates;
    updateSafetyExclusions();
}

void DStarLitePlanner::notifySafetyRadiusChanged(double newRadius) {
    params_.r = newRadius;
    updateSafetyExclusions();
}

void DStarLitePlanner::updateSafetyExclusions() {
    std::vector<KdPoint> badPoints;
    std::unordered_set<std::string> badSet(problem_.badStates.begin(), problem_.badStates.end());
    for (const auto& pair : states_) {
        if (badSet.find(pair.first) != badSet.end()) {
            badPoints.emplace_back(pair.first, pair.second.embedding);
        }
    }
    kdTree_.build(badPoints);

    std::vector<std::string> changedStates;
    for (auto& pair : states_) {
        State& s = pair.second;
        s.isBadState = (badSet.find(s.id) != badSet.end());
        if (badPoints.empty()) {
            s.distanceToNearestBadState = std::numeric_limits<double>::infinity();
        } else if (s.isBadState) {
            s.distanceToNearestBadState = 0.0;
        } else {
            s.distanceToNearestBadState = kdTree_.nearestDistance(s.embedding);
        }
        bool prevExcluded = s.isExcluded;
        s.isExcluded = (s.distanceToNearestBadState <= params_.r);
        if (prevExcluded != s.isExcluded) {
            changedStates.push_back(s.id);
        }
    }

    for (const auto& sId : changedStates) {
        updateVertex(sId);
        auto itSucc = successors_.find(sId);
        if (itSucc != successors_.end()) {
            for (const auto& succ : itSucc->second) {
                updateVertex(succ);
            }
        }
    }
}

double DStarLitePlanner::getG(const std::string& id) const {
    auto it = g_.find(id);
    return (it != g_.end()) ? it->second : std::numeric_limits<double>::infinity();
}

double DStarLitePlanner::getRHS(const std::string& id) const {
    auto it = rhs_.find(id);
    return (it != rhs_.end()) ? it->second : std::numeric_limits<double>::infinity();
}

const State* DStarLitePlanner::getState(const std::string& id) const {
    auto it = states_.find(id);
    return (it != states_.end()) ? &it->second : nullptr;
}

const Transition* DStarLitePlanner::getTransition(const std::string& id) const {
    auto it = transitions_.find(id);
    if (it != transitions_.end()) {
        static Transition t;
        t.id = it->second.transitionId;
        t.from = it->second.from;
        t.to = it->second.to;
        t.cost = it->second.cost;
        t.reliability = it->second.reliability;
        t.safetyScore = it->second.safetyScore;
        t.available = it->second.available;
        return &t;
    }
    return nullptr;
}

MemoryAccounting DStarLitePlanner::getAnalyticalMemoryAccounting() const {
    MemoryAccounting mem;
    // g and rhs maps: sizeof(string) + sizeof(double) per state
    mem.gRhsTableBytes = states_.size() * (sizeof(std::string) + 2 * sizeof(double) + 32 /* hash node overhead */);
    
    // Priority queue
    mem.priorityQueueBytes = openList_.size() * sizeof(QueueEntry);

    // Graph adjacency
    size_t edgeBytes = transitions_.size() * sizeof(InternalEdge);
    size_t adjBytes = (successors_.size() + predecessors_.size()) * sizeof(std::vector<std::string>);
    mem.graphAdjacencyBytes = sizeof(DStarLitePlanner) + edgeBytes + adjBytes;

    // KdTree
    mem.kdTreeBytes = kdTree_.getAnalyticalMemoryBytes();

    mem.totalBytes = mem.gRhsTableBytes + mem.priorityQueueBytes + mem.graphAdjacencyBytes + mem.kdTreeBytes;
    return mem;
}

PlannerMetrics DStarLitePlanner::getMetrics() const {
    PlannerMetrics m;
    m.statesExplored = currentCallExpansions_;
    m.planningTimeMs = lastPlanningTimeMs_;
    m.replanningTimeMs = lastReplanningTimeMs_;
    m.goalSuccessCount = goalSuccessCount_;
    m.attemptCount = attemptCount_;

    // Analytical memory accounting calculation per assignment spec:
    // sizeof(g/rhs entry) * active state count + sizeof(heap entry) * current heap size + kd-tree node size * bad state count
    size_t activeCount = 0;
    for (const auto& pair : states_) {
        if (!pair.second.isExcluded) {
            activeCount++;
        }
    }

    size_t sizeofGRhsEntry = sizeof(std::string) + 2 * sizeof(double);
    size_t sizeofHeapEntry = sizeof(QueueEntry);
    size_t sizeofKdTreeNode = sizeof(KdNode) + sizeof(std::string) + 2 * sizeof(double);

    m.memoryBytes = (sizeofGRhsEntry * activeCount) +
                    (sizeofHeapEntry * openList_.size()) +
                    (sizeofKdTreeNode * problem_.badStates.size());

    // Path analysis metrics: totalCost, minClearance, badStatesVisited
    PlanningResult curPath = extractPath();
    if (curPath.success && !curPath.statePath.empty()) {
        m.totalCost = curPath.totalCost;
        double minClear = std::numeric_limits<double>::infinity();
        size_t badVisited = 0;

        for (const auto& sId : curPath.statePath) {
            const State* s = getState(sId);
            if (s) {
                if (s->isBadState) {
                    badVisited++;
                }
                if (s->distanceToNearestBadState < minClear) {
                    minClear = s->distanceToNearestBadState;
                }
            }
        }
        m.badStatesVisited = badVisited;
        m.minClearance = (minClear == std::numeric_limits<double>::infinity()) ? 0.0 : minClear;
    } else {
        m.totalCost = 0.0;
        m.minClearance = 0.0;
        m.badStatesVisited = 0;
    }

    return m;
}

} // namespace safe_semantic_planner
