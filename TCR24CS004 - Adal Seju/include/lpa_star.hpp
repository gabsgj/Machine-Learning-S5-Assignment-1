#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "heuristic.hpp"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <queue>
#include <algorithm>
#include <chrono>
#include <cassert>

namespace SafePlanner {

/**
 * @brief Lexicographical priority queue key [k1, k2] for LPA* and D* Lite.
 */
struct Key {
    double k1 = INF;
    double k2 = INF;

    Key() : k1(INF), k2(INF) {}
    Key(double first, double second) : k1(first), k2(second) {}

    bool operator<(const Key& other) const {
        if (std::abs(k1 - other.k1) > EPSILON) {
            return k1 < other.k1;
        }
        if (std::abs(k2 - other.k2) > EPSILON) {
            return k2 < other.k2;
        }
        return false;
    }

    bool operator>(const Key& other) const {
        return other < (*this);
    }

    bool operator<=(const Key& other) const {
        return !(other < (*this));
    }

    bool operator>=(const Key& other) const {
        return !((*this) < other);
    }

    bool operator==(const Key& other) const {
        return std::abs(k1 - other.k1) <= EPSILON && std::abs(k2 - other.k2) <= EPSILON;
    }

    bool operator!=(const Key& other) const {
        return !(*this == other);
    }
};

/**
 * @brief Lifelong Planning A* (LPA*) Implementation.
 * 
 * LPA* is an incremental version of A* that efficiently replans when edge costs,
 * availability, bad states, or goals change, reusing previous search trees without
 * recomputing from scratch.
 */
class LPAStarPlanner : public Planner {
public:
    struct EdgeInfo {
        uint64_t transitionId;
        uint64_t target; // For succ: to; For pred: from
        double cost;
        double safety;
        double reliability;
        bool available;
    };

    struct StepSnapshot {
        size_t stepIndex;
        uint64_t vertexExpanded;
        std::string eventType; // "Overconsistent", "Underconsistent", "Consistent"
        double gVal;
        double rhsVal;
        Key key;
        size_t queueSize;
    };

private:
    PlanningProblem problem_;
    std::unordered_map<uint64_t, State> statesMap_;
    std::unordered_map<uint64_t, Transition> transitionsMap_;
    std::unordered_set<uint64_t> badStateSet_;
    std::unordered_map<uint64_t, std::vector<double>> embeddings_;

    // Graph Adjacency
    std::unordered_map<uint64_t, std::vector<EdgeInfo>> successors_;
    std::unordered_map<uint64_t, std::vector<EdgeInfo>> predecessors_;

    // LPA* Core Distance Values
    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;

    // Heuristic Engine
    Heuristic heuristic_;

    // Priority Queue: sorted by (Key, vertexId)
    std::set<std::pair<Key, uint64_t>> priorityQueue_;
    std::unordered_map<uint64_t, Key> vertexKeysInQueue_;

    // Telemetry & Profiling
    size_t exploredCount_ = 0;
    size_t queueOpsCount_ = 0;
    std::vector<StepSnapshot> stepHistory_;
    bool recordSteps_ = false;

    // Safety edge penalty weight for multi-objective mode
    double safetyMargin_ = 0.0;
    double safetyWeight_ = 0.0;
    double reliabilityWeight_ = 0.0;

public:
    LPAStarPlanner() = default;
    ~LPAStarPlanner() override = default;

    /**
     * @brief Loads and initializes the planning problem.
     */
    void loadProblem(const PlanningProblem& problem) {
        problem_ = problem;
        statesMap_.clear();
        transitionsMap_.clear();
        badStateSet_.clear();
        embeddings_.clear();
        successors_.clear();
        predecessors_.clear();
        g_.clear();
        rhs_.clear();
        priorityQueue_.clear();
        vertexKeysInQueue_.clear();
        exploredCount_ = 0;
        queueOpsCount_ = 0;
        stepHistory_.clear();

        for (const auto& s : problem.states) {
            statesMap_[s.id] = s;
            embeddings_[s.id] = s.embedding;
            g_[s.id] = INF;
            rhs_[s.id] = INF;
        }

        for (uint64_t badId : problem.badStates) {
            badStateSet_.insert(badId);
        }

        for (const auto& t : problem.transitions) {
            transitionsMap_[t.id] = t;
            EdgeInfo fwd{t.id, t.to, t.cost, t.safety, t.reliability, t.available};
            EdgeInfo bwd{t.id, t.from, t.cost, t.safety, t.reliability, t.available};
            successors_[t.from].push_back(fwd);
            predecessors_[t.to].push_back(bwd);
        }

        heuristic_.calibrate(problem.states, problem.transitions, problem.badStates);

        // Start state initialization
        if (badStateSet_.find(problem.initialState) == badStateSet_.end()) {
            rhs_[problem.initialState] = 0.0;
            Key startKey = calculateKey(problem.initialState);
            insertQueue(problem.initialState, startKey);
        }
    }

    /**
     * @brief Computes or replans the path for the currently loaded problem.
     */
    PlanningResult plan(const PlanningProblem& problem) override {
        auto startTime = std::chrono::high_resolution_clock::now();
        loadProblem(problem);
        computeShortestPath();
        auto endTime = std::chrono::high_resolution_clock::now();

        PlanningResult result = extractResult();
        result.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        result.algorithmName = "LPA*";
        return result;
    }

    /**
     * @brief Incremental Replan without reinitializing state tree (warm restart).
     */
    PlanningResult replanIncremental() {
        auto startTime = std::chrono::high_resolution_clock::now();
        exploredCount_ = 0;
        queueOpsCount_ = 0;
        
        computeShortestPath();
        
        auto endTime = std::chrono::high_resolution_clock::now();
        PlanningResult result = extractResult();
        result.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        result.algorithmName = "LPA* (Incremental)";
        return result;
    }

    // ------------------------------------------------------------------------
    // Dynamic Environment Mutators (Triggers O(1) or O(deg) local updates)
    // ------------------------------------------------------------------------

    /**
     * @brief Dynamically changes edge availability.
     */
    void updateEdgeAvailability(uint64_t transitionId, bool available) {
        auto it = transitionsMap_.find(transitionId);
        if (it == transitionsMap_.end()) return;
        if (it->second.available == available) return;

        it->second.available = available;
        uint64_t u = it->second.from;
        uint64_t v = it->second.to;

        // Update in adjacency lists
        for (auto& edge : successors_[u]) {
            if (edge.transitionId == transitionId) edge.available = available;
        }
        for (auto& edge : predecessors_[v]) {
            if (edge.transitionId == transitionId) edge.available = available;
        }

        // LPA* local update: only v's rhs is directly affected!
        updateVertex(v);
    }

    /**
     * @brief Dynamically updates edge cost.
     */
    void updateEdgeCost(uint64_t transitionId, double newCost) {
        auto it = transitionsMap_.find(transitionId);
        if (it == transitionsMap_.end()) return;
        if (std::abs(it->second.cost - newCost) < EPSILON) return;

        it->second.cost = newCost;
        uint64_t u = it->second.from;
        uint64_t v = it->second.to;

        for (auto& edge : successors_[u]) {
            if (edge.transitionId == transitionId) edge.cost = newCost;
        }
        for (auto& edge : predecessors_[v]) {
            if (edge.transitionId == transitionId) edge.cost = newCost;
        }

        updateVertex(v);
    }

    /**
     * @brief Adds a new directed transition dynamically.
     */
    void addTransition(const Transition& t) {
        transitionsMap_[t.id] = t;
        successors_[t.from].push_back({t.id, t.to, t.cost, t.safety, t.reliability, t.available});
        predecessors_[t.to].push_back({t.id, t.from, t.cost, t.safety, t.reliability, t.available});
        updateVertex(t.to);
    }

    /**
     * @brief Removes an existing transition.
     */
    void removeTransition(uint64_t transitionId) {
        auto it = transitionsMap_.find(transitionId);
        if (it == transitionsMap_.end()) return;
        uint64_t u = it->second.from;
        uint64_t v = it->second.to;
        transitionsMap_.erase(it);

        auto& succs = successors_[u];
        succs.erase(std::remove_if(succs.begin(), succs.end(),
            [transitionId](const EdgeInfo& e){ return e.transitionId == transitionId; }), succs.end());

        auto& preds = predecessors_[v];
        preds.erase(std::remove_if(preds.begin(), preds.end(),
            [transitionId](const EdgeInfo& e){ return e.transitionId == transitionId; }), preds.end());

        updateVertex(v);
    }

    /**
     * @brief Dynamically marks or unmarks a bad state.
     */
    void setBadState(uint64_t stateId, bool isBad) {
        if (isBad) {
            badStateSet_.insert(stateId);
            // Force state to be invalid
            removeFromQueue(stateId);
            g_[stateId] = INF;
            rhs_[stateId] = INF;

            // Invalidate all outgoing and incoming neighbors
            for (const auto& succ : successors_[stateId]) {
                updateVertex(succ.target);
            }
        } else {
            badStateSet_.erase(stateId);
            updateVertex(stateId);
        }
    }

    /**
     * @brief Updates the goal state dynamically.
     * In forward LPA*, g-values are measured from initialState, so they remain valid!
     * We simply update all priority keys with the new heuristic and resume search.
     */
    void updateGoal(uint64_t newGoal) {
        if (problem_.goalState == newGoal) return;
        problem_.goalState = newGoal;

        // Re-key all vertices currently in the priority queue with new h(s, newGoal)
        std::vector<uint64_t> activeVertices;
        for (const auto& pair : vertexKeysInQueue_) {
            activeVertices.push_back(pair.first);
        }

        priorityQueue_.clear();
        vertexKeysInQueue_.clear();

        for (uint64_t u : activeVertices) {
            Key newKey = calculateKey(u);
            insertQueue(u, newKey);
        }

        // Also ensure goal vertex is checked
        updateVertex(newGoal);
    }

    // ------------------------------------------------------------------------
    // Core LPA* Engine Methods
    // ------------------------------------------------------------------------

    /**
     * @brief Computes Key(s) = [min(g(s), rhs(s)) + h(s, s_goal), min(g(s), rhs(s))].
     */
    Key calculateKey(uint64_t u) const {
        double minVal = std::min(getG(u), getRhs(u));
        if (minVal >= INF) {
            return Key(INF, INF);
        }
        double hVal = heuristic_.compute(u, problem_.goalState);
        return Key(minVal + hVal, minVal);
    }

    /**
     * @brief Updates rhs(u) and synchronizes vertex u in the priority queue.
     */
    void updateVertex(uint64_t u) {
        if (badStateSet_.find(u) != badStateSet_.end()) {
            // Bad states are strictly pruned and never enter queue
            removeFromQueue(u);
            g_[u] = INF;
            rhs_[u] = INF;
            return;
        }

        if (u != problem_.initialState) {
            double minRhs = INF;
            auto itPred = predecessors_.find(u);
            if (itPred != predecessors_.end()) {
                for (const auto& edge : itPred->second) {
                    if (!edge.available) continue;
                    if (badStateSet_.find(edge.target) != badStateSet_.end()) continue;

                    double gPred = getG(edge.target);
                    if (gPred < INF) {
                        double effectiveCost = computeEffectiveEdgeCost(edge);
                        double candidate = gPred + effectiveCost;
                        if (candidate < minRhs) {
                            minRhs = candidate;
                        }
                    }
                }
            }
            rhs_[u] = minRhs;
        }

        removeFromQueue(u);

        double gVal = getG(u);
        double rhsVal = getRhs(u);
        if (std::abs(gVal - rhsVal) > EPSILON) {
            Key k = calculateKey(u);
            insertQueue(u, k);
        }
    }

    /**
     * @brief LPA* main search loop.
     */
    void computeShortestPath() {
        while (!priorityQueue_.empty()) {
            auto topPair = *priorityQueue_.begin();
            Key topKey = topPair.first;
            uint64_t u = topPair.second;

            Key goalKey = calculateKey(problem_.goalState);
            double gGoal = getG(problem_.goalState);
            double rhsGoal = getRhs(problem_.goalState);

            // Termination condition: topKey >= goalKey AND goal is locally consistent
            if (topKey >= goalKey && std::abs(gGoal - rhsGoal) <= EPSILON) {
                break;
            }

            priorityQueue_.erase(priorityQueue_.begin());
            vertexKeysInQueue_.erase(u);
            exploredCount_++;

            double gVal = getG(u);
            double rhsVal = getRhs(u);

            if (gVal > rhsVal) {
                // Overconsistent: relax vertex to optimal rhs value
                g_[u] = rhsVal;
                if (recordSteps_) {
                    stepHistory_.push_back({exploredCount_, u, "Overconsistent", rhsVal, rhsVal, topKey, priorityQueue_.size()});
                }

                auto itSucc = successors_.find(u);
                if (itSucc != successors_.end()) {
                    for (const auto& edge : itSucc->second) {
                        updateVertex(edge.target);
                    }
                }
            } else {
                // Underconsistent: edge cost increased, set g to INF and re-evaluate
                g_[u] = INF;
                if (recordSteps_) {
                    stepHistory_.push_back({exploredCount_, u, "Underconsistent", INF, rhsVal, topKey, priorityQueue_.size()});
                }

                updateVertex(u);
                auto itSucc = successors_.find(u);
                if (itSucc != successors_.end()) {
                    for (const auto& edge : itSucc->second) {
                        updateVertex(edge.target);
                    }
                }
            }
        }
    }

    /**
     * @brief Extracts optimal path from start to goal.
     */
    PlanningResult extractResult() const {
        PlanningResult result;
        result.success = false;
        result.totalCost = 0.0;
        result.safetyScore = INF;
        result.cumulativeReliability = 1.0;
        result.exploredStates = exploredCount_;
        result.queueOperations = queueOpsCount_;

        double gGoal = getG(problem_.goalState);
        if (gGoal >= INF || badStateSet_.find(problem_.goalState) != badStateSet_.end() ||
            badStateSet_.find(problem_.initialState) != badStateSet_.end()) 
        {
            return result;
        }

        // Trace backward from goal to start
        std::vector<uint64_t> revStates;
        std::vector<uint64_t> revTransitions;

        uint64_t curr = problem_.goalState;
        revStates.push_back(curr);
        std::unordered_set<uint64_t> visited;
        visited.insert(curr);

        while (curr != problem_.initialState) {
            uint64_t bestPred = 0;
            uint64_t bestTransId = 0;
            double bestCostSum = INF;
            double chosenTransCost = 0.0;
            double chosenTransRel = 1.0;

            auto itPred = predecessors_.find(curr);
            if (itPred == predecessors_.end()) break;

            for (const auto& edge : itPred->second) {
                if (!edge.available) continue;
                if (badStateSet_.find(edge.target) != badStateSet_.end()) continue;

                double gPred = getG(edge.target);
                if (gPred < INF) {
                    double effectiveCost = computeEffectiveEdgeCost(edge);
                    double candidate = gPred + effectiveCost;
                    if (candidate < bestCostSum) {
                        bestCostSum = candidate;
                        bestPred = edge.target;
                        bestTransId = edge.transitionId;
                        chosenTransCost = edge.cost;
                        chosenTransRel = edge.reliability;
                    }
                }
            }

            if (bestCostSum >= INF || visited.find(bestPred) != visited.end()) {
                // No valid predecessor or cycle detected
                break;
            }

            revTransitions.push_back(bestTransId);
            revStates.push_back(bestPred);
            visited.insert(bestPred);
            result.totalCost += chosenTransCost;
            result.cumulativeReliability *= chosenTransRel;
            curr = bestPred;
        }

        if (curr != problem_.initialState) {
            result.success = false;
            return result;
        }

        result.success = true;
        std::reverse(revStates.begin(), revStates.end());
        std::reverse(revTransitions.begin(), revTransitions.end());
        result.statePath = revStates;
        result.transitionPath = revTransitions;

        // Safety metrics
        result.safetyScore = Geometry::computePathSafetyScore(result.statePath, problem_.badStates, embeddings_);
        result.averageSafetyDistance = Geometry::computeAverageSafetyDistance(result.statePath, problem_.badStates, embeddings_);

        // Objective Score: Score(P) = alpha*G - beta*C + gamma*D + delta*R
        double G = result.success ? 1.0 : 0.0;
        double C = result.totalCost;
        double D = (result.safetyScore < INF) ? result.safetyScore : 10.0;
        double R = result.cumulativeReliability;
        result.objectiveScore = (problem_.alpha * G) - (problem_.beta * C) + (problem_.gamma * D) + (problem_.delta * R);

        return result;
    }

    // ------------------------------------------------------------------------
    // Getters & Helpers
    // ------------------------------------------------------------------------
    double getG(uint64_t u) const {
        auto it = g_.find(u);
        return (it != g_.end()) ? it->second : INF;
    }

    double getRhs(uint64_t u) const {
        auto it = rhs_.find(u);
        return (it != rhs_.end()) ? it->second : INF;
    }

    const PlanningProblem& getProblem() const { return problem_; }
    const std::unordered_map<uint64_t, State>& getStates() const { return statesMap_; }
    const std::unordered_map<uint64_t, Transition>& getTransitions() const { return transitionsMap_; }
    const std::unordered_set<uint64_t>& getBadStates() const { return badStateSet_; }
    const std::vector<StepSnapshot>& getStepHistory() const { return stepHistory_; }
    void setRecordSteps(bool enable) { recordSteps_ = enable; }
    void setMultiObjectiveWeights(double safetyMargin, double safetyWeight, double reliabilityWeight) {
        safetyMargin_ = safetyMargin;
        safetyWeight_ = safetyWeight;
        reliabilityWeight_ = reliabilityWeight;
    }

private:
    void insertQueue(uint64_t u, const Key& key) {
        priorityQueue_.insert({key, u});
        vertexKeysInQueue_[u] = key;
        queueOpsCount_++;
    }

    void removeFromQueue(uint64_t u) {
        auto it = vertexKeysInQueue_.find(u);
        if (it != vertexKeysInQueue_.end()) {
            priorityQueue_.erase({it->second, u});
            vertexKeysInQueue_.erase(it);
            queueOpsCount_++;
        }
    }

    double computeEffectiveEdgeCost(const EdgeInfo& edge) const {
        double cost = edge.cost;
        if (safetyWeight_ > 0.0 && !badStateSet_.empty()) {
            auto itTo = embeddings_.find(edge.target);
            if (itTo != embeddings_.end()) {
                double d = Geometry::distanceToBadStates(itTo->second, problem_.badStates, embeddings_);
                if (d < safetyMargin_) {
                    cost += safetyWeight_ * (safetyMargin_ - d);
                }
            }
        }
        if (reliabilityWeight_ > 0.0) {
            cost += reliabilityWeight_ * (1.0 - edge.reliability);
        }
        return cost;
    }
};

} // namespace SafePlanner
