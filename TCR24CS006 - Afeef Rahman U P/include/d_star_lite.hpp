#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "heuristic.hpp"
#include "lpa_star.hpp"

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <algorithm>
#include <chrono>

namespace SafePlanner {

/**
 * @brief D* Lite Planner (Backward Incremental Search).
 * 
 * Searches backwards from goal to start, making it optimal for moving agent scenarios.
 * Included for comparative benchmark against forward LPA*.
 */
class DStarLitePlanner : public Planner {
public:
    struct EdgeInfo {
        uint64_t transitionId;
        uint64_t target;
        double cost;
        double safety;
        double reliability;
        bool available;
    };

private:
    PlanningProblem problem_;
    std::unordered_map<uint64_t, State> statesMap_;
    std::unordered_map<uint64_t, Transition> transitionsMap_;
    std::unordered_set<uint64_t> badStateSet_;
    std::unordered_map<uint64_t, std::vector<double>> embeddings_;

    std::unordered_map<uint64_t, std::vector<EdgeInfo>> successors_;
    std::unordered_map<uint64_t, std::vector<EdgeInfo>> predecessors_;

    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;

    Heuristic heuristic_;
    std::set<std::pair<Key, uint64_t>> priorityQueue_;
    std::unordered_map<uint64_t, Key> vertexKeysInQueue_;

    double km_ = 0.0;
    uint64_t sLast_;
    size_t exploredCount_ = 0;

public:
    DStarLitePlanner() : sLast_(0) {}
    ~DStarLitePlanner() override = default;

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
        km_ = 0.0;
        sLast_ = problem.initialState;
        exploredCount_ = 0;

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
            successors_[t.from].push_back({t.id, t.to, t.cost, t.safety, t.reliability, t.available});
            predecessors_[t.to].push_back({t.id, t.from, t.cost, t.safety, t.reliability, t.available});
        }

        heuristic_.calibrate(problem.states, problem.transitions, problem.badStates);

        // In D* Lite, search runs backwards from goalState
        if (badStateSet_.find(problem.goalState) == badStateSet_.end()) {
            rhs_[problem.goalState] = 0.0;
            Key goalKey = calculateKey(problem.goalState);
            insertQueue(problem.goalState, goalKey);
        }
    }

    PlanningResult plan(const PlanningProblem& problem) override {
        auto startTime = std::chrono::high_resolution_clock::now();
        loadProblem(problem);
        computeShortestPath();
        auto endTime = std::chrono::high_resolution_clock::now();

        PlanningResult result = extractResult();
        result.planningTimeMicroseconds = std::chrono::duration<double, std::micro>(endTime - startTime).count();
        result.algorithmName = "D* Lite";
        return result;
    }

    Key calculateKey(uint64_t u) const {
        double minVal = std::min(getG(u), getRhs(u));
        if (minVal >= INF) {
            return Key(INF, INF);
        }
        // In D* Lite, heuristic is computed to the start state (or agent's current position)
        double hVal = heuristic_.compute(u, problem_.initialState);
        return Key(minVal + hVal + km_, minVal);
    }

    void updateVertex(uint64_t u) {
        if (badStateSet_.find(u) != badStateSet_.end()) {
            removeFromQueue(u);
            g_[u] = INF;
            rhs_[u] = INF;
            return;
        }

        if (u != problem_.goalState) {
            double minRhs = INF;
            auto itSucc = successors_.find(u);
            if (itSucc != successors_.end()) {
                for (const auto& edge : itSucc->second) {
                    if (!edge.available || badStateSet_.find(edge.target) != badStateSet_.end()) continue;
                    double gSucc = getG(edge.target);
                    if (gSucc < INF) {
                        double candidate = edge.cost + gSucc;
                        if (candidate < minRhs) {
                            minRhs = candidate;
                        }
                    }
                }
            }
            rhs_[u] = minRhs;
        }

        removeFromQueue(u);
        if (std::abs(getG(u) - getRhs(u)) > EPSILON) {
            insertQueue(u, calculateKey(u));
        }
    }

    void computeShortestPath() {
        while (!priorityQueue_.empty()) {
            auto topPair = *priorityQueue_.begin();
            Key kOld = topPair.first;
            uint64_t u = topPair.second;

            Key kNew = calculateKey(u);
            Key startKey = calculateKey(problem_.initialState);
            double gStart = getG(problem_.initialState);
            double rhsStart = getRhs(problem_.initialState);

            if (kOld < kNew) {
                // Key out of date, update in queue
                priorityQueue_.erase(priorityQueue_.begin());
                vertexKeysInQueue_.erase(u);
                insertQueue(u, kNew);
                continue;
            }

            if (kOld >= startKey && std::abs(gStart - rhsStart) <= EPSILON) {
                break;
            }

            priorityQueue_.erase(priorityQueue_.begin());
            vertexKeysInQueue_.erase(u);
            exploredCount_++;

            double gVal = getG(u);
            double rhsVal = getRhs(u);

            if (gVal > rhsVal) {
                g_[u] = rhsVal;
                auto itPred = predecessors_.find(u);
                if (itPred != predecessors_.end()) {
                    for (const auto& edge : itPred->second) {
                        updateVertex(edge.target);
                    }
                }
            } else {
                g_[u] = INF;
                updateVertex(u);
                auto itPred = predecessors_.find(u);
                if (itPred != predecessors_.end()) {
                    for (const auto& edge : itPred->second) {
                        updateVertex(edge.target);
                    }
                }
            }
        }
    }

    PlanningResult extractResult() const {
        PlanningResult result;
        result.success = false;
        result.totalCost = 0.0;
        result.safetyScore = INF;
        result.cumulativeReliability = 1.0;
        result.exploredStates = exploredCount_;

        double gStart = getG(problem_.initialState);
        if (gStart >= INF || badStateSet_.find(problem_.initialState) != badStateSet_.end() ||
            badStateSet_.find(problem_.goalState) != badStateSet_.end()) 
        {
            return result;
        }

        std::vector<uint64_t> statePath;
        std::vector<uint64_t> transitionPath;

        uint64_t curr = problem_.initialState;
        statePath.push_back(curr);
        std::unordered_set<uint64_t> visited;
        visited.insert(curr);

        while (curr != problem_.goalState) {
            uint64_t bestSucc = 0;
            uint64_t bestTransId = 0;
            double bestCostSum = INF;
            double chosenCost = 0.0;
            double chosenRel = 1.0;

            auto itSucc = successors_.find(curr);
            if (itSucc == successors_.end()) break;

            for (const auto& edge : itSucc->second) {
                if (!edge.available || badStateSet_.find(edge.target) != badStateSet_.end()) continue;
                double gSucc = getG(edge.target);
                if (gSucc < INF) {
                    double candidate = edge.cost + gSucc;
                    if (candidate < bestCostSum) {
                        bestCostSum = candidate;
                        bestSucc = edge.target;
                        bestTransId = edge.transitionId;
                        chosenCost = edge.cost;
                        chosenRel = edge.reliability;
                    }
                }
            }

            if (bestCostSum >= INF || visited.find(bestSucc) != visited.end()) {
                break;
            }

            transitionPath.push_back(bestTransId);
            statePath.push_back(bestSucc);
            visited.insert(bestSucc);
            result.totalCost += chosenCost;
            result.cumulativeReliability *= chosenRel;
            curr = bestSucc;
        }

        if (curr != problem_.goalState) {
            result.success = false;
            return result;
        }

        result.success = true;
        result.statePath = statePath;
        result.transitionPath = transitionPath;
        result.safetyScore = Geometry::computePathSafetyScore(result.statePath, problem_.badStates, embeddings_);
        result.averageSafetyDistance = Geometry::computeAverageSafetyDistance(result.statePath, problem_.badStates, embeddings_);

        double G = result.success ? 1.0 : 0.0;
        double C = result.totalCost;
        double D = (result.safetyScore < INF) ? result.safetyScore : 10.0;
        double R = result.cumulativeReliability;
        result.objectiveScore = (problem_.alpha * G) - (problem_.beta * C) + (problem_.gamma * D) + (problem_.delta * R);

        return result;
    }

    double getG(uint64_t u) const {
        auto it = g_.find(u);
        return (it != g_.end()) ? it->second : INF;
    }

    double getRhs(uint64_t u) const {
        auto it = rhs_.find(u);
        return (it != rhs_.end()) ? it->second : INF;
    }

private:
    void insertQueue(uint64_t u, const Key& key) {
        priorityQueue_.insert({key, u});
        vertexKeysInQueue_[u] = key;
    }

    void removeFromQueue(uint64_t u) {
        auto it = vertexKeysInQueue_.find(u);
        if (it != vertexKeysInQueue_.end()) {
            priorityQueue_.erase({it->second, u});
            vertexKeysInQueue_.erase(it);
        }
    }
};

} // namespace SafePlanner
