#ifndef SAFE_SEMANTIC_PLANNER_HPP
#define SAFE_SEMANTIC_PLANNER_HPP

#include "Types.hpp"
#include "SafetyField.hpp"
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <chrono>
#include <cmath>
#include <limits>
#include <algorithm>
#include <memory>

/**
 * Key representation for LPA* priority queue:
 * k(s) = [k1, k2] = [min(g(s), rhs(s)) + h(s, s_goal), min(g(s), rhs(s))]
 */
struct LPAKey {
    double k1;
    double k2;

    bool operator<(const LPAKey& other) const {
        const double eps = 1e-9;
        if (std::abs(k1 - other.k1) > eps) {
            return k1 < other.k1;
        }
        if (std::abs(k2 - other.k2) > eps) {
            return k2 < other.k2;
        }
        return false;
    }

    bool operator<=(const LPAKey& other) const {
        return (*this < other) || (!(*this < other) && !(other < *this));
    }

    bool operator==(const LPAKey& other) const {
        const double eps = 1e-9;
        return (std::abs(k1 - other.k1) <= eps) && (std::abs(k2 - other.k2) <= eps);
    }
};

/**
 * High-performance, memory-efficient Safe Semantic Incremental Planner
 * implementing Lifelong Planning A* (LPA*) with Cartesian safety barrier fields.
 */
class SafeSemanticPlanner : public Planner {
private:
    uint64_t startId;
    uint64_t goalId;

    std::unordered_map<uint64_t, State> stateMap;
    std::unordered_set<uint64_t> badSet;
    
    // Adjacency graph: from state -> list of transition IDs
    std::unordered_map<uint64_t, std::vector<uint64_t>> outTransitions; // successors
    std::unordered_map<uint64_t, std::vector<uint64_t>> inTransitions;  // predecessors
    std::unordered_map<uint64_t, Transition> transitionMap;

    // LPA* Core Data Structures
    std::unordered_map<uint64_t, double> gValues;
    std::unordered_map<uint64_t, double> rhsValues;

    // Priority Queue: sorted by (LPAKey, nodeId) for exact O(log N) lookup and removal
    std::set<std::pair<LPAKey, uint64_t>> openList;
    std::unordered_map<uint64_t, LPAKey> nodeKeyInQueue;

    // Configuration
    Safety::SafetyFieldConfig safetyConfig;
    Safety::ObjectiveWeights objectiveWeights;
    double heuristicScale;

    // Diagnostic metrics
    size_t exploredStates;

public:
    SafeSemanticPlanner(
        Safety::SafetyFieldConfig safetyCfg = Safety::SafetyFieldConfig(),
        Safety::ObjectiveWeights objWeights = Safety::ObjectiveWeights(),
        double hScale = 1.0
    ) : startId(0), goalId(0), safetyConfig(safetyCfg), 
        objectiveWeights(objWeights), heuristicScale(hScale), exploredStates(0) {}

    void setSafetyConfig(const Safety::SafetyFieldConfig& cfg) { safetyConfig = cfg; }
    void setObjectiveWeights(const Safety::ObjectiveWeights& w) { objectiveWeights = w; }
    void setHeuristicScale(double scale) { heuristicScale = scale; }

    const std::unordered_map<uint64_t, State>& getStates() const { return stateMap; }
    const std::unordered_map<uint64_t, Transition>& getTransitions() const { return transitionMap; }
    const std::unordered_set<uint64_t>& getBadStates() const { return badSet; }
    uint64_t getStartId() const { return startId; }
    uint64_t getGoalId() const { return goalId; }

    double getG(uint64_t id) const {
        auto it = gValues.find(id);
        return it != gValues.end() ? it->second : std::numeric_limits<double>::infinity();
    }

    double getRhs(uint64_t id) const {
        auto it = rhsValues.find(id);
        return it != rhsValues.end() ? it->second : std::numeric_limits<double>::infinity();
    }

    /**
     * Initializes graph and data structures for a planning problem.
     */
    void initialize(const PlanningProblem& problem) {
        startId = problem.initialState;
        goalId = problem.goalState;

        stateMap.clear();
        for (const auto& s : problem.states) {
            stateMap[s.id] = s;
        }

        badSet.clear();
        for (uint64_t b : problem.badStates) {
            badSet.insert(b);
        }

        transitionMap.clear();
        outTransitions.clear();
        inTransitions.clear();
        for (const auto& t : problem.transitions) {
            transitionMap[t.id] = t;
            outTransitions[t.from].push_back(t.id);
            inTransitions[t.to].push_back(t.id);
        }

        // Initialize LPA* values
        gValues.clear();
        rhsValues.clear();
        openList.clear();
        nodeKeyInQueue.clear();
        exploredStates = 0;

        for (const auto& kv : stateMap) {
            gValues[kv.first] = std::numeric_limits<double>::infinity();
            rhsValues[kv.first] = std::numeric_limits<double>::infinity();
        }

        // rhs(start) = 0
        rhsValues[startId] = 0.0;
        insertOrUpdateInQueue(startId);
    }

    /**
     * Computes the admissible Euclidean heuristic in Cartesian space.
     */
    double heuristic(uint64_t fromId, uint64_t toId) const {
        auto itFrom = stateMap.find(fromId);
        auto itTo = stateMap.find(toId);
        if (itFrom == stateMap.end() || itTo == stateMap.end()) return 0.0;
        return heuristicScale * Safety::euclideanDistance(itFrom->second.embedding, itTo->second.embedding);
    }

    /**
     * Calculates the two-element priority key for node u.
     */
    LPAKey calculateKey(uint64_t u) const {
        double g = getG(u);
        double rhs = getRhs(u);
        double m = std::min(g, rhs);
        return { m + heuristic(u, goalId), m };
    }

    /**
     * Inserts, updates or removes a node from the LPA* priority queue.
     */
    void updateVertex(uint64_t u) {
        if (u != startId) {
            double minRhs = std::numeric_limits<double>::infinity();
            auto itIn = inTransitions.find(u);
            if (itIn != inTransitions.end()) {
                for (uint64_t transId : itIn->second) {
                    const auto& trans = transitionMap.at(transId);
                    double edgeCost = getTransitionCost(trans);
                    double predG = getG(trans.from);
                    if (predG < std::numeric_limits<double>::infinity() && 
                        edgeCost < std::numeric_limits<double>::infinity()) {
                        double candidate = predG + edgeCost;
                        if (candidate < minRhs) {
                            minRhs = candidate;
                        }
                    }
                }
            }
            rhsValues[u] = minRhs;
        }

        insertOrUpdateInQueue(u);
    }

    void insertOrUpdateInQueue(uint64_t u) {
        // If node was previously in queue, remove it
        auto itKey = nodeKeyInQueue.find(u);
        if (itKey != nodeKeyInQueue.end()) {
            openList.erase({ itKey->second, u });
            nodeKeyInQueue.erase(itKey);
        }

        // If inconsistent, insert with new key
        double g = getG(u);
        double rhs = getRhs(u);
        const double eps = 1e-9;
        if (std::abs(g - rhs) > eps) {
            LPAKey key = calculateKey(u);
            openList.insert({ key, u });
            nodeKeyInQueue[u] = key;
        }
    }

    double getTransitionCost(const Transition& trans) const {
        auto itTarget = stateMap.find(trans.to);
        if (itTarget == stateMap.end()) return std::numeric_limits<double>::infinity();
        return Safety::computeEffectiveTransitionCost(trans, itTarget->second, badSet, stateMap, safetyConfig);
    }

    /**
     * LPA* Main loop: computes shortest safe path by resolving inconsistent vertices.
     */
    void computeShortestPath() {
        const double eps = 1e-9;

        while (!openList.empty()) {
            auto topIter = openList.begin();
            LPAKey topKey = topIter->first;
            uint64_t u = topIter->second;

            LPAKey goalKey = calculateKey(goalId);

            if (!(topKey < goalKey) && std::abs(getRhs(goalId) - getG(goalId)) <= eps) {
                break;
            }

            openList.erase(topIter);
            nodeKeyInQueue.erase(u);
            exploredStates++;

            double g = getG(u);
            double rhs = getRhs(u);

            if (g > rhs) {
                // Overconsistent: decrease g to rhs
                gValues[u] = rhs;
                auto itOut = outTransitions.find(u);
                if (itOut != outTransitions.end()) {
                    for (uint64_t transId : itOut->second) {
                        const auto& trans = transitionMap.at(transId);
                        updateVertex(trans.to);
                    }
                }
            } else {
                // Underconsistent: reset g to infinity and update self + successors
                gValues[u] = std::numeric_limits<double>::infinity();
                updateVertex(u);
                auto itOut = outTransitions.find(u);
                if (itOut != outTransitions.end()) {
                    for (uint64_t transId : itOut->second) {
                        const auto& trans = transitionMap.at(transId);
                        updateVertex(trans.to);
                    }
                }
            }
        }
    }

    /**
     * Extracts the planned path from start to goal.
     */
    PlanningResult extractResult(double durationMicros) {
        PlanningResult result;
        result.executionTimeMicroseconds = durationMicros;
        result.exploredStatesCount = exploredStates;

        if (getG(goalId) >= std::numeric_limits<double>::infinity() / 2.0) {
            result.success = false;
            result.totalCost = 0.0;
            result.safetyScore = 0.0;
            result.minSafetyDistance = 0.0;
            result.cumulativeReliability = 0.0;
            result.multiObjectiveScore = Safety::evaluateMultiObjectiveScore(
                false, 0.0, 0.0, 0.0, objectiveWeights
            );
            return result;
        }

        // Reconstruct path backward from goal to start
        std::vector<uint64_t> revStates;
        std::vector<uint64_t> revTransitions;

        uint64_t current = goalId;
        revStates.push_back(current);

        double nominalCostSum = 0.0;
        double cumulativeReliability = 1.0;
        double minSafetyDist = std::numeric_limits<double>::infinity();

        std::unordered_set<uint64_t> visited;
        visited.insert(current);

        while (current != startId) {
            auto itIn = inTransitions.find(current);
            if (itIn == end(inTransitions) || itIn->second.empty()) {
                result.success = false;
                return result;
            }

            uint64_t bestPred = 0;
            uint64_t bestTransId = 0;
            double bestVal = std::numeric_limits<double>::infinity();
            const Transition* bestTransPtr = nullptr;

            for (uint64_t transId : itIn->second) {
                const auto& trans = transitionMap.at(transId);
                double edgeCost = getTransitionCost(trans);
                double predG = getG(trans.from);

                if (predG < std::numeric_limits<double>::infinity() && 
                    edgeCost < std::numeric_limits<double>::infinity()) {
                    double val = predG + edgeCost;
                    if (val < bestVal) {
                        bestVal = val;
                        bestPred = trans.from;
                        bestTransId = transId;
                        bestTransPtr = &trans;
                    }
                }
            }

            if (bestTransPtr == nullptr || visited.find(bestPred) != visited.end()) {
                // Cycle or no predecessor found
                result.success = false;
                return result;
            }

            nominalCostSum += bestTransPtr->cost;
            cumulativeReliability *= bestTransPtr->reliability;
            revTransitions.push_back(bestTransId);
            revStates.push_back(bestPred);
            visited.insert(bestPred);
            current = bestPred;
        }

        // Reverse to get start -> goal order
        std::reverse(revStates.begin(), revStates.end());
        std::reverse(revTransitions.begin(), revTransitions.end());

        result.success = true;
        result.statePath = std::move(revStates);
        result.transitionPath = std::move(revTransitions);
        result.totalCost = nominalCostSum;
        result.cumulativeReliability = cumulativeReliability;

        // Calculate minimum safety distance from any visited state to nearest bad state
        for (uint64_t sid : result.statePath) {
            auto itS = stateMap.find(sid);
            if (itS != stateMap.end()) {
                double d = Safety::minDistanceToBadStates(itS->second, badSet, stateMap);
                if (d < minSafetyDist) {
                    minSafetyDist = d;
                }
            }
        }
        if (minSafetyDist == std::numeric_limits<double>::infinity()) {
            minSafetyDist = 100.0;
        }

        result.minSafetyDistance = minSafetyDist;
        result.safetyScore = minSafetyDist; // Meets interface double safetyScore
        result.multiObjectiveScore = Safety::evaluateMultiObjectiveScore(
            true, result.totalCost, result.minSafetyDistance, 
            result.cumulativeReliability, objectiveWeights
        );

        return result;
    }

    /**
     * Standard Planner interface implementation.
     */
    PlanningResult plan(const PlanningProblem& problem) override {
        auto startTime = std::chrono::high_resolution_clock::now();
        initialize(problem);
        computeShortestPath();
        auto endTime = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double, std::nano>(endTime - startTime).count() / 1000.0;
        return extractResult(duration);
    }

    // =========================================================================
    // Dynamic Replanning Methods (Incremental updates without full rebuild)
    // =========================================================================

    /**
     * Updates availability or cost of an existing transition and updates LPA* queue.
     */
    void updateTransitionAvailability(uint64_t transId, bool available) {
        auto it = transitionMap.find(transId);
        if (it != transitionMap.end()) {
            it->second.available = available;
            updateVertex(it->second.to);
        }
    }

    void updateTransitionCost(uint64_t transId, double newCost) {
        auto it = transitionMap.find(transId);
        if (it != transitionMap.end()) {
            it->second.cost = newCost;
            updateVertex(it->second.to);
        }
    }

    /**
     * Dynamically adds a new transition to the graph.
     */
    void addDynamicTransition(const Transition& t) {
        transitionMap[t.id] = t;
        outTransitions[t.from].push_back(t.id);
        inTransitions[t.to].push_back(t.id);
        updateVertex(t.to);
    }

    /**
     * Dynamically removes a transition from the graph.
     */
    void removeDynamicTransition(uint64_t transId) {
        auto it = transitionMap.find(transId);
        if (it != transitionMap.end()) {
            uint64_t toNode = it->second.to;
            uint64_t fromNode = it->second.from;

            // Remove from outTransitions
            auto& outList = outTransitions[fromNode];
            outList.erase(std::remove(outList.begin(), outList.end(), transId), outList.end());

            // Remove from inTransitions
            auto& inList = inTransitions[toNode];
            inList.erase(std::remove(inList.begin(), inList.end(), transId), inList.end());

            transitionMap.erase(it);
            updateVertex(toNode);
        }
    }

    /**
     * Dynamically updates the set of bad states.
     */
    void setBadStates(const std::vector<uint64_t>& newBadStates) {
        badSet.clear();
        for (uint64_t b : newBadStates) {
            badSet.insert(b);
        }
        // Invalidate vertices affected by safety field changes
        for (const auto& kv : stateMap) {
            updateVertex(kv.first);
        }
    }

    /**
     * Dynamically updates the goal state and recalculates queue keys.
     */
    void setGoal(uint64_t newGoal) {
        if (goalId == newGoal) return;
        goalId = newGoal;

        // Re-key existing open list elements with updated heuristic to new goal
        std::set<std::pair<LPAKey, uint64_t>> newOpenList;
        std::unordered_map<uint64_t, LPAKey> newKeyMap;

        for (const auto& item : openList) {
            uint64_t u = item.second;
            LPAKey newKey = calculateKey(u);
            newOpenList.insert({ newKey, u });
            newKeyMap[u] = newKey;
        }

        openList = std::move(newOpenList);
        nodeKeyInQueue = std::move(newKeyMap);
    }

    /**
     * Performs incremental replanning and returns updated PlanningResult.
     */
    PlanningResult replan() {
        auto startTime = std::chrono::high_resolution_clock::now();
        size_t initialExplored = exploredStates;
        computeShortestPath();
        auto endTime = std::chrono::high_resolution_clock::now();
        double duration = std::chrono::duration<double, std::nano>(endTime - startTime).count() / 1000.0;

        PlanningResult res = extractResult(duration);
        res.exploredStatesCount = exploredStates - initialExplored;
        return res;
    }
};

#endif // SAFE_SEMANTIC_PLANNER_HPP
