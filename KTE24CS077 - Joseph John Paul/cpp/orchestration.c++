#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <limits>
#include <cstdint>
#include <algorithm>

// --- Problem Specifications & Interfaces ---

class State {
public:
    uint64_t id;
    std::vector<double> embedding;
};

class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

class PlanningResult {
public:
    bool success = false;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

// --- LPA* (Lifelong Planning A*) Implementation ---

class LPAPlanner : public Planner {
private:
    struct Key {
        double k1;
        double k2;

        bool operator<(const Key& other) const {
            if (std::abs(k1 - other.k1) > 1e-9) return k1 < other.k1;
            return k2 < other.k2;
        }
        bool operator>(const Key& other) const {
            if (std::abs(k1 - other.k1) > 1e-9) return k1 > other.k1;
            return k2 > other.k2;
        }
        bool operator==(const Key& other) const {
            return std::abs(k1 - other.k1) <= 1e-9 && std::abs(k2 - other.k2) <= 1e-9;
        }
        bool operator<=(const Key& other) const {
            return *this < other || *this == other;
        }
    };

    struct Edge {
        uint64_t transId;
        uint64_t to;
        double cost;
        bool available;
    };

    const double INF = std::numeric_limits<double>::infinity();
    double alpha = 2.0; // Trade-off weight between Euclidean distance cost and safety penalty

    std::unordered_map<uint64_t, State> statesMap;
    std::unordered_set<uint64_t> badStatesSet;
    std::unordered_map<uint64_t, std::vector<Edge>> succs;
    std::unordered_map<uint64_t, std::vector<Edge>> preds;
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;
    std::unordered_map<uint64_t, double> safetyDist;

    uint64_t s_start;
    uint64_t s_goal;

    // Priority Queue implementation with decrease-key support
    std::vector<std::pair<Key, uint64_t>> openList;

    double euclideanDist(const std::vector<double>& a, const std::vector<double>& b) {
        double sum = 0.0;
        for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }
        return std::sqrt(sum);
    }

    double computeMinBadStateDist(uint64_t u) {
        if (badStatesSet.empty() || badStatesSet.count(u)) return 0.0;
        double minDist = INF;
        for (uint64_t bId : badStatesSet) {
            minDist = std::min(minDist, euclideanDist(statesMap[u].embedding, statesMap[bId].embedding));
        }
        return minDist;
    }

    double heuristic(uint64_t u) {
        if (!statesMap.count(u) || !statesMap.count(s_goal)) return 0.0;
        return euclideanDist(statesMap[u].embedding, statesMap[s_goal].embedding);
    }

    Key calculateKey(uint64_t u) {
        double min_val = std::min(g[u], rhs[u]);
        return {min_val + heuristic(u), min_val};
    }

    void openListInsert(uint64_t u, Key k) {
        openList.push_back({k, u});
    }

    void openListRemove(uint64_t u) {
        openList.erase(std::remove_if(openList.begin(), openList.end(),
            [u](const std::pair<Key, uint64_t>& item) { return item.second == u; }), openList.end());
    }

    bool openListContains(uint64_t u) {
        for (const auto& item : openList) {
            if (item.second == u) return true;
        }
        return false;
    }

    Key openListTopKey() {
        if (openList.empty()) return {INF, INF};
        auto minIt = std::min_element(openList.begin(), openList.end(),
            [](const std::pair<Key, uint64_t>& a, const std::pair<Key, uint64_t>& b) { return a.first < b.first; });
        return minIt->first;
    }

    uint64_t openListPop() {
        auto minIt = std::min_element(openList.begin(), openList.end(),
            [](const std::pair<Key, uint64_t>& a, const std::pair<Key, uint64_t>& b) { return a.first < b.first; });
        uint64_t node = minIt->second;
        openList.erase(minIt);
        return node;
    }

    double getEffectiveCost(const Edge& e) {
        if (!e.available || badStatesSet.count(e.to)) return INF;
        // Safety-augmented edge cost: Higher penalty when closest to bad states
        double safetyFactor = (safetyDist[e.to] > 0.0) ? (alpha / safetyDist[e.to]) : 0.0;
        return e.cost + safetyFactor;
    }

    void updateVertex(uint64_t u) {
        if (u != s_start) {
            rhs[u] = INF;
            for (const auto& p : preds[u]) {
                double c = getEffectiveCost(p);
                if (c < INF && g[p.to] < INF) {
                    rhs[u] = std::min(rhs[u], g[p.to] + c);
                }
            }
        }
        if (openListContains(u)) {
            openListRemove(u);
        }
        if (std::abs(g[u] - rhs[u]) > 1e-9) {
            openListInsert(u, calculateKey(u));
        }
    }

    void computeShortestPath() {
        while (!openList.empty() && (openListTopKey() < calculateKey(s_goal) || rhs[s_goal] != g[s_goal])) {
            uint64_t u = openListPop();
            if (g[u] > rhs[u]) {
                g[u] = rhs[u];
                for (const auto& s : succs[u]) {
                    updateVertex(s.to);
                }
            } else {
                g[u] = INF;
                updateVertex(u);
                for (const auto& s : succs[u]) {
                    updateVertex(s.to);
                }
            }
        }
    }

public:
    PlanningResult plan(const PlanningProblem& problem) override {
        // Initialization
        statesMap.clear();
        badStatesSet.clear();
        succs.clear();
        preds.clear();
        g.clear();
        rhs.clear();
        safetyDist.clear();
        openList.clear();

        s_start = problem.initialState;
        s_goal = problem.goalState;

        for (const auto& s : problem.states) statesMap[s.id] = s;
        for (uint64_t b : problem.badStates) badStatesSet.insert(b);

        for (const auto& s : problem.states) {
            g[s.id] = INF;
            rhs[s.id] = INF;
            safetyDist[s.id] = computeMinBadStateDist(s.id);
        }

        for (const auto& t : problem.transitions) {
            succs[t.from].push_back({t.id, t.to, t.cost, t.available});
            preds[t.to].push_back({t.id, t.from, t.cost, t.available});
        }

        rhs[s_start] = 0.0;
        openListInsert(s_start, calculateKey(s_start));

        computeShortestPath();

        PlanningResult result;
        if (g[s_goal] >= INF) {
            result.success = false;
            return result;
        }

        // Path Reconstruction
        uint64_t curr = s_goal;
        result.statePath.push_back(curr);
        double minSafety = safetyDist[curr];

        while (curr != s_start) {
            uint64_t bestPrev = curr;
            uint64_t bestTrans = 0;
            double minCost = INF;

            for (const auto& p : preds[curr]) {
                double c = getEffectiveCost(p);
                if (c < INF && g[p.to] < INF) {
                    if (g[p.to] + c < minCost) {
                        minCost = g[p.to] + c;
                        bestPrev = p.to;
                        bestTrans = p.transId;
                    }
                }
            }

            if (bestPrev == curr) {
                result.success = false;
                return result;
            }

            result.totalCost += minCost - g[bestPrev];
            result.transitionPath.push_back(bestTrans);
            curr = bestPrev;
            result.statePath.push_back(curr);
            minSafety = std::min(minSafety, safetyDist[curr]);
        }

        std::reverse(result.statePath.begin(), result.statePath.end());
        std::reverse(result.transitionPath.begin(), result.transitionPath.end());

        result.success = true;
        result.safetyScore = minSafety;
        return result;
    }
};

// --- Test Verification Driver ---

int main() {
    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {3}; // State 3 is BAD

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 1.0}},
        {3, {1.0, 0.0}}, // Bad state
        {4, {2.0, 0.0}}
    };

    // Paths: 1 -> 3 -> 4 (Bad) vs 1 -> 2 -> 4 (Safe)
    problem.transitions = {
        {101, 1, 3, 1.0, 1.0, 1.0, true},
        {102, 3, 4, 1.0, 1.0, 1.0, true},
        {103, 1, 2, 2.0, 1.0, 1.0, true},
        {104, 2, 4, 2.0, 1.0, 1.0, true}
    };

    LPAPlanner planner;
    PlanningResult res = planner.plan(problem);

    if (res.success) {
        std::cout << "Plan found successfully!\nPath: ";
        for (uint64_t sid : res.statePath) std::cout << sid << " ";
        std::cout << "\nTotal Cost: " << res.totalCost << "\nMin Safety Distance: " << res.safetyScore << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}
