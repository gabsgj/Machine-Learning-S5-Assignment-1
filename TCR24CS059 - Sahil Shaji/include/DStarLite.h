#pragma once

#include "Planner.h"
#include "Types.h"
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <limits>
#include <cmath>
#include <iostream>

const double INF = std::numeric_limits<double>::infinity();

struct DStarNode {
    uint64_t id;
    double g;
    double rhs;
    DStarNode() : id(0), g(INF), rhs(INF) {}
};

class DStarLite : public Planner {
public:
    // Weights for the objective function: alpha*G - beta*C + gamma*D + delta*R
    // Since D* Lite requires positive edge weights, we compute edge cost as:
    // cost_weight * T.cost - reliability_weight * T.reliability + safety_penalty_weight * (max_dist - dist_to_bad)
    DStarLite(double cost_weight = 1.0, double safety_weight = 2.0, double reliability_weight = 0.5);
    
    // Main interface
    PlanningResult plan(const PlanningProblem& problem) override;

    // Methods for dynamic environment testing
    void initialize(const PlanningProblem& problem);
    void updateTransition(uint64_t from, uint64_t to, bool available);
    void addTransition(const Transition& t);
    void updateGoal(uint64_t newGoal);
    PlanningResult replan(uint64_t currentStart);

private:
    double costWeight;
    double safetyWeight;
    double reliabilityWeight;

    uint64_t s_start;
    uint64_t s_goal;
    double k_m;

    std::unordered_map<uint64_t, State> states;
    std::unordered_map<uint64_t, std::vector<Transition>> adj_forward;
    std::unordered_map<uint64_t, std::vector<Transition>> adj_backward;
    
    std::unordered_map<uint64_t, DStarNode> node_info;
    std::unordered_map<uint64_t, double> safety_distances;
    std::unordered_set<uint64_t> bad_states;

    std::unordered_map<uint64_t, Transition> all_transitions;

    // U represents the priority queue
    using Key = std::pair<double, double>;
    struct QueueElement {
        Key key;
        uint64_t id;
        bool operator>(const QueueElement& other) const {
            if (key.first == other.key.first) {
                return key.second > other.key.second;
            }
            return key.first > other.key.first;
        }
    };

    std::priority_queue<QueueElement, std::vector<QueueElement>, std::greater<QueueElement>> U;
    std::unordered_map<uint64_t, Key> U_dict;

    Key calculateKey(uint64_t s);
    void updateVertex(uint64_t u);
    void computeShortestPath();
    
    double heuristic(uint64_t a, uint64_t b);
    double getCost(uint64_t u, uint64_t v);
    void precomputeSafetyDistances();
    double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b);
    
    PlanningResult extractPath();
    double calculatePathSafety(const std::vector<uint64_t>& path);
};
