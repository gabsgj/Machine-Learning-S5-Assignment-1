#include "DStarLite.h"
#include <algorithm>
#include <iostream>

DStarLite::DStarLite(double c_w, double s_w, double r_w)
    : costWeight(c_w), safetyWeight(s_w), reliabilityWeight(r_w), k_m(0.0) {}

double DStarLite::euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    for (size_t i = 0; i < a.size() && i < b.size(); ++i) {
        double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double DStarLite::heuristic(uint64_t a, uint64_t b) {
    if (states.find(a) == states.end() || states.find(b) == states.end()) return INF;
    return euclideanDistance(states[a].embedding, states[b].embedding);
}

void DStarLite::precomputeSafetyDistances() {
    safety_distances.clear();
    double max_dist = 0.0;
    
    if (bad_states.empty()) {
        for (const auto& pair : states) safety_distances[pair.first] = 0.0;
        return;
    }

    for (const auto& pair : states) {
        uint64_t u = pair.first;
        if (bad_states.count(u)) {
            safety_distances[u] = 0.0;
            continue;
        }
        double min_dist = INF;
        for (uint64_t b : bad_states) {
            double d = euclideanDistance(states[u].embedding, states[b].embedding);
            min_dist = std::min(min_dist, d);
        }
        safety_distances[u] = min_dist;
        max_dist = std::max(max_dist, min_dist);
    }
    
    // Invert the distance so closer to bad states is higher penalty
    for (auto& pair : safety_distances) {
        if (!bad_states.count(pair.first)) {
            pair.second = max_dist - pair.second;
        }
    }
}

double DStarLite::getCost(uint64_t u, uint64_t v) {
    if (bad_states.count(u) || bad_states.count(v)) return INF;
    
    double min_edge_cost = INF;
    for (const auto& t : adj_forward[u]) {
        if (t.to == v && t.available) {
            double c = costWeight * t.cost 
                     - reliabilityWeight * t.reliability 
                     + safetyWeight * safety_distances[v];
            c = std::max(0.001, c); // Ensure positive edge cost
            min_edge_cost = std::min(min_edge_cost, c);
        }
    }
    return min_edge_cost;
}

DStarLite::Key DStarLite::calculateKey(uint64_t s) {
    double min_val = std::min(node_info[s].g, node_info[s].rhs);
    return {min_val + heuristic(s_start, s) + k_m, min_val};
}

void DStarLite::updateVertex(uint64_t u) {
    if (u != s_goal) {
        double min_rhs = INF;
        for (const auto& t : adj_forward[u]) {
            if (t.available) {
                double c = getCost(u, t.to);
                if (c != INF && node_info[t.to].g != INF) {
                    min_rhs = std::min(min_rhs, c + node_info[t.to].g);
                }
            }
        }
        node_info[u].rhs = min_rhs;
    }

    if (U_dict.find(u) != U_dict.end()) {
        U_dict.erase(u);
    }

    if (node_info[u].g != node_info[u].rhs) {
        Key key = calculateKey(u);
        U_dict[u] = key;
        U.push({key, u});
    }
}

void DStarLite::computeShortestPath() {
    while (!U.empty()) {
        QueueElement top = U.top();
        Key k_old = top.key;
        uint64_t u = top.id;
        
        // lazy deletion handling
        if (U_dict.find(u) == U_dict.end() || 
           (U_dict[u].first != k_old.first || U_dict[u].second != k_old.second)) {
            U.pop();
            continue;
        }

        Key k_new = calculateKey(u);
        if (k_old < k_new) {
            U.pop();
            U_dict[u] = k_new;
            U.push({k_new, u});
        } else if (node_info[u].g > node_info[u].rhs) {
            U.pop();
            node_info[u].g = node_info[u].rhs;
            U_dict.erase(u);
            for (const auto& t : adj_backward[u]) {
                if (t.available) updateVertex(t.from);
            }
        } else {
            U.pop();
            node_info[u].g = INF;
            U_dict.erase(u);
            updateVertex(u);
            for (const auto& t : adj_backward[u]) {
                if (t.available) updateVertex(t.from);
            }
        }
        
        if (U_dict.find(s_start) != U_dict.end() && 
            node_info[s_start].rhs == node_info[s_start].g) {
            // Need to check if k_old >= calc_key(s_start) 
            Key start_key = calculateKey(s_start);
            if (!(k_old < start_key)) {
                break;
            }
        }
    }
}

void DStarLite::initialize(const PlanningProblem& problem) {
    s_start = problem.initialState;
    s_goal = problem.goalState;
    k_m = 0.0;
    
    states.clear();
    for (const auto& s : problem.states) {
        states[s.id] = s;
        node_info[s.id] = DStarNode();
        node_info[s.id].id = s.id;
    }
    
    bad_states.clear();
    for (uint64_t b : problem.badStates) bad_states.insert(b);

    adj_forward.clear();
    adj_backward.clear();
    all_transitions.clear();
    
    for (const auto& t : problem.transitions) {
        adj_forward[t.from].push_back(t);
        adj_backward[t.to].push_back(t);
        all_transitions[t.id] = t;
    }

    precomputeSafetyDistances();

    while(!U.empty()) U.pop();
    U_dict.clear();

    node_info[s_goal].rhs = 0.0;
    Key key = calculateKey(s_goal);
    U_dict[s_goal] = key;
    U.push({key, s_goal});

    computeShortestPath();
}

PlanningResult DStarLite::plan(const PlanningProblem& problem) {
    initialize(problem);
    return extractPath();
}

double DStarLite::calculatePathSafety(const std::vector<uint64_t>& path) {
    if (bad_states.empty()) return 0.0;
    double min_d = INF;
    for (uint64_t u : path) {
        for (uint64_t b : bad_states) {
            min_d = std::min(min_d, euclideanDistance(states[u].embedding, states[b].embedding));
        }
    }
    return min_d == INF ? 0.0 : min_d;
}

PlanningResult DStarLite::extractPath() {
    PlanningResult res;
    res.success = false;
    res.totalCost = 0.0;
    res.safetyScore = 0.0;
    
    if (node_info[s_start].g == INF) {
        return res; // No path found
    }

    uint64_t curr = s_start;
    res.statePath.push_back(curr);

    while (curr != s_goal) {
        double min_cost = INF;
        uint64_t next_state = curr;
        uint64_t best_trans_id = 0;
        double best_base_cost = 0;
        
        for (const auto& t : adj_forward[curr]) {
            if (t.available) {
                double c = getCost(curr, t.to);
                if (c != INF && node_info[t.to].g != INF) {
                    if (c + node_info[t.to].g < min_cost) {
                        min_cost = c + node_info[t.to].g;
                        next_state = t.to;
                        best_trans_id = t.id;
                        best_base_cost = t.cost;
                    }
                }
            }
        }
        
        if (next_state == curr) break; // Stuck
        
        res.statePath.push_back(next_state);
        res.transitionPath.push_back(best_trans_id);
        res.totalCost += best_base_cost;
        curr = next_state;
    }

    if (curr == s_goal) {
        res.success = true;
        res.safetyScore = calculatePathSafety(res.statePath);
    }
    return res;
}

// Dynamic Updates
void DStarLite::updateTransition(uint64_t from, uint64_t to, bool available) {
    // Find transition in forward
    for (auto& t : adj_forward[from]) {
        if (t.to == to) {
            t.available = available;
        }
    }
    // Find transition in backward
    for (auto& t : adj_backward[to]) {
        if (t.from == from) {
            t.available = available;
        }
    }
    // Update vertex because edge cost changed
    updateVertex(from);
}

void DStarLite::addTransition(const Transition& t) {
    adj_forward[t.from].push_back(t);
    adj_backward[t.to].push_back(t);
    all_transitions[t.id] = t;
    updateVertex(t.from);
}

void DStarLite::updateGoal(uint64_t newGoal) {
    s_goal = newGoal;
    for (auto& pair : node_info) {
        pair.second.g = INF;
        pair.second.rhs = INF;
    }
    while(!U.empty()) U.pop();
    U_dict.clear();
    
    node_info[s_goal].rhs = 0.0;
    Key key = calculateKey(s_goal);
    U_dict[s_goal] = key;
    U.push({key, s_goal});
}

PlanningResult DStarLite::replan(uint64_t currentStart) {
    s_start = currentStart;
    computeShortestPath();
    return extractPath();
}
