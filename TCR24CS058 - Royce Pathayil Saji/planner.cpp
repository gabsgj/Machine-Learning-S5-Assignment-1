#include "planner.hpp"
#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>
#include <algorithm>

SafePlanner::SafePlanner() : alpha(1.0), beta(1.0), gamma(1.0) {}

void SafePlanner::setAlpha(double a) { alpha = a; }
void SafePlanner::setBeta(double b) { beta = b; }
void SafePlanner::setGamma(double g) { gamma = g; }

double SafePlanner::getDistance(const std::vector<double>& a, const std::vector<double>& b) {
    if (a.empty() || b.empty()) return 0.0;
    double sum = 0.0;
    size_t dim = std::min(a.size(), b.size());
    for (size_t i = 0; i < dim; ++i) {
        double diff = a[i] - b[i];
        sum += diff * diff;
    }
    return std::sqrt(sum);
}

double SafePlanner::getSafety(uint64_t stateId, const PlanningProblem& problem) {
    if (problem.badStates.empty()) return std::numeric_limits<double>::infinity();
    
    std::unordered_map<uint64_t, State> stateMap;
    for (const auto& s : problem.states) {
        stateMap[s.id] = s;
    }
    
    if (std::find(problem.badStates.begin(), problem.badStates.end(), stateId) != problem.badStates.end()) {
        return 0.0;
    }
    
    auto it = stateMap.find(stateId);
    if (it == stateMap.end()) return 0.0;
    
    double minDist = std::numeric_limits<double>::infinity();
    for (uint64_t badId : problem.badStates) {
        auto badIt = stateMap.find(badId);
        if (badIt != stateMap.end()) {
            double d = getDistance(it->second.embedding, badIt->second.embedding);
            if (d < minDist) {
                minDist = d;
            }
        }
    }
    return minDist;
}

struct AStarNode {
    uint64_t stateId;
    double f;
    double g;
    bool operator>(const AStarNode& other) const {
        return f > other.f;
    }
};

PlanningResult SafePlanner::plan(const PlanningProblem& problem) {
    PlanningResult result;
    result.success = false;
    result.totalCost = 0.0;
    result.safetyScore = 0.0;
    
    std::unordered_map<uint64_t, State> stateMap;
    for (const auto& s : problem.states) {
        stateMap[s.id] = s;
    }
    
    std::unordered_set<uint64_t> badSet(problem.badStates.begin(), problem.badStates.end());
    
    if (badSet.find(problem.initialState) != badSet.end() || badSet.find(problem.goalState) != badSet.end()) {
        return result;
    }
    
    std::unordered_map<uint64_t, std::vector<Transition>> adj;
    for (const auto& t : problem.transitions) {
        if (t.available) {
            adj[t.from].push_back(t);
        }
    }
    
    std::priority_queue<AStarNode, std::vector<AStarNode>, std::greater<AStarNode>> openList;
    std::unordered_map<uint64_t, double> gScore;
    std::unordered_map<uint64_t, uint64_t> cameFrom;
    std::unordered_map<uint64_t, Transition> edgeTo;
    
    gScore[problem.initialState] = 0.0;
    openList.push({problem.initialState, 0.0, 0.0});
    
    while (!openList.empty()) {
        AStarNode current = openList.top();
        openList.pop();
        
        if (current.stateId == problem.goalState) {
            result.success = true;
            break;
        }
        
        if (current.g > gScore[current.stateId]) continue;
        
        for (const auto& transition : adj[current.stateId]) {
            uint64_t neighbor = transition.to;
            if (badSet.find(neighbor) != badSet.end()) continue;
            
            double safety = getSafety(neighbor, problem);
            double penalty = 0.0;
            if (safety > 0 && safety < std::numeric_limits<double>::infinity()) {
                penalty = gamma * (1.0 / safety);
            }
            
            double tentativeG = current.g + transition.cost + penalty;
            
            if (gScore.find(neighbor) == gScore.end() || tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = current.stateId;
                edgeTo[neighbor] = transition;
                gScore[neighbor] = tentativeG;
                
                double h = 0.0;
                if (stateMap.find(neighbor) != stateMap.end() && stateMap.find(problem.goalState) != stateMap.end()) {
                    h = getDistance(stateMap[neighbor].embedding, stateMap[problem.goalState].embedding);
                }
                openList.push({neighbor, tentativeG + h, tentativeG});
            }
        }
    }
    
    if (result.success) {
        uint64_t curr = problem.goalState;
        std::vector<uint64_t> path;
        std::vector<uint64_t> tPath;
        
        path.push_back(curr);
        while (curr != problem.initialState) {
            Transition t = edgeTo[curr];
            tPath.push_back(t.id);
            curr = cameFrom[curr];
            path.push_back(curr);
            result.totalCost += t.cost;
        }
        
        std::reverse(path.begin(), path.end());
        std::reverse(tPath.begin(), tPath.end());
        
        result.statePath = path;
        result.transitionPath = tPath;
        
        double minSafety = std::numeric_limits<double>::infinity();
        for (uint64_t s : path) {
            double safe = getSafety(s, problem);
            if (safe < minSafety) {
                minSafety = safe;
            }
        }
        result.safetyScore = minSafety;
    }
    
    return result;
}
