#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <set>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <cstdint>
#include <limits>

using namespace std;

class State
{
public:
    uint64_t id;
    vector<double> embedding;
};

class Transition
{
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

class PlanningProblem
{
public:
    uint64_t initialState;
    uint64_t goalState;
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

class PlanningResult
{
public:
    bool success;
    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
};

class Planner
{
public:
    virtual PlanningResult plan(const PlanningProblem &problem) = 0;
};

class LPAStarPlanner : public Planner
{
private:
    unordered_map<uint64_t, double> g;
    unordered_map<uint64_t, double> rhs;
    unordered_map<uint64_t, State> stateMap;
    unordered_map<uint64_t, vector<Transition>> adj;
    unordered_map<uint64_t, vector<Transition>> pred;
    unordered_set<uint64_t> badSet;
    set<pair<pair<double, double>, uint64_t>> U;
    uint64_t start, goal;

    double calcHeuristic(uint64_t u, uint64_t v)
    {
        if (!stateMap.count(u) || !stateMap.count(v))
            return 0.0;
        double dist = 0.0;
        for (size_t i = 0; i < stateMap[u].embedding.size(); ++i)
        {
            double diff = stateMap[u].embedding[i] - stateMap[v].embedding[i];
            dist += diff * diff;
        }
        return sqrt(dist);
    }

    double calcSafetyPenalty(uint64_t u)
    {
        double minDistance = numeric_limits<double>::infinity();
        if (!stateMap.count(u))
            return 0.0;
        for (uint64_t b : badSet)
        {
            double d = calcHeuristic(u, b);
            if (d < minDistance)
                minDistance = d;
        }
        if (minDistance == 0)
            return 10000.0;
        return 1.0 / (minDistance + 0.1);
    }

    pair<double, double> calcKey(uint64_t u)
    {
        double k2 = min(g[u], rhs[u]);
        double k1 = k2 + calcHeuristic(u, goal);
        return {k1, k2};
    }

    void updateVertex(uint64_t u)
    {   
        if (u != start)
        {
            rhs[u] = numeric_limits<double>::infinity();
            for (const auto &t : pred[u])
            {
                if (t.available && !badSet.count(t.from) && !badSet.count(t.to))
                {
                    double edgeWeight = t.cost + calcSafetyPenalty(t.to);
                    rhs[u] = min(rhs[u], g[t.from] + edgeWeight);
                }
            }
        }

        auto it = U.begin();
        while (it != U.end())
        {
            if (it->second == u)
            {
                U.erase(it);
                break;
            }
            ++it;
        }

        if (g[u] != rhs[u])
        {
            U.insert({calcKey(u), u});
        }
    }

    void computeShortestPath()
    {
        while (!U.empty() && (U.begin()->first < calcKey(goal) || rhs[goal] != g[goal]))
        { 
            uint64_t u = U.begin()->second;
            U.erase(U.begin());

            if (g[u] > rhs[u])
            {
                g[u] = rhs[u];
                for (const auto &t : adj[u])
                {
                    updateVertex(t.to);
                }
            }
            else
            {
                g[u] = numeric_limits<double>::infinity();
                for (const auto &t : adj[u])
                {
                    updateVertex(t.to);
                }
                updateVertex(u);
            }
        }
    }

public:
    PlanningResult plan(const PlanningProblem &problem) override
    {
        start = problem.initialState;
        goal = problem.goalState;
        badSet = unordered_set<uint64_t>(problem.badStates.begin(), problem.badStates.end());

        for (const auto &s : problem.states)
        {
            stateMap[s.id] = s;
            g[s.id] = numeric_limits<double>::infinity();
            rhs[s.id] = numeric_limits<double>::infinity();
        }

        for (const auto &t : problem.transitions)
        {
            adj[t.from].push_back(t);
            pred[t.to].push_back(t);
        }

        rhs[start] = 0.0;
        U.insert({calcKey(start), start});

        computeShortestPath();

        PlanningResult result;
        result.success = (g[goal] != numeric_limits<double>::infinity());
        result.totalCost = 0.0;
        result.safetyScore = 0.0;

        if (result.success)
        {
            uint64_t curr = goal;
            result.statePath.push_back(curr);
            double minSafety = numeric_limits<double>::infinity();

            while (curr != start)
            {
                uint64_t bestPrev = curr;
                double minCost = numeric_limits<double>::infinity();
                uint64_t bestTransId = 0;
                double transCost = 0;

                for (const auto &t : pred[curr])
                {
                    if (t.available)
                    {
                        double c = g[t.from] + t.cost + calcSafetyPenalty(curr);
                        if (c < minCost)
                        {
                            minCost = c;
                            bestPrev = t.from;
                            bestTransId = t.id;
                            transCost = t.cost;
                        }
                    }
                }

                result.transitionPath.push_back(bestTransId);
                result.totalCost += transCost;
                curr = bestPrev;
                result.statePath.push_back(curr);

                double sDist = numeric_limits<double>::infinity();
                for (uint64_t b : badSet)
                {
                    sDist = min(sDist, calcHeuristic(curr, b));
                }
                minSafety = min(minSafety, sDist);
            }

            result.safetyScore = minSafety;
            reverse(result.statePath.begin(), result.statePath.end());
            reverse(result.transitionPath.begin(), result.transitionPath.end());
        }

        return result;
    }
};

int main()
{
    PlanningProblem p;
    p.initialState = 10;
    p.goalState = 40;
    p.badStates = {99};

    p.states = {
        {10, {0.0, 0.0}},
        {20, {2.0, 0.0}},
        {30, {2.0, 2.0}},
        {40, {4.0, 2.0}},
        {50, {1.0, -2.0}},
        {60, {3.0, -2.0}},
        {99, {2.0, 1.0}}};

    p.transitions = {
        {201, 10, 20, 2.0, 1.0, 1.0, true},
        {202, 20, 30, 2.0, 1.0, 1.0, true},
        {203, 30, 40, 2.0, 1.0, 1.0, true},
        {204, 10, 50, 2.5, 1.0, 1.0, true},
        {205, 50, 60, 2.5, 1.0, 1.0, true},
        {206, 60, 40, 2.5, 1.0, 1.0, true}};

    LPAStarPlanner planner;

    PlanningResult res1 = planner.plan(p);
    cout << "Initial Plan Success: " << res1.success << "\n";
    cout << "Path: ";
    for (auto s : res1.statePath)
        cout << s << " ";
    cout << "\nTotal Cost: " << res1.totalCost << " | Min Safety: " << res1.safetyScore << "\n\n";

    p.transitions[3].available = false;

    PlanningResult res2 = planner.plan(p);
    cout << "Replanned Plan Success (Edge 204 removed): " << res2.success << "\n";
    cout << "Path: ";
    for (auto s : res2.statePath)
        cout << s << " ";
    cout << "\nTotal Cost: " << res2.totalCost << " | Min Safety: " << res2.safetyScore << "\n";

    return 0;
}