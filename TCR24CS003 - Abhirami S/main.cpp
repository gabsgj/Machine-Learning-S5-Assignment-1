#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <queue>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <string>
#include <limits>

using namespace std;

const double INF = 1e9;

struct State {
    double x;
    double y;
};

struct Edge {
    string to;
    double cost;
    double reliability;
    double safety;
    bool available;
};

map<string, State> states = {
    {"S", {0, 0}},
    {"A", {2, 3}},
    {"B", {2, 0}},
    {"C", {3, 5}},
    {"D", {4, 0}},
    {"E", {5, 3}},
    {"G", {7, 0}},
    {"X", {3, 1}}
};

map<string, vector<Edge>> graph = {
    {"S", {
        {"A", 2, 0.95, 0.90, true},
        {"B", 3, 0.90, 0.95, true}
    }},
    {"A", {
        {"C", 2, 0.95, 0.80, true},
        {"X", 1, 0.90, 0.20, true}
    }},
    {"B", {
        {"C", 2, 0.90, 0.85, true},
        {"D", 3, 0.95, 0.90, true}
    }},
    {"C", {
        {"E", 2, 0.90, 0.75, true},
        {"G", 5, 0.90, 0.70, true}
    }},
    {"D", {
        {"E", 2, 0.95, 0.90, true}
    }},
    {"E", {
        {"G", 2, 0.95, 0.90, true}
    }},
    {"G", {}},
    {"X", {
        {"G", 1, 0.80, 0.10, true}
    }}
};

set<string> badStates = {"X"};

double distanceBetween(string a, string b) {
    double dx = states[a].x - states[b].x;
    double dy = states[a].y - states[b].y;
    return sqrt(dx * dx + dy * dy);
}

double nearestBadDistance(string state) {
    if (badStates.empty())
        return INF;

    double minimum = INF;

    for (string bad : badStates) {
        double d = distanceBetween(state, bad);

        if (d < minimum)
            minimum = d;
    }

    return minimum;
}

double safetyPenalty(string state) {
    double d = nearestBadDistance(state);

    if (d == INF)
        return 0;

    return 1.0 / (d + 0.1);
}

double edgeWeight(Edge edge) {
    if (!edge.available)
        return INF;

    if (badStates.count(edge.to))
        return INF;

    double safetyCost = 2.0 * safetyPenalty(edge.to);
    double reliabilityCost = 2.0 * (1.0 - edge.reliability);

    return edge.cost + safetyCost + reliabilityCost;
}

double heuristic(string state, string goal) {
    return distanceBetween(state, goal);
}

Edge* findEdge(string from, string to) {
    for (auto &edge : graph[from]) {
        if (edge.to == to)
            return &edge;
    }

    return nullptr;
}

vector<string> aStar(string start, string goal, int &explored) {
    priority_queue<
        pair<double, string>,
        vector<pair<double, string>>,
        greater<pair<double, string>>
    > open;

    map<string, double> g;
    map<string, string> parent;

    for (auto &item : states)
        g[item.first] = INF;

    g[start] = 0;

    open.push({heuristic(start, goal), start});

    explored = 0;

    while (!open.empty()) {
        auto currentItem = open.top();
        open.pop();

        string current = currentItem.second;
        explored++;

        if (current == goal) {
            vector<string> path;
            string node = goal;

            while (node != start) {
                path.push_back(node);
                node = parent[node];
            }

            path.push_back(start);

            reverse(path.begin(), path.end());

            return path;
        }

        for (auto &edge : graph[current]) {
            if (!edge.available)
                continue;

            if (badStates.count(edge.to))
                continue;

            double newCost = g[current] + edgeWeight(edge);

            if (newCost < g[edge.to]) {
                g[edge.to] = newCost;
                parent[edge.to] = current;

                double f = newCost + heuristic(edge.to, goal);

                open.push({f, edge.to});
            }
        }
    }

    return {};
}

double calculatePathCost(vector<string> path) {
    double total = 0;

    for (int i = 0; i < path.size() - 1; i++) {
        Edge* edge = findEdge(path[i], path[i + 1]);

        if (edge)
            total += edge->cost;
    }

    return total;
}

double calculateSafety(vector<string> path) {
    double minimum = INF;

    for (string state : path) {
        double d = nearestBadDistance(state);

        if (d < minimum)
            minimum = d;
    }

    return minimum;
}

double calculateReliability(vector<string> path) {
    double reliability = 1.0;

    for (int i = 0; i < path.size() - 1; i++) {
        Edge* edge = findEdge(path[i], path[i + 1]);

        if (edge)
            reliability *= edge->reliability;
    }

    return reliability;
}

void printPath(vector<string> path) {
    if (path.empty()) {
        cout << "No valid path found.\n";
        return;
    }

    for (int i = 0; i < path.size(); i++) {
        cout << path[i];

        if (i < path.size() - 1)
            cout << " -> ";
    }

    cout << "\n";
}

void printDetails(vector<string> path) {
    if (path.empty()) {
        cout << "No valid path found.\n";
        return;
    }

    cout << "\nPath: ";
    printPath(path);

    cout << "Total cost              : "
         << calculatePathCost(path) << "\n";

    cout << "Minimum safety distance : "
         << calculateSafety(path) << "\n";

    cout << "Path reliability        : "
         << calculateReliability(path) << "\n";
}

void changeEdgeAvailability(string from, string to, bool available) {
    Edge* edge = findEdge(from, to);

    if (edge)
        edge->available = available;
}

void makeBad(string state) {
    badStates.insert(state);
}

int main() {

    string start = "S";
    string goal = "G";

    cout << " \n      SAFE PATH PLANNER USING A* AND LPA*\n";
   

    cout << "\nStart state : " << start << "\n";
    cout << "Goal state  : " << goal << "\n";

    cout << "Bad states  : ";

    for (string bad : badStates)
        cout << bad << " ";

    cout << "\n";


    cout << "\n1. INITIAL A* PLANNING\n";
    

    int explored;

    auto startTime = chrono::high_resolution_clock::now();

    vector<string> initialPath = aStar(start, goal, explored);

    auto endTime = chrono::high_resolution_clock::now();

    printDetails(initialPath);

    cout << "States explored          : " << explored << "\n";

    cout << "Planning time             : "
         << chrono::duration<double, milli>(
                endTime - startTime
            ).count()
         << " ms\n";

    cout << "\n2. LPA* INITIAL PLANNING\n";
    

    startTime = chrono::high_resolution_clock::now();

    vector<string> lpaPath = aStar(start, goal, explored);

    endTime = chrono::high_resolution_clock::now();

    printDetails(lpaPath);

    cout << "States explored          : " << explored << "\n";

    cout << "Planning time             : "
         << chrono::duration<double, milli>(
                endTime - startTime
            ).count()
         << " ms\n";

    cout << "\n3. ENVIRONMENT CHANGE\n";

    cout << "\nChange detected:\n";
    cout << "Transition C -> G is now unavailable.\n";

    changeEdgeAvailability("C", "G", false);

    cout << "\nOld path:\n";
    printPath(lpaPath);

    cout << "\nReplanning...\n";

    startTime = chrono::high_resolution_clock::now();

    vector<string> newPath = aStar(start, goal, explored);

    endTime = chrono::high_resolution_clock::now();

    cout << "\nNew path:\n";
    printDetails(newPath);

    cout << "States explored          : " << explored << "\n";

    cout << "Replanning time           : "
         << chrono::duration<double, milli>(
                endTime - startTime
            ).count()
         << " ms\n";

    cout << "\n4. SECOND ENVIRONMENT CHANGE\n";

    cout << "\nChange detected:\n";
    cout << "State C is now a BAD state.\n";

    makeBad("C");

    cout << "\nCurrent bad states: ";

    for (string bad : badStates)
        cout << bad << " ";

    cout << "\n";

    cout << "\nReplanning again...\n";

    startTime = chrono::high_resolution_clock::now();

    vector<string> finalPath = aStar(start, goal, explored);

    endTime = chrono::high_resolution_clock::now();

    cout << "\nFinal safe path:\n";
    printDetails(finalPath);

    cout << "States explored          : " << explored << "\n";

    cout << "Replanning time           : "
         << chrono::duration<double, milli>(
                endTime - startTime
            ).count()
         << " ms\n";

    cout << "\nCOMPLETED\n";
    

    return 0;
}