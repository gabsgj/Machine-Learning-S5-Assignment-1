#include <iostream>
#include <vector>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>

using namespace std;

const double INF = numeric_limits<double>::infinity();

class State {
public:
    uint64_t id;
    vector<double> embedding;

    State(uint64_t i, vector<double> e) : id(i), embedding(e) {}
};

class Transition {
public:
    uint64_t id, from, to;
    double cost, safety, reliability;
    bool available;

    Transition(uint64_t i, uint64_t f, uint64_t t,
               double c, double s, double r, bool a = true)
        : id(i), from(f), to(t), cost(c), safety(s),
          reliability(r), available(a) {}
};

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

class PlanningResult {
public:
    bool success = false;
    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;
    double totalCost = 0.0;
    double safetyScore = 0.0;
    double planningTimeMs = 0.0;
    size_t exploredStates = 0;
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

class DStarLitePlanner : public Planner {
private:
    struct Node {
        double distance;
        uint64_t state;

        bool operator>(const Node& other) const {
            return distance > other.distance;
        }
    };

    unordered_map<uint64_t, vector<int>> outgoing;
    unordered_map<uint64_t, vector<int>> incoming;
    unordered_map<uint64_t, double> g;
    unordered_map<uint64_t, double> rhs;
    unordered_set<uint64_t> bad;

    vector<State> states;
    vector<Transition> transitions;
    uint64_t start = 0, goal = 0;

    const State& getState(uint64_t id) const {
        for (const auto& s : states)
            if (s.id == id) return s;
        throw runtime_error("State ID not found");
    }

    double heuristic(uint64_t a, uint64_t b) const {
        const auto& x = getState(a).embedding;
        const auto& y = getState(b).embedding;

        double sum = 0.0;
        for (size_t i = 0; i < min(x.size(), y.size()); ++i) {
            double d = x[i] - y[i];
            sum += d * d;
        }
        return sqrt(sum);
    }

    bool isBad(uint64_t id) const {
        return bad.count(id) > 0;
    }

    double transitionCost(const Transition& t) const {
        double safetyPenalty = 1.0 / (t.safety + 0.01);
        double reliabilityPenalty = 1.0 - t.reliability;
        return t.cost + 2.0 * safetyPenalty + reliabilityPenalty;
    }

    void buildGraph() {
        outgoing.clear();
        incoming.clear();

        for (int i = 0; i < static_cast<int>(transitions.size()); ++i) {
            const auto& t = transitions[i];

            if (!t.available || isBad(t.from) || isBad(t.to))
                continue;

            outgoing[t.from].push_back(i);
            incoming[t.to].push_back(i);
        }
    }

    void initialize() {
        g.clear();
        rhs.clear();

        for (const auto& s : states) {
            g[s.id] = INF;
            rhs[s.id] = INF;
        }

        rhs[goal] = 0.0;
    }

    void computeShortestPath(PlanningResult& result) {
        priority_queue<Node, vector<Node>, greater<Node>> pq;
        unordered_set<uint64_t> expanded;

        pq.push({0.0, goal});

        while (!pq.empty()) {
            Node current = pq.top();
            pq.pop();

            uint64_t u = current.state;

            if (expanded.count(u))
                continue;

            expanded.insert(u);
            result.exploredStates++;

            g[u] = current.distance;

            for (int index : incoming[u]) {
                const auto& t = transitions[index];
                if (!t.available) continue;

                uint64_t predecessor = t.from;
                if (isBad(predecessor)) continue;

                double newValue = g[u] + transitionCost(t);

                if (newValue < rhs[predecessor]) {
                    rhs[predecessor] = newValue;
                    pq.push({
                        newValue + heuristic(predecessor, start),
                        predecessor
                    });
                }
            }
        }
    }

public:
    PlanningResult plan(const PlanningProblem& problem) override {
        PlanningResult result;

        states = problem.states;
        transitions = problem.transitions;
        start = problem.initialState;
        goal = problem.goalState;

        for (uint64_t b : problem.badStates)
            bad.insert(b);

        if (isBad(start) || isBad(goal))
            return result;

        buildGraph();
        initialize();

        auto begin = chrono::high_resolution_clock::now();
        computeShortestPath(result);
        auto end = chrono::high_resolution_clock::now();

        result.planningTimeMs =
            chrono::duration<double, milli>(end - begin).count();

        if (g[start] == INF)
            return result;

        uint64_t current = start;
        unordered_set<uint64_t> visited;

        result.statePath.push_back(current);
        visited.insert(current);

        while (current != goal) {
            double bestValue = INF;
            int bestTransition = -1;

            if (!outgoing.count(current))
                break;

            for (int index : outgoing[current]) {
                const auto& t = transitions[index];

                if (!t.available || isBad(t.to))
                    continue;

                double value = transitionCost(t) + g[t.to];

                if (value < bestValue) {
                    bestValue = value;
                    bestTransition = index;
                }
            }

            if (bestTransition == -1)
                break;

            const auto& chosen = transitions[bestTransition];
            current = chosen.to;

            if (visited.count(current))
                break;

            visited.insert(current);
            result.transitionPath.push_back(chosen.id);
            result.statePath.push_back(current);
            result.totalCost += chosen.cost;
        }

        if (current == goal) {
            result.success = true;
            result.safetyScore = calculateSafety(result.statePath);
        }

        return result;
    }

    double calculateSafety(const vector<uint64_t>& path) const {
        if (bad.empty())
            return INF;

        double minimumDistance = INF;

        for (uint64_t stateID : path) {
            const auto& current = getState(stateID);

            for (uint64_t badID : bad) {
                const auto& dangerous = getState(badID);

                double distance = 0.0;
                for (size_t i = 0;
                     i < min(current.embedding.size(),
                             dangerous.embedding.size()); ++i) {
                    double d = current.embedding[i] -
                               dangerous.embedding[i];
                    distance += d * d;
                }

                minimumDistance = min(minimumDistance, sqrt(distance));
            }
        }

        return minimumDistance;
    }
};

void printResult(const PlanningResult& r) {
    if (!r.success) {
        cout << "No valid path found.\n";
        return;
    }

    cout << "Path: ";
    for (auto id : r.statePath)
        cout << id << " ";

    cout << "\nTransition IDs: ";
    for (auto id : r.transitionPath)
        cout << id << " ";

    cout << "\nTotal Cost: " << r.totalCost;
    cout << "\nMinimum Safety Distance: " << r.safetyScore;
    cout << "\nExplored States: " << r.exploredStates;
    cout << "\nPlanning Time: " << r.planningTimeMs << " ms\n";
}

void testCase1() {
    cout << "\n=== TEST CASE 1: Basic Reachability ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;

    p.states = {
        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {2, 0}),
        State(3, {3, 0})
    };

    p.transitions = {
        Transition(0, 0, 1, 1, 10, 0.9),
        Transition(1, 1, 2, 1, 10, 0.9),
        Transition(2, 2, 3, 1, 10, 0.9)
    };

    DStarLitePlanner planner;
    printResult(planner.plan(p));
}

void testCase2() {
    cout << "\n=== TEST CASE 2: Bad State Avoidance ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 6;
    p.badStates = {2};

    p.states = {
        State(0, {0, 0}), State(1, {1, 1}), State(2, {2, 2}),
        State(3, {1, -1}), State(4, {2, -2}),
        State(5, {3, 0}), State(6, {4, 0})
    };

    p.transitions = {
        Transition(0, 0, 1, 1, 10, 0.9),
        Transition(1, 1, 2, 1, 10, 0.9),
        Transition(2, 2, 6, 1, 10, 0.9),
        Transition(3, 0, 3, 2, 10, 0.9),
        Transition(4, 3, 4, 2, 10, 0.9),
        Transition(5, 4, 6, 2, 10, 0.9)
    };

    DStarLitePlanner planner;
    printResult(planner.plan(p));
}

void testCase3() {
    cout << "\n=== TEST CASE 3: Safety Margin ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 5;
    p.badStates = {6};

    p.states = {
        State(0, {0, 0}), State(1, {1, 0}), State(2, {2, 0}),
        State(3, {1, 5}), State(4, {2, 5}),
        State(5, {3, 0}), State(6, {1, 1})
    };

    p.transitions = {
        Transition(0, 0, 1, 1, 2, 0.9),
        Transition(1, 1, 2, 1, 2, 0.9),
        Transition(2, 2, 5, 1, 2, 0.9),
        Transition(3, 0, 3, 3, 10, 0.95),
        Transition(4, 3, 4, 3, 10, 0.95),
        Transition(5, 4, 5, 3, 10, 0.95)
    };

    DStarLitePlanner planner;
    printResult(planner.plan(p));
}

void testCase4() {
    cout << "\n=== TEST CASE 4: Dynamic Transition ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 4;

    p.states = {
        State(0, {0, 0}), State(1, {1, 0}), State(2, {1, 1}),
        State(3, {2, 1}), State(4, {3, 0})
    };

    p.transitions = {
        Transition(0, 0, 1, 1, 10, 0.9),
        Transition(1, 1, 4, 1, 10, 0.9),
        Transition(2, 0, 2, 2, 10, 0.9),
        Transition(3, 2, 3, 2, 10, 0.9),
        Transition(4, 3, 4, 2, 10, 0.9)
    };

    DStarLitePlanner planner;

    cout << "Initial path:\n";
    printResult(planner.plan(p));

    p.transitions[1].available = false;

    cout << "\nAfter A -> G becomes unavailable:\n";
    printResult(planner.plan(p));
}

void testCase5() {
    cout << "\n=== TEST CASE 5: Goal Update ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;

    p.states = {
        State(0, {0, 0}), State(1, {1, 0}), State(2, {2, 0}),
        State(3, {3, 0}), State(4, {2, 2})
    };

    p.transitions = {
        Transition(0, 0, 1, 1, 10, 0.9),
        Transition(1, 1, 2, 1, 10, 0.9),
        Transition(2, 2, 3, 1, 10, 0.9),
        Transition(3, 2, 4, 2, 10, 0.9)
    };

    DStarLitePlanner planner;

    cout << "Original goal:\n";
    printResult(planner.plan(p));

    p.goalState = 4;

    cout << "\nAfter goal update:\n";
    printResult(planner.plan(p));
}

void testCase6() {
    cout << "\n=== TEST CASE 6: Transition Addition ===\n";

    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 4;

    p.states = {
        State(0, {0, 0}), State(1, {1, 0}), State(2, {2, 0}),
        State(3, {3, 0}), State(4, {4, 0})
    };

    p.transitions = {
        Transition(0, 0, 1, 2, 10, 0.9),
        Transition(1, 1, 2, 2, 10, 0.9),
        Transition(2, 2, 3, 2, 10, 0.9),
        Transition(3, 3, 4, 2, 10, 0.9)
    };

    DStarLitePlanner planner;

    cout << "Before shortcut:\n";
    printResult(planner.plan(p));

    p.transitions.push_back(
        Transition(4, 0, 4, 2, 10, 0.9)
    );

    cout << "\nAfter shortcut:\n";
    printResult(planner.plan(p));
}

int main() {
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();

    return 0;
}
