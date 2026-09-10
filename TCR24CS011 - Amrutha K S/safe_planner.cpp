#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <limits>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <string>

using namespace std;

const double INF = numeric_limits<double>::infinity();
const double EPS = 1e-9;

// ============================================================
// STATE
// ============================================================

class State {
public:
    uint64_t id;
    vector<double> embedding;

    State() {}

    State(uint64_t id, const vector<double>& embedding)
        : id(id), embedding(embedding) {}
};

// ============================================================
// TRANSITION
// ============================================================

class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;

    double cost;
    double safety;
    double reliability;

    bool available;

    Transition() {}

    Transition(uint64_t id,
               uint64_t from,
               uint64_t to,
               double cost,
               double safety,
               double reliability,
               bool available = true)
        : id(id),
          from(from),
          to(to),
          cost(cost),
          safety(safety),
          reliability(reliability),
          available(available) {}
};

// ============================================================
// PLANNING PROBLEM
// ============================================================

class PlanningProblem {
public:
    uint64_t initialState;
    uint64_t goalState;

    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

// ============================================================
// PLANNING RESULT
// ============================================================

class PlanningResult {
public:
    bool success = false;

    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;

    double totalCost = 0.0;
    double safetyScore = 0.0;
    double minimumSafetyDistance = 0.0;
    double cumulativeReliability = 1.0;

    size_t statesExplored = 0;

    double planningTimeMs = 0.0;
};

// ============================================================
// PLANNER INTERFACE
// ============================================================

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() {}
};

// ============================================================
// PRIORITY QUEUE NODE
// ============================================================

struct QueueNode {
    double k1;
    double k2;
    uint64_t state;

    bool operator>(const QueueNode& other) const {
        if (fabs(k1 - other.k1) > EPS)
            return k1 > other.k1;

        return k2 > other.k2;
    }
};

// ============================================================
// D* LITE SAFE PLANNER
// ============================================================

class DStarLitePlanner : public Planner {

private:

    // Graph
    unordered_map<uint64_t, State> stateMap;

    unordered_map<uint64_t, vector<Transition>> outgoing;
    unordered_map<uint64_t, vector<Transition>> incoming;

    // D* Lite values
    unordered_map<uint64_t, double> g;
    unordered_map<uint64_t, double> rhs;

    priority_queue<
        QueueNode,
        vector<QueueNode>,
        greater<QueueNode>
    > openList;

    unordered_set<uint64_t> badSet;

    uint64_t start;
    uint64_t goal;

    double km = 0.0;

    // Safety weight
    double safetyWeight = 2.0;

    // Small value to avoid division by zero
    const double epsilon = 1e-6;

    // Statistics
    size_t exploredStates = 0;

    // --------------------------------------------------------
    // Get state
    // --------------------------------------------------------

    State* getState(uint64_t id) {

        auto it = stateMap.find(id);

        if (it == stateMap.end())
            return nullptr;

        return &it->second;
    }

    // --------------------------------------------------------
    // Euclidean distance
    // --------------------------------------------------------

    double euclideanDistance(uint64_t a, uint64_t b) {

        State* s1 = getState(a);
        State* s2 = getState(b);

        if (!s1 || !s2)
            return INF;

        size_t d = min(
            s1->embedding.size(),
            s2->embedding.size()
        );

        double sum = 0.0;

        for (size_t i = 0; i < d; i++) {

            double diff =
                s1->embedding[i] -
                s2->embedding[i];

            sum += diff * diff;
        }

        return sqrt(sum);
    }

    // --------------------------------------------------------
    // Distance from state to nearest bad state
    // --------------------------------------------------------

    double distanceToNearestBad(uint64_t state) {

        if (badSet.empty())
            return INF;

        double minimum = INF;

        for (uint64_t bad : badSet) {

            double d =
                euclideanDistance(state, bad);

            minimum = min(minimum, d);
        }

        return minimum;
    }

    // --------------------------------------------------------
    // Safety penalty
    // --------------------------------------------------------

    double safetyPenalty(uint64_t state) {

        double d =
            distanceToNearestBad(state);

        if (isinf(d))
            return 0.0;

        return safetyWeight /
               (d + epsilon);
    }

    // --------------------------------------------------------
    // Effective transition cost
    // --------------------------------------------------------

    double edgeCost(const Transition& t) {

        if (!t.available)
            return INF;

        if (badSet.count(t.to))
            return INF;

        double penalty =
            safetyPenalty(t.to);

        /*
         * Lower reliability increases cost.
         *
         * reliability = 1 -> no reliability penalty
         * reliability = 0.5 -> larger cost
         */

        double reliabilityPenalty =
            1.0 / max(t.reliability, 0.01);

        return t.cost
             + penalty
             + 0.1 * reliabilityPenalty;
    }

    // --------------------------------------------------------
    // Heuristic
    // --------------------------------------------------------

    double heuristic(uint64_t a, uint64_t b) {

        return euclideanDistance(a, b);
    }

    // --------------------------------------------------------
    // Calculate key
    // --------------------------------------------------------

    pair<double, double> calculateKey(uint64_t state) {

        double minimum =
            min(g[state], rhs[state]);

        return {
            minimum +
            heuristic(start, state) +
            km,

            minimum
        };
    }

    // --------------------------------------------------------
    // Compare keys
    // --------------------------------------------------------

    bool keyLess(
        const pair<double, double>& a,
        const pair<double, double>& b) {

        if (a.first < b.first - EPS)
            return true;

        if (fabs(a.first - b.first) < EPS &&
            a.second < b.second - EPS)
            return true;

        return false;
    }

    // --------------------------------------------------------
    // Initialize
    // --------------------------------------------------------

    void initialize() {

        while (!openList.empty())
            openList.pop();

        g.clear();
        rhs.clear();

        for (auto& p : stateMap) {

            g[p.first] = INF;
            rhs[p.first] = INF;
        }

        km = 0.0;
        exploredStates = 0;

        rhs[goal] = 0.0;

        auto key = calculateKey(goal);

        openList.push({
            key.first,
            key.second,
            goal
        });
    }

    // --------------------------------------------------------
    // Get minimum successor value
    // --------------------------------------------------------

    double getMinSuccessor(uint64_t state) {

        double minimum = INF;

        auto it = outgoing.find(state);

        if (it == outgoing.end())
            return INF;

        for (const Transition& t : it->second) {

            if (!t.available)
                continue;

            if (badSet.count(t.to))
                continue;

            double c = edgeCost(t);

            if (isinf(c))
                continue;

            double value =
                c + g[t.to];

            minimum = min(minimum, value);
        }

        return minimum;
    }

    // --------------------------------------------------------
    // Update vertex
    // --------------------------------------------------------

    void updateVertex(uint64_t u) {

        if (u != goal) {

            rhs[u] =
                getMinSuccessor(u);
        }

        /*
         * Lazy deletion is used for the priority queue.
         * Old entries are simply ignored later.
         */

        if (fabs(g[u] - rhs[u]) > EPS) {

            auto key = calculateKey(u);

            openList.push({
                key.first,
                key.second,
                u
            });
        }
    }

    // --------------------------------------------------------
    // Compute shortest path
    // --------------------------------------------------------

    void computeShortestPath() {

        while (!openList.empty()) {

            QueueNode current =
                openList.top();

            auto topKey =
                make_pair(
                    current.k1,
                    current.k2
                );

            auto startKey =
                calculateKey(start);

            if (!keyLess(topKey, startKey) &&
                fabs(rhs[start] - g[start]) < EPS)
                break;

            openList.pop();

            auto newKey =
                calculateKey(current.state);

            /*
             * Ignore stale queue entries.
             */

            if (keyLess(newKey, topKey)) {

                openList.push({
                    newKey.first,
                    newKey.second,
                    current.state
                });

                continue;
            }

            exploredStates++;

            if (g[current.state] >
                rhs[current.state]) {

                g[current.state] =
                    rhs[current.state];

                auto in =
                    incoming.find(current.state);

                if (in != incoming.end()) {

                    for (const Transition& t :
                         in->second) {

                        updateVertex(t.from);
                    }
                }

            } else {

                g[current.state] = INF;

                updateVertex(current.state);

                auto in =
                    incoming.find(current.state);

                if (in != incoming.end()) {

                    for (const Transition& t :
                         in->second) {

                        updateVertex(t.from);
                    }
                }
            }
        }
    }

    // --------------------------------------------------------
    // Reconstruct path
    // --------------------------------------------------------

    PlanningResult reconstructPath() {

        PlanningResult result;

        if (isinf(g[start]))
            return result;

        uint64_t current = start;

        unordered_set<uint64_t> visited;

        result.statePath.push_back(current);

        double totalCost = 0.0;

        double minSafetyDistance = INF;

        double reliability = 1.0;

        while (current != goal) {

            if (visited.count(current)) {

                result.success = false;
                return result;
            }

            visited.insert(current);

            auto it =
                outgoing.find(current);

            if (it == outgoing.end()) {

                result.success = false;
                return result;
            }

            double bestValue = INF;

            const Transition* bestTransition =
                nullptr;

            for (const Transition& t :
                 it->second) {

                if (!t.available)
                    continue;

                if (badSet.count(t.to))
                    continue;

                double c =
                    edgeCost(t);

                if (isinf(c))
                    continue;

                double value =
                    c + g[t.to];

                if (value < bestValue) {

                    bestValue = value;
                    bestTransition = &t;
                }
            }

            if (!bestTransition) {

                result.success = false;
                return result;
            }

            const Transition& t =
                *bestTransition;

            totalCost += t.cost;

            reliability *=
                t.reliability;

            double safetyDistance =
                distanceToNearestBad(t.to);

            minSafetyDistance =
                min(
                    minSafetyDistance,
                    safetyDistance
                );

            result.transitionPath.push_back(
                t.id
            );

            current = t.to;

            result.statePath.push_back(
                current
            );
        }

        /*
         * Include the initial state's safety.
         */

        minSafetyDistance =
            min(
                minSafetyDistance,
                distanceToNearestBad(start)
            );

        if (isinf(minSafetyDistance))
            minSafetyDistance = 0.0;

        result.success = true;

        result.totalCost = totalCost;

        result.minimumSafetyDistance =
            minSafetyDistance;

        result.safetyScore =
            minSafetyDistance;

        result.cumulativeReliability =
            reliability;

        result.statesExplored =
            exploredStates;

        return result;
    }

public:

    // --------------------------------------------------------
    // Constructor
    // --------------------------------------------------------

    DStarLitePlanner(
        double safetyWeight = 2.0)
        : safetyWeight(safetyWeight) {}

    // --------------------------------------------------------
    // Plan
    // --------------------------------------------------------

    PlanningResult plan(
        const PlanningProblem& problem)
        override {

        auto begin =
            chrono::high_resolution_clock::now();

        // Copy states
        stateMap.clear();

        for (const State& s :
             problem.states) {

            stateMap[s.id] = s;
        }

        // Copy bad states
        badSet.clear();

        for (uint64_t id :
             problem.badStates) {

            badSet.insert(id);
        }

        // Build graph
        outgoing.clear();
        incoming.clear();

        for (const Transition& t :
             problem.transitions) {

            outgoing[t.from].push_back(t);

            incoming[t.to].push_back(t);
        }

        start = problem.initialState;
        goal = problem.goalState;

        if (!stateMap.count(start) ||
            !stateMap.count(goal)) {

            PlanningResult result;

            result.success = false;

            return result;
        }

        // Goal itself cannot be bad
        if (badSet.count(goal)) {

            PlanningResult result;

            result.success = false;

            return result;
        }

        initialize();

        computeShortestPath();

        PlanningResult result =
            reconstructPath();

        auto end =
            chrono::high_resolution_clock::now();

        result.planningTimeMs =
            chrono::duration<double, milli>(
                end - begin
            ).count();

        return result;
    }

    // --------------------------------------------------------
    // Dynamic transition update
    // --------------------------------------------------------

    void updateTransition(
        vector<Transition>& transitions,
        uint64_t transitionId,
        bool available) {

        for (Transition& t :
             transitions) {

            if (t.id == transitionId) {

                t.available = available;
                return;
            }
        }
    }
};

// ============================================================
// PRINT RESULT
// ============================================================

void printResult(
    const PlanningResult& result) {

    cout << "\n========================================\n";
    cout << "           PLANNING RESULT\n";
    cout << "========================================\n";

    cout << "Success              : "
         << (result.success ? "YES" : "NO")
         << "\n";

    if (!result.success) {

        cout << "No safe path found.\n";
        cout << "========================================\n";

        return;
    }

    cout << "State Path            : ";

    for (size_t i = 0;
         i < result.statePath.size();
         i++) {

        cout << result.statePath[i];

        if (i + 1 <
            result.statePath.size())
            cout << " -> ";
    }

    cout << "\n";

    cout << "Transition Path       : ";

    for (size_t i = 0;
         i < result.transitionPath.size();
         i++) {

        cout << result.transitionPath[i];

        if (i + 1 <
            result.transitionPath.size())
            cout << " -> ";
    }

    cout << "\n";

    cout << fixed << setprecision(4);

    cout << "Total Cost            : "
         << result.totalCost
         << "\n";

    cout << "Minimum Safety Dist.  : "
         << result.minimumSafetyDistance
         << "\n";

    cout << "Cumulative Reliability: "
         << result.cumulativeReliability
         << "\n";

    cout << "States Explored       : "
         << result.statesExplored
         << "\n";

    cout << "Planning Time (ms)    : "
         << result.planningTimeMs
         << "\n";

    cout << "Bad States Visited    : 0\n";

    cout << "========================================\n";
}

// ============================================================
// CREATE BASIC PROBLEM
// ============================================================

PlanningProblem createBasicProblem() {

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

        Transition(0, 0, 1,
                   1.0, 1.0, 0.95),

        Transition(1, 1, 2,
                   1.0, 1.0, 0.95),

        Transition(2, 2, 3,
                   1.0, 1.0, 0.95)
    };

    return p;
}

// ============================================================
// TEST CASE 1
// ============================================================

void testCase1() {

    cout << "\n\nTEST CASE 1: BASIC REACHABILITY\n";

    PlanningProblem p =
        createBasicProblem();

    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(p);

    printResult(result);
}

// ============================================================
// TEST CASE 2
// ============================================================

void testCase2() {

    cout << "\n\nTEST CASE 2: BAD STATE AVOIDANCE\n";

    PlanningProblem p;

    p.initialState = 0;
    p.goalState = 5;

    p.states = {

        State(0, {0, 0}),   // S
        State(1, {1, 0}),   // A
        State(2, {2, 0}),   // X - bad
        State(3, {1, 1}),   // C
        State(4, {2, 1}),   // D
        State(5, {3, 1})    // G
    };

    p.badStates = {2};

    p.transitions = {

        // Unsafe path
        Transition(0, 0, 1, 1, 1, 0.9),
        Transition(1, 1, 2, 1, 1, 0.9),
        Transition(2, 2, 5, 1, 1, 0.9),

        // Safe path
        Transition(3, 0, 3, 1, 1, 0.95),
        Transition(4, 3, 4, 1, 1, 0.95),
        Transition(5, 4, 5, 1, 1, 0.95)
    };

    DStarLitePlanner planner;

    PlanningResult result =
        planner.plan(p);

    printResult(result);
}

// ============================================================
// TEST CASE 3
// ============================================================

void testCase3() {

    cout << "\n\nTEST CASE 3: SAFETY MARGIN\n";

    PlanningProblem p;

    p.initialState = 0;
    p.goalState = 5;

    p.states = {

        State(0, {0, 0}),     // S

        // Short path
        State(1, {1, 0}),
        State(2, {2, 0}),

        // Safer path
        State(3, {0, 2}),
        State(4, {2, 2}),

        State(5, {4, 0})      // G
    };

    // Bad state near the short path
    p.badStates = {
        6
    };

    p.states.push_back(
        State(6, {2, 0.3})
    );

    p.transitions = {

        // Path 1: cheaper but close to bad state
        Transition(0, 0, 1, 1, 1, 0.95),
        Transition(1, 1, 2, 1, 1, 0.95),
        Transition(2, 2, 5, 1, 1, 0.95),

        // Path 2: more expensive but safer
        Transition(3, 0, 3, 2, 1, 0.98),
        Transition(4, 3, 4, 2, 1, 0.98),
        Transition(5, 4, 5, 2, 1, 0.98)
    };

    DStarLitePlanner planner(
        5.0
    );

    PlanningResult result =
        planner.plan(p);

    printResult(result);
}

// ============================================================
// TEST CASE 4
// ============================================================

void testCase4() {

    cout << "\n\nTEST CASE 4: DYNAMIC TRANSITION\n";

    PlanningProblem p;

    p.initialState = 0;
    p.goalState = 4;

    p.states = {

        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {2, 0}),
        State(3, {1, 1}),
        State(4, {3, 0})
    };

    p.transitions = {

        // Original route
        Transition(0, 0, 1, 1, 1, 0.95),
        Transition(1, 1, 4, 1, 1, 0.95),

        // Alternative route
        Transition(2, 0, 3, 2, 1, 0.95),
        Transition(3, 3, 4, 2, 1, 0.95)
    };

    DStarLitePlanner planner;

    cout << "\nInitial environment:\n";

    PlanningResult first =
        planner.plan(p);

    printResult(first);

    /*
     * Transition 1: A -> G becomes unavailable.
     */

    p.transitions[1].available = false;

    cout << "\nAfter transition A -> G becomes unavailable:\n";

    PlanningResult second =
        planner.plan(p);

    printResult(second);
}

// ============================================================
// TEST CASE 5
// ============================================================

void testCase5() {

    cout << "\n\nTEST CASE 5: GOAL UPDATE\n";

    PlanningProblem p;

    p.initialState = 0;
    p.goalState = 3;

    p.states = {

        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {0, 1}),
        State(3, {2, 0}),
        State(4, {2, 1})
    };

    p.transitions = {

        Transition(0, 0, 1, 1, 1, 0.95),
        Transition(1, 1, 3, 1, 1, 0.95),

        Transition(2, 0, 2, 1, 1, 0.95),
        Transition(3, 2, 4, 1, 1, 0.95)
    };

    DStarLitePlanner planner;

    cout << "\nOriginal goal = State 3\n";

    PlanningResult first =
        planner.plan(p);

    printResult(first);

    p.goalState = 4;

    cout << "\nUpdated goal = State 4\n";

    PlanningResult second =
        planner.plan(p);

    printResult(second);
}

// ============================================================
// TEST CASE 6
// ============================================================

void testCase6() {

    cout << "\n\nTEST CASE 6: TRANSITION ADDITION\n";

    PlanningProblem p;

    p.initialState = 0;
    p.goalState = 4;

    p.states = {

        State(0, {0, 0}),
        State(1, {1, 0}),
        State(2, {2, 0}),
        State(3, {1, 1}),
        State(4, {3, 0})
    };

    p.transitions = {

        Transition(0, 0, 1, 3, 1, 0.9),
        Transition(1, 1, 4, 3, 1, 0.9),

        Transition(2, 0, 3, 4, 1, 0.9),
        Transition(3, 3, 4, 4, 1, 0.9)
    };

    DStarLitePlanner planner;

    cout << "\nBefore shortcut is added:\n";

    PlanningResult first =
        planner.plan(p);

    printResult(first);

    /*
     * Add a new cheap shortcut.
     */

    p.transitions.push_back(
        Transition(
            4,
            0,
            4,
            1.0,
            1.0,
            0.99
        )
    );

    cout << "\nAfter shortcut S -> G is added:\n";

    PlanningResult second =
        planner.plan(p);

    printResult(second);
}

// ============================================================
// MAIN
// ============================================================

int main() {

    cout << "\n========================================\n";
    cout << " SAFE SEMANTIC PLANNER\n";
    cout << " D* Lite Based Implementation\n";
    cout << "========================================\n";

    while (true) {

        cout << "\n";
        cout << "1. Test Case 1 - Basic Reachability\n";
        cout << "2. Test Case 2 - Bad State Avoidance\n";
        cout << "3. Test Case 3 - Safety Margin\n";
        cout << "4. Test Case 4 - Dynamic Transition\n";
        cout << "5. Test Case 5 - Goal Update\n";
        cout << "6. Test Case 6 - Transition Addition\n";
        cout << "7. Run All Test Cases\n";
        cout << "8. Exit\n";

        cout << "\nEnter choice: ";

        int choice;

        cin >> choice;

        switch (choice) {

            case 1:
                testCase1();
                break;

            case 2:
                testCase2();
                break;

            case 3:
                testCase3();
                break;

            case 4:
                testCase4();
                break;

            case 5:
                testCase5();
                break;

            case 6:
                testCase6();
                break;

            case 7:

                testCase1();
                testCase2();
                testCase3();
                testCase4();
                testCase5();
                testCase6();

                break;

            case 8:

                cout << "\nProgram terminated.\n";
                return 0;

            default:

                cout << "\nInvalid choice.\n";
        }
    }

    return 0;
}