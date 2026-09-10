#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <cmath>
#include <cstdint>
#include <limits>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>

using namespace std;

const double INF = numeric_limits<double>::infinity();


// ============================================================
// STATE
// ============================================================

class State {
public:
    uint64_t id;
    vector<double> embedding;

    State() {}

    State(uint64_t id, vector<double> embedding)
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

    Transition(
        uint64_t id,
        uint64_t from,
        uint64_t to,
        double cost,
        double safety,
        double reliability,
        bool available = true
    )
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
    bool success;

    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;

    double totalCost;
    double safetyScore;
    double cumulativeReliability;

    int exploredStates;
    double planningTimeMs;

    PlanningResult()
        : success(false),
          totalCost(0.0),
          safetyScore(0.0),
          cumulativeReliability(1.0),
          exploredStates(0),
          planningTimeMs(0.0) {}
};


// ============================================================
// D* LITE PLANNER
// ============================================================

class DStarLite {

private:

    unordered_map<uint64_t, double> g;
    unordered_map<uint64_t, double> rhs;

    // --------------------------------------------------------
    // Find state
    // --------------------------------------------------------

    const State* findState(
        const PlanningProblem& problem,
        uint64_t id
    ) const {

        for (const State& state : problem.states) {

            if (state.id == id)
                return &state;
        }

        return nullptr;
    }


    // --------------------------------------------------------
    // Euclidean distance
    // --------------------------------------------------------

    double distance(
        const State& a,
        const State& b
    ) const {

        double sum = 0.0;

        size_t dimensions =
            min(
                a.embedding.size(),
                b.embedding.size()
            );

        for (size_t i = 0; i < dimensions; i++) {

            double difference =
                a.embedding[i] - b.embedding[i];

            sum += difference * difference;
        }

        return sqrt(sum);
    }


    // --------------------------------------------------------
    // Heuristic
    // --------------------------------------------------------

    double heuristic(
        const PlanningProblem& problem,
        uint64_t from,
        uint64_t to
    ) const {

        const State* a =
            findState(problem, from);

        const State* b =
            findState(problem, to);

        if (a == nullptr || b == nullptr)
            return 0.0;

        return distance(*a, *b);
    }


    // --------------------------------------------------------
    // Check whether state is bad
    // --------------------------------------------------------

    bool isBad(
        const PlanningProblem& problem,
        uint64_t state
    ) const {

        return find(
            problem.badStates.begin(),
            problem.badStates.end(),
            state
        ) != problem.badStates.end();
    }


    // --------------------------------------------------------
    // Get available outgoing transitions
    // --------------------------------------------------------

    vector<const Transition*> outgoing(
        const PlanningProblem& problem,
        uint64_t state
    ) const {

        vector<const Transition*> result;

        for (const Transition& transition :
             problem.transitions) {

            if (transition.from != state)
                continue;

            if (!transition.available)
                continue;

            if (isBad(problem, transition.to))
                continue;

            result.push_back(&transition);
        }

        return result;
    }


    // --------------------------------------------------------
    // Dijkstra-style reverse computation
    //
    // This is used as the shortest-path core for the
    // D* Lite demonstration.
    // --------------------------------------------------------

    void computeShortestPath(
        const PlanningProblem& problem
    ) {

        g.clear();
        rhs.clear();

        for (const State& state :
             problem.states) {

            g[state.id] = INF;
            rhs[state.id] = INF;
        }

        rhs[problem.goalState] = 0.0;

        priority_queue<
            pair<double, uint64_t>,
            vector<pair<double, uint64_t>>,
            greater<pair<double, uint64_t>>
        > open;

        open.push({
            0.0,
            problem.goalState
        });


        while (!open.empty()) {

            auto current = open.top();
            open.pop();

            double currentCost =
                current.first;

            uint64_t currentState =
                current.second;

            if (currentCost >
                g[currentState]) {

                continue;
            }

            g[currentState] =
                currentCost;


            // Find predecessors.
            for (const Transition& transition :
                 problem.transitions) {

                if (!transition.available)
                    continue;

                if (transition.to != currentState)
                    continue;

                if (isBad(
                        problem,
                        transition.from
                    )) {

                    continue;
                }

                uint64_t predecessor =
                    transition.from;

                double newCost =
                    transition.cost
                    + currentCost;

                if (newCost <
                    rhs[predecessor]) {

                    rhs[predecessor] =
                        newCost;

                    open.push({
                        newCost,
                        predecessor
                    });
                }
            }
        }
    }


    // --------------------------------------------------------
    // Choose best transition
    // --------------------------------------------------------

    const Transition* bestTransition(
        const PlanningProblem& problem,
        uint64_t current
    ) const {

        const Transition* best =
            nullptr;

        double bestCost =
            INF;

        for (const Transition& transition :
             problem.transitions) {

            if (transition.from != current)
                continue;

            if (!transition.available)
                continue;

            if (isBad(
                    problem,
                    transition.to
                )) {

                continue;
            }

            if (g.find(transition.to) ==
                g.end()) {

                continue;
            }

            if (g.at(transition.to) == INF)
                continue;

            double value =
                transition.cost
                + g.at(transition.to);

            if (value < bestCost) {

                bestCost = value;
                best = &transition;
            }
        }

        return best;
    }


public:

    // ========================================================
    // PLAN
    // ========================================================

    PlanningResult plan(
        const PlanningProblem& problem
    ) {

        PlanningResult result;

        auto start =
            chrono::high_resolution_clock::now();


        // ----------------------------------------------------
        // Initial and goal states cannot be bad.
        // ----------------------------------------------------

        if (isBad(
                problem,
                problem.initialState
            ) ||
            isBad(
                problem,
                problem.goalState
            )) {

            return result;
        }


        // ----------------------------------------------------
        // D* Lite shortest-path computation
        // ----------------------------------------------------

        computeShortestPath(problem);


        // ----------------------------------------------------
        // Check whether initial state is reachable.
        // ----------------------------------------------------

        if (g[problem.initialState] == INF) {

            auto end =
                chrono::high_resolution_clock::now();

            result.planningTimeMs =
                chrono::duration<double, milli>(
                    end - start
                ).count();

            return result;
        }


        // ----------------------------------------------------
        // Reconstruct path.
        // ----------------------------------------------------

        uint64_t current =
            problem.initialState;

        unordered_set<uint64_t> visited;

        visited.insert(current);

        result.statePath.push_back(current);


        while (
            current != problem.goalState
        ) {

            const Transition* next =
                bestTransition(
                    problem,
                    current
                );

            if (next == nullptr) {

                result.success = false;
                break;
            }


            // Prevent loops.
            if (visited.count(next->to)) {

                result.success = false;
                break;
            }


            result.transitionPath.push_back(
                next->id
            );

            result.totalCost +=
                next->cost;

            result.cumulativeReliability *=
                next->reliability;

            current =
                next->to;

            result.statePath.push_back(
                current
            );

            visited.insert(current);


            if (result.statePath.size() >
                problem.states.size() + 1) {

                result.success = false;
                break;
            }
        }


        if (current ==
            problem.goalState) {

            result.success = true;
        }


        // ----------------------------------------------------
        // Calculate minimum safety distance.
        // ----------------------------------------------------

        if (result.success) {

            result.safetyScore = INF;

            for (uint64_t stateID :
                 result.statePath) {

                const State* state =
                    findState(
                        problem,
                        stateID
                    );

                if (state == nullptr)
                    continue;

                double nearestBad =
                    INF;

                for (uint64_t badID :
                     problem.badStates) {

                    const State* badState =
                        findState(
                            problem,
                            badID
                        );

                    if (badState == nullptr)
                        continue;

                    double d =
                        distance(
                            *state,
                            *badState
                        );

                    nearestBad =
                        min(
                            nearestBad,
                            d
                        );
                }

                if (nearestBad != INF) {

                    result.safetyScore =
                        min(
                            result.safetyScore,
                            nearestBad
                        );
                }
            }

            if (result.safetyScore == INF) {

                result.safetyScore = 0.0;
            }
        }


        // ----------------------------------------------------
        // Number of explored states.
        // ----------------------------------------------------

        result.exploredStates = 0;

        for (const auto& entry : g) {

            if (entry.second != INF)
                result.exploredStates++;
        }


        // ----------------------------------------------------
        // Planning time.
        // ----------------------------------------------------

        auto end =
            chrono::high_resolution_clock::now();

        result.planningTimeMs =
            chrono::duration<double, milli>(
                end - start
            ).count();

        return result;
    }
};


// ============================================================
// STATE NAME
// ============================================================

string stateName(uint64_t id) {

    switch (id) {

        case 0:
            return "S";

        case 1:
            return "A";

        case 2:
            return "B";

        case 3:
            return "C";

        case 4:
            return "G";

        case 5:
            return "X";

        default:
            return "State" +
                   to_string(id);
    }
}


// ============================================================
// PRINT RESULT
// ============================================================

void printResult(
    const PlanningProblem& problem,
    const PlanningResult& result
) {

    cout << "\nPath: ";

    if (!result.success) {

        cout << "NO VALID PATH";
    }
    else {

        for (size_t i = 0;
             i < result.statePath.size();
             i++) {

            cout << stateName(
                result.statePath[i]
            );

            if (i + 1 <
                result.statePath.size()) {

                cout << " -> ";
            }
        }
    }

    cout << fixed << setprecision(4);

    cout << "\nSuccess: "
         << (result.success ? "YES" : "NO");

    cout << "\nTotal Cost: "
         << result.totalCost;

    cout << "\nMinimum Safety Distance: "
         << result.safetyScore;

    cout << "\nCumulative Reliability: "
         << result.cumulativeReliability;

    cout << "\nExplored States: "
         << result.exploredStates;

    cout << "\nPlanning Time: "
         << result.planningTimeMs
         << " ms\n";
}

// ============================================================
// COUNT BAD STATES
// ============================================================
bool isBadStateForReport(
    const PlanningProblem& problem,
    uint64_t state
);
int countBadStates(
    const PlanningProblem& problem,
    const PlanningResult& result
) {

    int count = 0;

    for (uint64_t state :
         result.statePath) {

        if (isBadStateForReport(
                problem,
                state
            )) {

            count++;
        }
    }

    return count;
}


// Helper used only by countBadStates.
bool isBadStateForReport(
    const PlanningProblem& problem,
    uint64_t state
) {

    return find(
        problem.badStates.begin(),
        problem.badStates.end(),
        state
    ) != problem.badStates.end();
}


// ============================================================
// CREATE BASE PROBLEM
// ============================================================

PlanningProblem createBaseProblem() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 4;


    // --------------------------------------------------------
    // States
    // --------------------------------------------------------

    problem.states = {

        State(0, {0.0, 0.0}),     // S

        State(1, {2.0, 2.0}),     // A

        State(2, {4.0, 3.0}),     // B

        State(3, {2.0, -2.0}),    // C

        State(4, {6.0, 0.0}),     // G

        State(5, {4.0, 1.0})      // X
    };


    // --------------------------------------------------------
    // Bad state
    // --------------------------------------------------------

    problem.badStates = {
        5
    };


    // --------------------------------------------------------
    // Transitions
    // --------------------------------------------------------

    problem.transitions = {

        // Safe path 1:
        // S -> A -> B -> G

        Transition(
            0,
            0,
            1,
            5.0,
            0.9,
            0.95
        ),

        Transition(
            1,
            1,
            2,
            4.0,
            0.8,
            0.90
        ),

        Transition(
            2,
            2,
            4,
            3.0,
            0.9,
            0.95
        ),


        // Safe path 2:
        // S -> C -> G

        Transition(
            3,
            0,
            3,
            6.0,
            0.95,
            0.98
        ),

        Transition(
            4,
            3,
            4,
            5.0,
            0.95,
            0.97
        ),


        // Dangerous path:
        // A -> X -> G

        Transition(
            5,
            1,
            5,
            1.0,
            0.2,
            0.50
        ),

        Transition(
            6,
            5,
            4,
            1.0,
            0.2,
            0.50
        )
    };


    return problem;
}


// ============================================================
// TEST CASE 1
// ============================================================

PlanningResult testCase1() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 1: BASIC REACHABILITY";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();

    DStarLite planner;

    PlanningResult result =
        planner.plan(problem);

    printResult(
        problem,
        result
    );

    return result;
}


// ============================================================
// TEST CASE 2
// ============================================================

PlanningResult testCase2() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 2: BAD STATE AVOIDANCE";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();

    DStarLite planner;

    PlanningResult result =
        planner.plan(problem);

    printResult(
        problem,
        result
    );

    cout << "Bad States Visited: "
         << countBadStates(
                problem,
                result
            )
         << "\n";

    return result;
}


// ============================================================
// TEST CASE 3
// ============================================================

PlanningResult testCase3() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 3: SAFETY MARGIN";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();


    // Make the upper route cheaper.
    problem.transitions[0].cost = 2.0;
    problem.transitions[1].cost = 2.0;
    problem.transitions[2].cost = 2.0;


    // Make the lower route expensive.
    problem.transitions[3].cost = 8.0;
    problem.transitions[4].cost = 8.0;


    cout << "Two safe paths are available.";
    cout << "\nThe planner compares their transition costs";
    cout << "\nwhile completely avoiding the bad state.\n";


    DStarLite planner;

    PlanningResult result =
        planner.plan(problem);

    printResult(
        problem,
        result
    );

    return result;
}


// ============================================================
// TEST CASE 4
// ============================================================

PlanningResult testCase4() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 4: DYNAMIC TRANSITION";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();

    DStarLite planner;


    cout << "\nInitial environment:\n";

    PlanningResult first =
        planner.plan(problem);

    printResult(
        problem,
        first
    );


    // Disable A -> B.
    problem.transitions[1].available =
        false;


    cout << "\nAfter A -> B becomes unavailable:\n";

    PlanningResult second =
        planner.plan(problem);

    printResult(
        problem,
        second
    );

    return second;
}


// ============================================================
// TEST CASE 5
// ============================================================

PlanningResult testCase5() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 5: GOAL UPDATE";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();

    DStarLite planner;


    cout << "\nOriginal goal: G\n";

    PlanningResult first =
        planner.plan(problem);

    printResult(
        problem,
        first
    );


    // Change goal to B.
    problem.goalState = 2;


    cout << "\nUpdated goal: B\n";

    PlanningResult second =
        planner.plan(problem);

    printResult(
        problem,
        second
    );

    return second;
}


// ============================================================
// TEST CASE 6
// ============================================================

PlanningResult testCase6() {

    cout << "\n\n========================================";
    cout << "\nTEST CASE 6: TRANSITION ADDITION";
    cout << "\n========================================\n";

    PlanningProblem problem =
        createBaseProblem();

    DStarLite planner;


    cout << "\nBefore shortcut:\n";

    PlanningResult first =
        planner.plan(problem);

    printResult(
        problem,
        first
    );


    // Add direct shortcut S -> G.
    problem.transitions.push_back(
        Transition(
            7,
            0,
            4,
            4.0,
            0.95,
            0.99
        )
    );


    cout << "\nAfter adding shortcut S -> G:\n";

    PlanningResult second =
        planner.plan(problem);

    printResult(
        problem,
        second
    );

    return second;
}


// ============================================================
// SAVE RESULTS
// ============================================================
void saveResults() {

    ofstream file("results/results.csv");

    if (!file.is_open()) {
        cout << "\nCould not open results/results.csv\n";
        return;
    }

    file << "Test Case,Success,Bad States,Cost,Minimum Safety Distance,Reliability,Explored States,Planning Time(ms)\n";

    // Test 1
    {
        PlanningProblem problem = createBaseProblem();
        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "1,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    // Test 2
    {
        PlanningProblem problem = createBaseProblem();
        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "2,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    // Test 3
    {
        PlanningProblem problem = createBaseProblem();

        problem.transitions[0].cost = 2.0;
        problem.transitions[1].cost = 2.0;
        problem.transitions[2].cost = 2.0;

        problem.transitions[3].cost = 8.0;
        problem.transitions[4].cost = 8.0;

        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "3,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    // Test 4
    {
        PlanningProblem problem = createBaseProblem();

        problem.transitions[1].available = false;

        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "4,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    // Test 5
    {
        PlanningProblem problem = createBaseProblem();

        problem.goalState = 2;

        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "5,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    // Test 6
    {
        PlanningProblem problem = createBaseProblem();

        // Add shortcut S -> G
        problem.transitions.push_back(
            Transition(
                7,
                0,
                4,
                4.0,
                0.95,
                0.99
            )
        );

        DStarLite planner;
        PlanningResult r = planner.plan(problem);

        file << "6,"
             << r.success << ","
             << countBadStates(problem, r) << ","
             << r.totalCost << ","
             << r.safetyScore << ","
             << r.cumulativeReliability << ","
             << r.exploredStates << ","
             << r.planningTimeMs << "\n";
    }

    file.close();

    cout << "\nExperimental results saved to:";
    cout << "\nresults/results.csv\n";
}

// ============================================================
// MAIN
// ============================================================

int main() {

    cout << "\n\n";
    cout << "==============================================";
    cout << "\n       SAFE SEMANTIC PLANNER";
    cout << "\n             D* LITE";
    cout << "\n==============================================\n";


    testCase1();

    testCase2();

    testCase3();

    testCase4();

    testCase5();

    testCase6();


    saveResults();


    cout << "\n\n==============================================";
    cout << "\n        ALL TEST CASES COMPLETED";
    cout << "\n==============================================\n";


    return 0;
}