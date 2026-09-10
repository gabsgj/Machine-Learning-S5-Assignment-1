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

/*
    SAFE SEMANTIC PLANNER
    PCCST503 - Machine Learning
    Assignment 1

    D* Lite-style safe planner for a finite Cartesian state space.
*/


// ============================================================
// STATE
// ============================================================

class State {
public:
    unsigned long long id;
    vector<double> embedding;

    State() {}

    State(unsigned long long id, vector<double> embedding) {
        this->id = id;
        this->embedding = embedding;
    }
};


// ============================================================
// TRANSITION
// ============================================================

class Transition {
public:
    unsigned long long id;
    unsigned long long from;
    unsigned long long to;

    double cost;
    double safety;
    double reliability;

    bool available;

    Transition() {}

    Transition(unsigned long long id,
               unsigned long long from,
               unsigned long long to,
               double cost,
               double safety,
               double reliability,
               bool available = true) {

        this->id = id;
        this->from = from;
        this->to = to;
        this->cost = cost;
        this->safety = safety;
        this->reliability = reliability;
        this->available = available;
    }
};


// ============================================================
// PLANNING PROBLEM
// ============================================================

class PlanningProblem {
public:
    unsigned long long initialState;
    unsigned long long goalState;

    vector<unsigned long long> badStates;
    vector<State> states;
    vector<Transition> transitions;
};


// ============================================================
// PLANNING RESULT
// ============================================================

class PlanningResult {
public:
    bool success;

    vector<unsigned long long> statePath;
    vector<unsigned long long> transitionPath;

    double totalCost;
    double safetyScore;
    double reliability;

    int exploredStates;
    double planningTimeMs;

    size_t approximateMemoryBytes;

    PlanningResult() {
        success = false;
        totalCost = 0.0;
        safetyScore = 0.0;
        reliability = 1.0;
        exploredStates = 0;
        planningTimeMs = 0.0;
        approximateMemoryBytes = 0;
    }
};


// ============================================================
// PRIORITY QUEUE NODE
// ============================================================

struct QueueNode {
    unsigned long long state;

    double k1;
    double k2;

    QueueNode(unsigned long long state,
              double k1,
              double k2) {

        this->state = state;
        this->k1 = k1;
        this->k2 = k2;
    }
};


struct QueueCompare {

    bool operator()(const QueueNode& a,
                    const QueueNode& b) const {

        if (fabs(a.k1 - b.k1) > EPS) {
            return a.k1 > b.k1;
        }

        return a.k2 > b.k2;
    }
};


// ============================================================
// PLANNER CLASS
// ============================================================

class Planner {

private:

    PlanningProblem problem;

    unordered_map<unsigned long long, State> stateMap;

    unordered_map<unsigned long long,
                 vector<Transition> > outgoing;

    unordered_map<unsigned long long,
                 vector<Transition> > incoming;

    unordered_set<unsigned long long> bad;

    unordered_map<unsigned long long, double> g;
    unordered_map<unsigned long long, double> rhs;

    priority_queue<
        QueueNode,
        vector<QueueNode>,
        QueueCompare
    > open;

    unsigned long long goal;

    double km;

    double safetyWeight;
    double reliabilityWeight;


    // --------------------------------------------------------
    // GET G VALUE
    // --------------------------------------------------------

    double getG(unsigned long long s) const {

        unordered_map<unsigned long long, double>::const_iterator it;

        it = g.find(s);

        if (it == g.end()) {
            return INF;
        }

        return it->second;
    }


    // --------------------------------------------------------
    // GET RHS VALUE
    // --------------------------------------------------------

    double getRHS(unsigned long long s) const {

        unordered_map<unsigned long long, double>::const_iterator it;

        it = rhs.find(s);

        if (it == rhs.end()) {
            return INF;
        }

        return it->second;
    }


    // --------------------------------------------------------
    // CHECK BAD STATE
    // --------------------------------------------------------

    bool isBad(unsigned long long s) const {

        return bad.count(s) != 0;
    }


    // --------------------------------------------------------
    // EUCLIDEAN DISTANCE
    // --------------------------------------------------------

    double euclideanDistance(unsigned long long a,
                             unsigned long long b) const {

        unordered_map<unsigned long long, State>::const_iterator ia;
        unordered_map<unsigned long long, State>::const_iterator ib;

        ia = stateMap.find(a);
        ib = stateMap.find(b);

        if (ia == stateMap.end() ||
            ib == stateMap.end()) {

            return INF;
        }

        const vector<double>& x = ia->second.embedding;
        const vector<double>& y = ib->second.embedding;

        if (x.size() != y.size()) {
            return INF;
        }

        double sum = 0.0;

        for (size_t i = 0; i < x.size(); i++) {

            double d = x[i] - y[i];

            sum += d * d;
        }

        return sqrt(sum);
    }


    // --------------------------------------------------------
    // DISTANCE TO NEAREST BAD STATE
    // --------------------------------------------------------

    double distanceToNearestBad(unsigned long long s) const {

        if (bad.empty()) {
            return INF;
        }

        double minimum = INF;

        unordered_set<unsigned long long>::const_iterator it;

        for (it = bad.begin(); it != bad.end(); ++it) {

            double distance =
                euclideanDistance(s, *it);

            minimum = min(minimum, distance);
        }

        return minimum;
    }


    // --------------------------------------------------------
    // HEURISTIC
    // --------------------------------------------------------

    double heuristic(unsigned long long s) const {

        return euclideanDistance(s, goal);
    }


    // --------------------------------------------------------
    // EDGE COST
    // --------------------------------------------------------

    double edgeCost(const Transition& t) const {

        if (!t.available) {
            return INF;
        }

        if (isBad(t.to)) {
            return INF;
        }

        double transitionSafety =
            max(0.0, min(1.0, t.safety));

        double reliability =
            max(0.0, min(1.0, t.reliability));


        double stateSafetyPenalty = 0.0;

        double stateSafetyDistance =
            distanceToNearestBad(t.to);


        if (isfinite(stateSafetyDistance)) {

            stateSafetyPenalty =
                1.0 / (1.0 + stateSafetyDistance);
        }


        double transitionSafetyPenalty =
            1.0 - transitionSafety;

        double reliabilityPenalty =
            1.0 - reliability;


        return
            t.cost
            + safetyWeight * stateSafetyPenalty
            + safetyWeight * transitionSafetyPenalty
            + reliabilityWeight * reliabilityPenalty;
    }


    // --------------------------------------------------------
    // CALCULATE KEY
    // --------------------------------------------------------

    pair<double, double>
    calculateKey(unsigned long long s) const {

        double minimum =
            min(getG(s), getRHS(s));

        return make_pair(
            minimum + heuristic(s) + km,
            minimum
        );
    }


    // --------------------------------------------------------
    // PUSH TO OPEN
    // --------------------------------------------------------

    void pushOpen(unsigned long long s) {

        pair<double, double> key =
            calculateKey(s);

        open.push(
            QueueNode(
                s,
                key.first,
                key.second
            )
        );
    }


    // --------------------------------------------------------
    // UPDATE VERTEX
    // --------------------------------------------------------

    void updateVertex(unsigned long long u) {

        if (u != goal) {

            double best = INF;

            unordered_map<unsigned long long,
                           vector<Transition> >::iterator it;

            it = outgoing.find(u);

            if (it != outgoing.end()) {

                vector<Transition>& list =
                    it->second;

                for (size_t i = 0;
                     i < list.size();
                     i++) {

                    const Transition& t = list[i];

                    double c = edgeCost(t);

                    if (!isfinite(c)) {
                        continue;
                    }

                    double candidate =
                        c + getG(t.to);

                    best =
                        min(best, candidate);
                }
            }

            rhs[u] = best;
        }


        if (fabs(getG(u) - getRHS(u)) > EPS) {

            pushOpen(u);
        }
    }


    // --------------------------------------------------------
    // BEST SUCCESSOR
    // --------------------------------------------------------

    unsigned long long
    bestSuccessor(unsigned long long u) const {

        double bestValue = INF;

        unsigned long long bestState = 0;

        unordered_map<unsigned long long,
                       vector<Transition> >::const_iterator it;

        it = outgoing.find(u);

        if (it == outgoing.end()) {
            return 0;
        }

        const vector<Transition>& list =
            it->second;


        for (size_t i = 0;
             i < list.size();
             i++) {

            const Transition& t = list[i];

            double c = edgeCost(t);

            if (!isfinite(c)) {
                continue;
            }

            double value =
                c + getG(t.to);

            if (value < bestValue) {

                bestValue = value;

                bestState = t.to;
            }
        }

        return bestState;
    }


    // --------------------------------------------------------
    // COMPUTE SHORTEST PATH
    // --------------------------------------------------------

    void computeShortestPath(
        unsigned long long start,
        int& explored) {


        while (!open.empty()) {

            QueueNode current =
                open.top();

            open.pop();


            unsigned long long u =
                current.state;


            pair<double, double> newKey =
                calculateKey(u);


            bool stale =
                current.k1 > newKey.first + EPS
                ||
                (
                    fabs(
                        current.k1 -
                        newKey.first
                    ) <= EPS
                    &&
                    current.k2 >
                    newKey.second + EPS
                );


            if (stale) {
                continue;
            }


            pair<double, double> startKey =
                calculateKey(start);


            bool startConsistent =
                fabs(
                    getG(start) -
                    getRHS(start)
                ) <= EPS;


            if (!open.empty()) {

                QueueNode next =
                    open.top();


                if (
                    next.k1 + EPS >=
                    startKey.first
                    &&
                    (
                        fabs(
                            next.k1 -
                            startKey.first
                        ) > EPS
                        ||
                        next.k2 + EPS >=
                        startKey.second
                    )
                    &&
                    startConsistent
                ) {

                    break;
                }

            } else {

                if (startConsistent) {
                    break;
                }
            }


            explored++;


            if (getG(u) > getRHS(u)) {

                g[u] = getRHS(u);


                unordered_map<
                    unsigned long long,
                    vector<Transition>
                >::iterator it;


                it = incoming.find(u);


                if (it != incoming.end()) {

                    vector<Transition>& list =
                        it->second;


                    for (size_t i = 0;
                         i < list.size();
                         i++) {

                        updateVertex(
                            list[i].from
                        );
                    }
                }

            } else {

                g[u] = INF;

                updateVertex(u);


                unordered_map<
                    unsigned long long,
                    vector<Transition>
                >::iterator it;


                it = incoming.find(u);


                if (it != incoming.end()) {

                    vector<Transition>& list =
                        it->second;


                    for (size_t i = 0;
                         i < list.size();
                         i++) {

                        updateVertex(
                            list[i].from
                        );
                    }
                }
            }
        }
    }


    // --------------------------------------------------------
    // MEMORY ESTIMATION
    // --------------------------------------------------------

    size_t approximateMemoryBytes() const {

        size_t bytes = 0;


        bytes +=
            stateMap.size()
            * sizeof(State);


        bytes +=
            outgoing.size()
            * sizeof(
                vector<Transition>
            );


        bytes +=
            incoming.size()
            * sizeof(
                vector<Transition>
            );


        bytes +=
            bad.size()
            * sizeof(
                unsigned long long
            );


        bytes +=
            g.size()
            * (
                sizeof(
                    unsigned long long
                )
                +
                sizeof(double)
            );


        bytes +=
            rhs.size()
            * (
                sizeof(
                    unsigned long long
                )
                +
                sizeof(double)
            );


        bytes +=
            problem.states.size()
            * sizeof(State);


        bytes +=
            problem.transitions.size()
            * sizeof(Transition);


        return bytes;
    }


    // --------------------------------------------------------
    // EXTRACT PATH
    // --------------------------------------------------------

    PlanningResult
    extractPath(
        unsigned long long start,
        double planningTime,
        int explored) const {


        PlanningResult result;


        result.planningTimeMs =
            planningTime;


        result.exploredStates =
            explored;


        result.approximateMemoryBytes =
            approximateMemoryBytes();


        if (isBad(start) ||
            isBad(goal)) {

            return result;
        }


        if (start == goal) {

            result.success = true;

            result.statePath.push_back(start);

            result.safetyScore =
                distanceToNearestBad(start);

            result.reliability = 1.0;

            return result;
        }


        if (!isfinite(getG(start))) {

            return result;
        }


        unsigned long long current =
            start;


        unordered_set<unsigned long long>
            visited;


        visited.insert(current);

        result.statePath.push_back(
            current
        );


        double minimumSafety =
            distanceToNearestBad(current);


        double cumulativeReliability =
            1.0;


        while (current != goal) {


            unsigned long long next =
                bestSuccessor(current);


            if (next == 0 ||
                isBad(next) ||
                visited.count(next)) {

                PlanningResult failed;

                failed.planningTimeMs =
                    planningTime;

                failed.exploredStates =
                    explored;

                failed.approximateMemoryBytes =
                    approximateMemoryBytes();

                return failed;
            }


            bool found = false;


            unordered_map<
                unsigned long long,
                vector<Transition>
            >::const_iterator it;


            it = outgoing.find(current);


            if (it != outgoing.end()) {

                const vector<Transition>& list =
                    it->second;


                for (size_t i = 0;
                     i < list.size();
                     i++) {


                    const Transition& t =
                        list[i];


                    if (
                        t.to == next
                        &&
                        t.available
                        &&
                        !isBad(t.to)
                    ) {


                        result.transitionPath.push_back(
                            t.id
                        );


                        result.totalCost +=
                            t.cost;


                        cumulativeReliability *=
                            max(
                                0.0,
                                min(
                                    1.0,
                                    t.reliability
                                )
                            );


                        found = true;

                        break;
                    }
                }
            }


            if (!found) {

                PlanningResult failed;

                failed.planningTimeMs =
                    planningTime;

                failed.exploredStates =
                    explored;

                failed.approximateMemoryBytes =
                    approximateMemoryBytes();

                return failed;
            }


            minimumSafety =
                min(
                    minimumSafety,
                    distanceToNearestBad(next)
                );


            current = next;


            result.statePath.push_back(
                current
            );


            visited.insert(current);
        }


        result.success = true;

        result.safetyScore =
            minimumSafety;

        result.reliability =
            cumulativeReliability;


        return result;
    }


public:

    // --------------------------------------------------------
    // CONSTRUCTOR
    // --------------------------------------------------------

    Planner(
        const PlanningProblem& p
    ) {

        problem = p;


        for (size_t i = 0;
             i < problem.states.size();
             i++) {

            stateMap[
                problem.states[i].id
            ] =
                problem.states[i];
        }


        for (size_t i = 0;
             i < problem.badStates.size();
             i++) {

            bad.insert(
                problem.badStates[i]
            );
        }


        for (size_t i = 0;
             i < problem.transitions.size();
             i++) {

            Transition t =
                problem.transitions[i];


            outgoing[t.from].push_back(t);

            incoming[t.to].push_back(t);
        }


        goal =
            problem.goalState;


        km = 0.0;

        safetyWeight = 0.5;

        reliabilityWeight = 0.3;
    }


    // --------------------------------------------------------
    // INITIALIZE
    // --------------------------------------------------------

    void initialize() {

        while (!open.empty()) {
            open.pop();
        }


        g.clear();

        rhs.clear();

        km = 0.0;


        for (size_t i = 0;
             i < problem.states.size();
             i++) {

            unsigned long long id =
                problem.states[i].id;

            g[id] = INF;

            rhs[id] = INF;
        }


        if (!isBad(goal)) {

            rhs[goal] = 0.0;

            pushOpen(goal);
        }
    }


    // --------------------------------------------------------
    // PLAN
    // --------------------------------------------------------

    PlanningResult plan() {

        chrono::high_resolution_clock::time_point
            startTime =
            chrono::high_resolution_clock::now();


        initialize();


        int explored = 0;


        computeShortestPath(
            problem.initialState,
            explored
        );


        chrono::high_resolution_clock::time_point
            endTime =
            chrono::high_resolution_clock::now();


        double elapsed =
            chrono::duration<double, milli>(
                endTime - startTime
            ).count();


        return extractPath(
            problem.initialState,
            elapsed,
            explored
        );
    }


    // --------------------------------------------------------
    // UPDATE GOAL
    // --------------------------------------------------------

    void updateGoal(
        unsigned long long newGoal
    ) {

        goal = newGoal;

        initialize();
    }


    // --------------------------------------------------------
    // UPDATE BAD STATES
    // --------------------------------------------------------

    void updateBadStates(
        const vector<unsigned long long>&
        newBadStates
    ) {

        bad.clear();


        for (size_t i = 0;
             i < newBadStates.size();
             i++) {

            bad.insert(
                newBadStates[i]
            );
        }


        initialize();
    }


    // --------------------------------------------------------
    // CHANGE TRANSITION AVAILABILITY
    // --------------------------------------------------------

    bool setTransitionAvailability(
        unsigned long long transitionID,
        bool available
    ) {


        bool changed = false;


        for (size_t i = 0;
             i < problem.transitions.size();
             i++) {


            if (
                problem.transitions[i].id
                ==
                transitionID
            ) {


                problem.transitions[i].available =
                    available;


                changed = true;

                break;
            }
        }


        if (!changed) {
            return false;
        }


        // Update outgoing transitions

        unordered_map<
            unsigned long long,
            vector<Transition>
        >::iterator outIt;


        for (
            outIt = outgoing.begin();
            outIt != outgoing.end();
            ++outIt
        ) {


            vector<Transition>& list =
                outIt->second;


            for (size_t i = 0;
                 i < list.size();
                 i++) {


                if (
                    list[i].id ==
                    transitionID
                ) {

                    list[i].available =
                        available;
                }
            }
        }


        // Update incoming transitions

        unordered_map<
            unsigned long long,
            vector<Transition>
        >::iterator inIt;


        for (
            inIt = incoming.begin();
            inIt != incoming.end();
            ++inIt
        ) {


            vector<Transition>& list =
                inIt->second;


            for (size_t i = 0;
                 i < list.size();
                 i++) {


                if (
                    list[i].id ==
                    transitionID
                ) {

                    list[i].available =
                        available;
                }
            }
        }


        return true;
    }


    // --------------------------------------------------------
    // ADD TRANSITION
    // --------------------------------------------------------

    void addTransition(
        const Transition& t
    ) {

        problem.transitions.push_back(t);

        outgoing[t.from].push_back(t);

        incoming[t.to].push_back(t);
    }


    // --------------------------------------------------------
    // REMOVE TRANSITION
    // --------------------------------------------------------

    bool removeTransition(
        unsigned long long transitionID
    ) {

        return setTransitionAvailability(
            transitionID,
            false
        );
    }


    // --------------------------------------------------------
    // REPLAN
    // --------------------------------------------------------

    PlanningResult replan() {

        chrono::high_resolution_clock::time_point
            startTime =
            chrono::high_resolution_clock::now();


        /*
            Reinitialize the search after a
            dynamic graph change.
        */

        initialize();


        int explored = 0;


        computeShortestPath(
            problem.initialState,
            explored
        );


        chrono::high_resolution_clock::time_point
            endTime =
            chrono::high_resolution_clock::now();


        double elapsed =
            chrono::duration<double, milli>(
                endTime - startTime
            ).count();


        return extractPath(
            problem.initialState,
            elapsed,
            explored
        );
    }


    // --------------------------------------------------------
    // PRINT GRAPH
    // --------------------------------------------------------

    void printGraph() const {

        cout << endl;

        cout << "Directed Graph" << endl;

        cout << "--------------" << endl;


        unordered_map<
            unsigned long long,
            vector<Transition>
        >::const_iterator it;


        for (
            it = outgoing.begin();
            it != outgoing.end();
            ++it
        ) {


            const vector<Transition>& list =
                it->second;


            for (size_t i = 0;
                 i < list.size();
                 i++) {


                const Transition& t =
                    list[i];


                cout
                    << t.from
                    << " -> "
                    << t.to

                    << " | ID="
                    << t.id

                    << " | cost="
                    << t.cost

                    << " | safety="
                    << t.safety

                    << " | reliability="
                    << t.reliability

                    << " | available="
                    << (
                        t.available
                        ? "YES"
                        : "NO"
                    )

                    << endl;
            }
        }
    }
};


// ============================================================
// PRINT RESULT
// ============================================================

void printResult(
    const string& title,
    const PlanningResult& r
) {


    cout << endl;

    cout << "========================================"
         << endl;

    cout << title << endl;

    cout << "========================================"
         << endl;


    if (!r.success) {

        cout
            << "Success              : NO"
            << endl;

        cout
            << "No safe path found."
            << endl;

        return;
    }


    cout
        << "Success              : YES"
        << endl;


    cout
        << "State Path            : ";


    for (size_t i = 0;
         i < r.statePath.size();
         i++) {


        cout
            << r.statePath[i];


        if (
            i + 1 <
            r.statePath.size()
        ) {

            cout << " -> ";
        }
    }


    cout << endl;


    cout
        << "Transition Path       : ";


    for (size_t i = 0;
         i < r.transitionPath.size();
         i++) {


        cout
            << r.transitionPath[i];


        if (
            i + 1 <
            r.transitionPath.size()
        ) {

            cout << " -> ";
        }
    }


    cout << endl;


    cout
        << fixed
        << setprecision(4);


    cout
        << "Total Path Cost       : "
        << r.totalCost
        << endl;


    cout
        << "Minimum Safety Dist.  : "
        << r.safetyScore
        << endl;


    cout
        << "Cumulative Reliability: "
        << r.reliability
        << endl;


    cout
        << "Explored States       : "
        << r.exploredStates
        << endl;


    cout
        << "Planning Time (ms)    : "
        << r.planningTimeMs
        << endl;


    cout
        << "Approx. Memory (bytes): "
        << r.approximateMemoryBytes
        << endl;
}


// ============================================================
// CREATE ASSIGNMENT PROBLEM
// ============================================================

PlanningProblem createProblem() {

    PlanningProblem p;


    // Initial state

    p.initialState = 0;


    // Goal state

    p.goalState = 3;


    // Bad state

    p.badStates.push_back(6);


    // --------------------------------------------------------
    // STATES
    // --------------------------------------------------------

    p.states.push_back(
        State(0, vector<double>{0, 0})
    );


    p.states.push_back(
        State(1, vector<double>{1, 0})
    );


    p.states.push_back(
        State(2, vector<double>{2, 0})
    );


    p.states.push_back(
        State(3, vector<double>{3, 0})
    );


    p.states.push_back(
        State(4, vector<double>{1, 1})
    );


    p.states.push_back(
        State(5, vector<double>{2, 1})
    );


    // Bad state X

    p.states.push_back(
        State(6, vector<double>{2, -1})
    );


    p.states.push_back(
        State(7, vector<double>{1, 2})
    );


    p.states.push_back(
        State(8, vector<double>{2, 2})
    );


    p.states.push_back(
        State(9, vector<double>{3, 2})
    );


    // --------------------------------------------------------
    // TRANSITIONS
    // --------------------------------------------------------

    // S -> A -> B -> G

    p.transitions.push_back(
        Transition(
            1, 0, 1,
            2.0,
            0.90,
            0.95
        )
    );


    p.transitions.push_back(
        Transition(
            2, 1, 2,
            2.0,
            0.90,
            0.95
        )
    );


    p.transitions.push_back(
        Transition(
            3, 2, 3,
            2.0,
            0.90,
            0.95
        )
    );


    // S -> C -> D -> G

    p.transitions.push_back(
        Transition(
            4, 0, 4,
            2.5,
            0.95,
            0.90
        )
    );


    p.transitions.push_back(
        Transition(
            5, 4, 5,
            2.5,
            0.95,
            0.90
        )
    );


    p.transitions.push_back(
        Transition(
            6, 5, 3,
            2.5,
            0.95,
            0.90
        )
    );


    // Dangerous path:
    //
    // S -> A -> X -> G
    //
    // X is a bad state.

    p.transitions.push_back(
        Transition(
            7, 1, 6,
            1.0,
            0.10,
            0.90
        )
    );


    p.transitions.push_back(
        Transition(
            8, 6, 3,
            1.0,
            0.10,
            0.90
        )
    );


    // Longer safe path

    p.transitions.push_back(
        Transition(
            9, 0, 7,
            3.0,
            0.98,
            0.95
        )
    );


    p.transitions.push_back(
        Transition(
            10, 7, 8,
            3.0,
            0.98,
            0.95
        )
    );


    p.transitions.push_back(
        Transition(
            11, 8, 9,
            3.0,
            0.98,
            0.95
        )
    );


    p.transitions.push_back(
        Transition(
            12, 9, 3,
            3.0,
            0.98,
            0.95
        )
    );


    return p;
}


// ============================================================
// TEST CASE 1
// ============================================================

void testCase1() {

    PlanningProblem p =
        createProblem();


    // Keep only:
    //
    // S -> A -> B -> G

    for (
        size_t i = 0;
        i < p.transitions.size();
        i++
    ) {

        if (p.transitions[i].id >= 4) {

            p.transitions[i].available =
                false;
        }
    }


    Planner planner(p);


    PlanningResult result =
        planner.plan();


    printResult(
        "TEST CASE 1 - BASIC REACHABILITY",
        result
    );
}


// ============================================================
// TEST CASE 2
// ============================================================

void testCase2() {

    PlanningProblem p =
        createProblem();


    Planner planner(p);


    PlanningResult result =
        planner.plan();


    printResult(
        "TEST CASE 2 - BAD STATE AVOIDANCE",
        result
    );


    cout
        << "Expected: state 6 (X) is never visited."
        << endl;
}


// ============================================================
// TEST CASE 3
// ============================================================

void testCase3() {

    PlanningProblem p =
        createProblem();


    // Make B a bad state too.

    p.badStates.push_back(2);


    Planner planner(p);


    PlanningResult result =
        planner.plan();


    printResult(
        "TEST CASE 3 - SAFETY MARGIN",
        result
    );


    cout
        << "Expected: the path avoids both bad states."
        << endl;
}


// ============================================================
// TEST CASE 4
// ============================================================

void testCase4() {

    PlanningProblem p =
        createProblem();


    Planner planner(p);


    PlanningResult initial =
        planner.plan();


    printResult(
        "TEST CASE 4 - INITIAL PLAN",
        initial
    );


    cout
        << endl
        << "Making transition 2 (A -> B) unavailable..."
        << endl;


    planner.setTransitionAvailability(
        2,
        false
    );


    PlanningResult revised =
        planner.replan();


    printResult(
        "TEST CASE 4 - REPLANNED PATH",
        revised
    );
}


// ============================================================
// TEST CASE 5
// ============================================================

void testCase5() {

    PlanningProblem p =
        createProblem();


    Planner planner(p);


    PlanningResult initial =
        planner.plan();


    printResult(
        "TEST CASE 5 - INITIAL GOAL",
        initial
    );


    cout
        << endl
        << "Changing goal from state 3 to state 5..."
        << endl;


    planner.updateGoal(5);


    PlanningResult revised =
        planner.plan();


    printResult(
        "TEST CASE 5 - UPDATED GOAL",
        revised
    );
}


// ============================================================
// TEST CASE 6
// ============================================================

void testCase6() {

    PlanningProblem p =
        createProblem();


    Planner planner(p);


    PlanningResult before =
        planner.plan();


    printResult(
        "TEST CASE 6 - BEFORE SHORTCUT",
        before
    );


    cout
        << endl
        << "Adding shortcut S -> H -> G..."
        << endl;


    // S -> H

    planner.addTransition(
        Transition(
            20,
            0,
            9,
            0.5,
            0.99,
            0.99
        )
    );


    // H -> G

    planner.addTransition(
        Transition(
            21,
            9,
            3,
            0.5,
            0.99,
            0.99
        )
    );


    PlanningResult after =
        planner.replan();


    printResult(
        "TEST CASE 6 - AFTER SHORTCUT",
        after
    );
}


// ============================================================
// RUN ALL TESTS
// ============================================================

void runAllTests() {

    testCase1();

    testCase2();

    testCase3();

    testCase4();

    testCase5();

    testCase6();
}


// ============================================================
// INTERACTIVE MODE
// ============================================================

void interactiveMode() {

    PlanningProblem p =
        createProblem();


    Planner planner(p);


    while (true) {

        cout << endl;

        cout
            << "========================================"
            << endl;

        cout
            << "SAFE SEMANTIC PLANNER - MENU"
            << endl;

        cout
            << "========================================"
            << endl;


        cout
            << "1. Plan path"
            << endl;

        cout
            << "2. Display graph"
            << endl;

        cout
            << "3. Disable transition"
            << endl;

        cout
            << "4. Enable transition"
            << endl;

        cout
            << "5. Change goal"
            << endl;

        cout
            << "6. Add transition"
            << endl;

        cout
            << "7. Run all test cases"
            << endl;

        cout
            << "0. Exit"
            << endl;


        cout
            << "Enter choice: ";


        int choice;

        cin >> choice;


        if (!cin) {
            break;
        }


        if (choice == 0) {
            break;
        }


        // ----------------------------------------------------
        // PLAN
        // ----------------------------------------------------

        if (choice == 1) {

            printResult(
                "PLANNING RESULT",
                planner.plan()
            );
        }


        // ----------------------------------------------------
        // DISPLAY GRAPH
        // ----------------------------------------------------

        else if (choice == 2) {

            planner.printGraph();
        }


        // ----------------------------------------------------
        // ENABLE / DISABLE
        // ----------------------------------------------------

        else if (
            choice == 3 ||
            choice == 4
        ) {


            unsigned long long id;


            cout
                << "Transition ID: ";


            cin >> id;


            bool ok =
                planner.setTransitionAvailability(
                    id,
                    choice == 4
                );


            if (ok) {

                cout
                    << "Transition updated."
                    << endl;

            } else {

                cout
                    << "Transition not found."
                    << endl;
            }
        }


        // ----------------------------------------------------
        // CHANGE GOAL
        // ----------------------------------------------------

        else if (choice == 5) {

            unsigned long long newGoal;


            cout
                << "New goal state: ";


            cin >> newGoal;


            planner.updateGoal(
                newGoal
            );


            cout
                << "Goal updated."
                << endl;
        }


        // ----------------------------------------------------
        // ADD TRANSITION
        // ----------------------------------------------------

        else if (choice == 6) {


            unsigned long long id;

            unsigned long long from;

            unsigned long long to;


            double cost;

            double safety;

            double reliability;


            cout
                << "Transition ID: ";

            cin >> id;


            cout
                << "From state: ";

            cin >> from;


            cout
                << "To state: ";

            cin >> to;


            cout
                << "Cost: ";

            cin >> cost;


            cout
                << "Safety (0-1): ";

            cin >> safety;


            cout
                << "Reliability (0-1): ";

            cin >> reliability;


            planner.addTransition(
                Transition(
                    id,
                    from,
                    to,
                    cost,
                    safety,
                    reliability,
                    true
                )
            );


            cout
                << "Transition added."
                << endl;
        }


        // ----------------------------------------------------
        // RUN TESTS
        // ----------------------------------------------------

        else if (choice == 7) {

            runAllTests();
        }


        else {

            cout
                << "Invalid choice."
                << endl;
        }
    }
}


// ============================================================
// MAIN
// ============================================================

int main() {

    cout
        << "============================================"
        << endl;

    cout
        << " SAFE SEMANTIC PLANNER"
        << endl;

    cout
        << " PCCST503 - Machine Learning"
        << endl;

    cout
        << " Assignment 1"
        << endl;

    cout
        << "============================================"
        << endl;


    // Run all assignment test cases.

    runAllTests();


    /*
        If you want interactive mode, uncomment:

        interactiveMode();
    */


    return 0;
}