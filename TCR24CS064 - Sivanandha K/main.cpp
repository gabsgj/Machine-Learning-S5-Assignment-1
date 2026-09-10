#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <chrono>

using namespace std;

struct State {
    uint64_t id{};
    vector<double> embedding;
};

struct Transition {
    uint64_t id{};
    uint64_t from{};
    uint64_t to{};
    double cost{};
    double safety{};
    double reliability{};
    bool available{true};
};

struct PlanningProblem {
    uint64_t initialState{};
    uint64_t goalState{};
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

struct PlanningResult {
    bool success{false};
    vector<uint64_t> statePath;
    vector<uint64_t> transitionPath;
    double totalCost{0.0};
    double safetyScore{0.0};
    double minSafetyDistance{0.0};
    double cumulativeReliability{0.0};
    size_t exploredStates{0};
    double planningTimeMs{0.0};
    size_t memoryBytes{0};
    double replanningTimeMs{0.0};
    size_t badStatesVisited{0};
    string message;
};

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

class DStarLitePlanner : public Planner {
private:
    static constexpr double INF = numeric_limits<double>::infinity();
    static constexpr double EPS = 1e-9;

    struct Key {
        double k1;
        double k2;
    };

    struct OpenEntry {
        Key key;
        uint64_t state;
    };

    struct Compare {
        bool operator()(const OpenEntry& a, const OpenEntry& b) const {
            if (fabs(a.key.k1 - b.key.k1) > EPS) return a.key.k1 > b.key.k1;
            if (fabs(a.key.k2 - b.key.k2) > EPS) return a.key.k2 > b.key.k2;
            return a.state > b.state;
        }
    };

    unordered_map<uint64_t, State> states_;
    unordered_map<uint64_t, Transition> transitions_;
    unordered_map<uint64_t, vector<uint64_t>> outgoing_;
    unordered_map<uint64_t, vector<uint64_t>> incoming_;
    unordered_set<uint64_t> bad_;
    unordered_map<uint64_t, double> g_;
    unordered_map<uint64_t, double> rhs_;
    unordered_map<uint64_t, double> safetyDistance_;
    priority_queue<OpenEntry, vector<OpenEntry>, Compare> open_;

    uint64_t start_ = 0;
    uint64_t goal_ = 0;
    double km_ = 0.0;
    double lambdaSafety_ = 2.0;
    double reliabilityWeight_ = 0.25;
    double heuristicScale_ = 0.0;
    size_t expanded_ = 0;

    double value(const unordered_map<uint64_t, double>& m, uint64_t s) const {
        auto it = m.find(s);
        return it == m.end() ? INF : it->second;
    }

    double euclidean(const vector<double>& a, const vector<double>& b) const {
        size_t n = min(a.size(), b.size());
        double sum = 0.0;
        for (size_t i = 0; i < n; ++i) {
            double d = a[i] - b[i];
            sum += d * d;
        }
        return sqrt(sum);
    }

    double heuristic(uint64_t a, uint64_t b) const {
        auto ia = states_.find(a), ib = states_.find(b);
        if (ia == states_.end() || ib == states_.end()) return 0.0;
        return heuristicScale_ * euclidean(ia->second.embedding, ib->second.embedding);
    }

    void computeHeuristicScale() {
        // Scale Euclidean distance so that h remains an admissible lower bound
        // for the effective edge cost. This preserves D* Lite optimality.
        heuristicScale_ = INF;
        for (const auto& [id, t] : transitions_) {
            if (!t.available || bad_.count(t.from) || bad_.count(t.to)) continue;
            auto a = states_.find(t.from), b = states_.find(t.to);
            if (a == states_.end() || b == states_.end()) continue;
            double d = euclidean(a->second.embedding, b->second.embedding);
            if (d > EPS) heuristicScale_ = min(heuristicScale_, effectiveCost(t) / d);
        }
        if (!isfinite(heuristicScale_)) heuristicScale_ = 0.0;
    }

    double effectiveCost(const Transition& t) const {
        // Safety distance is a property of the destination state. Bad states are
        // never expanded, so this cost is only used for safe destinations.
        double d = 0.0;
        auto it = safetyDistance_.find(t.to);
        if (it != safetyDistance_.end()) d = it->second;
        double transitionSafetyPenalty = lambdaSafety_ * (1.0 - max(0.0, min(1.0, t.safety))) / (d + 1.0);
        double reliabilityPenalty = reliabilityWeight_ * (1.0 - max(0.0, min(1.0, t.reliability)));
        return t.cost + transitionSafetyPenalty + reliabilityPenalty;
    }

    Key calculateKey(uint64_t s) const {
        double m = min(value(g_, s),
                       value(rhs_, s));
        return {m + heuristic(start_, s) + km_, m};
    }

    bool keyLess(const Key& a, const Key& b) const {
        if (fabs(a.k1 - b.k1) > EPS) return a.k1 < b.k1;
        return a.k2 < b.k2 - EPS;
    }

    void pushOpen(uint64_t s) { open_.push({calculateKey(s), s}); }

    void computeSafetyDistances() {
        safetyDistance_.clear();
        for (const auto& [id, st] : states_) {
            double best = INF;
            for (uint64_t badId : bad_) {
                auto it = states_.find(badId);
                if (it != states_.end()) best = min(best, euclidean(st.embedding, it->second.embedding));
            }
            safetyDistance_[id] = best;
        }
    }

    void initialize() {
        while (!open_.empty()) open_.pop();
        g_.clear();
        rhs_.clear();
        expanded_ = 0;
        km_ = 0.0;
        rhs_[goal_] = 0.0;
        pushOpen(goal_);
    }

    void updateVertex(uint64_t u) {
        if (u != goal_) {
            double best = INF;
            auto it = outgoing_.find(u);
            if (it != outgoing_.end() && bad_.count(u) == 0) {
                for (uint64_t tid : it->second) {
                    const auto& t = transitions_.at(tid);
                    if (!t.available || bad_.count(t.to)) continue;
                    double gv = value(g_, t.to);
                    if (isfinite(gv)) best = min(best, effectiveCost(t) + gv);
                }
            }
            rhs_[u] = best;
        }
        if (fabs(value(g_, u) - value(rhs_, u)) > EPS) pushOpen(u);
    }

    void computeShortestPath() {
        while (!open_.empty()) {
            OpenEntry top = open_.top();
            Key startKey = calculateKey(start_);
            if (!keyLess(top.key, startKey) && fabs(value(rhs_, start_) - value(g_, start_)) <= EPS) break;
            open_.pop();
            Key newKey = calculateKey(top.state);
            if (keyLess(top.key, newKey)) {
                open_.push({newKey, top.state});
                continue;
            }
            double gs = value(g_, top.state);
            double rs = value(rhs_, top.state);
            ++expanded_;
            if (gs > rs) {
                g_[top.state] = rs;
                auto in = incoming_.find(top.state);
                if (in != incoming_.end()) {
                    for (uint64_t tid : in->second) updateVertex(transitions_.at(tid).from);
                }
            } else {
                g_[top.state] = INF;
                updateVertex(top.state);
                auto in = incoming_.find(top.state);
                if (in != incoming_.end()) {
                    for (uint64_t tid : in->second) updateVertex(transitions_.at(tid).from);
                }
            }
        }
    }

    vector<uint64_t> reconstructTransitions(vector<uint64_t>& statesPath) const {
        vector<uint64_t> transPath;
        if (!isfinite(value(g_, start_))) return transPath;
        uint64_t u = start_;
        statesPath.push_back(u);
        unordered_set<uint64_t> seen;
        while (u != goal_) {
            if (!seen.insert(u).second) { transPath.clear(); return transPath; }
            double best = INF;
            uint64_t bestTid = 0;
            auto out = outgoing_.find(u);
            if (out == outgoing_.end()) { transPath.clear(); return transPath; }
            for (uint64_t tid : out->second) {
                const auto& t = transitions_.at(tid);
                if (!t.available || bad_.count(t.to)) continue;
                double candidate = effectiveCost(t) + value(g_, t.to);
                if (candidate < best - EPS || (fabs(candidate-best) <= EPS && t.id < bestTid)) {
                    best = candidate;
                    bestTid = tid;
                }
            }
            if (bestTid == 0 || !isfinite(best)) { transPath.clear(); return transPath; }
            transPath.push_back(bestTid);
            u = transitions_.at(bestTid).to;
            statesPath.push_back(u);
            if (statesPath.size() > states_.size() + 1) { transPath.clear(); return transPath; }
        }
        return transPath;
    }

    PlanningResult evaluate(const vector<uint64_t>& path, const vector<uint64_t>& transPath,
                            double planningMs, double replanningMs = 0.0) const {
        PlanningResult r;
        r.success = !path.empty() && path.back() == goal_ && transPath.size() + 1 == path.size();
        r.statePath = path;
        r.transitionPath = transPath;
        r.planningTimeMs = planningMs;
        r.replanningTimeMs = replanningMs;
        r.exploredStates = expanded_;
        r.memoryBytes = states_.size() * sizeof(State) + transitions_.size() * sizeof(Transition)
                       + (g_.size() + rhs_.size()) * sizeof(pair<uint64_t,double>);
        if (!r.success) { r.message = "No safe path exists under current transition availability."; return r; }
        r.minSafetyDistance = INF;
        double reliabilitySum = 0.0;
        for (uint64_t sid : path) {
            if (bad_.count(sid)) ++r.badStatesVisited;
            r.minSafetyDistance = min(r.minSafetyDistance, safetyDistance_.at(sid));
        }
        if (path.size() == 1) r.minSafetyDistance = safetyDistance_.at(path.front());
        if (!isfinite(r.minSafetyDistance)) r.minSafetyDistance = -1.0;
        for (uint64_t tid : transPath) {
            const auto& t = transitions_.at(tid);
            r.totalCost += t.cost;
            reliabilitySum += t.reliability;
        }
        r.cumulativeReliability = reliabilitySum;
        r.safetyScore = r.minSafetyDistance;
        r.message = "Safe path found.";
        return r;
    }

public:
    explicit DStarLitePlanner(double lambdaSafety = 2.0, double reliabilityWeight = 0.25)
        : lambdaSafety_(lambdaSafety), reliabilityWeight_(reliabilityWeight) {}

    PlanningResult plan(const PlanningProblem& problem) override {
        auto t0 = chrono::steady_clock::now();
        loadProblem(problem);
        initialize();
        computeShortestPath();
        vector<uint64_t> path;
        vector<uint64_t> trans = reconstructTransitions(path);
        auto t1 = chrono::steady_clock::now();
        double ms = chrono::duration<double, milli>(t1-t0).count();
        return evaluate(path, trans, ms);
    }

    void loadProblem(const PlanningProblem& problem) {
        states_.clear(); transitions_.clear(); outgoing_.clear(); incoming_.clear(); bad_.clear();
        for (const auto& s : problem.states) states_[s.id] = s;
        for (const auto& t : problem.transitions) {
            transitions_[t.id] = t;
            outgoing_[t.from].push_back(t.id);
            incoming_[t.to].push_back(t.id);
        }
        for (uint64_t b : problem.badStates) bad_.insert(b);
        start_ = problem.initialState;
        goal_ = problem.goalState;
        computeSafetyDistances();
        computeHeuristicScale();
    }

    PlanningResult replanAfterTransitionChange(uint64_t transitionId, bool available) {
        auto it = transitions_.find(transitionId);
        if (it == transitions_.end()) {
            PlanningResult r; r.message = "Unknown transition."; return r;
        }
        auto t0 = chrono::steady_clock::now();
        it->second.available = available;
        computeHeuristicScale();
        updateVertex(it->second.from);
        computeShortestPath();
        vector<uint64_t> path;
        vector<uint64_t> trans = reconstructTransitions(path);
        auto t1 = chrono::steady_clock::now();
        double ms = chrono::duration<double, milli>(t1-t0).count();
        return evaluate(path, trans, 0.0, ms);
    }

    PlanningResult replanAfterGoalChange(uint64_t newGoal) {
        if (!states_.count(newGoal) || bad_.count(newGoal)) {
            PlanningResult r; r.message = "New goal is invalid or unsafe."; return r;
        }
        auto t0 = chrono::steady_clock::now();
        goal_ = newGoal;
        // Reuse the existing graph, safety distances, and transition records.
        // Only D* Lite's goal-dependent g/rhs/open state is reinitialized.
        initialize();
        computeShortestPath();
        vector<uint64_t> path;
        vector<uint64_t> trans = reconstructTransitions(path);
        auto t1 = chrono::steady_clock::now();
        double ms = chrono::duration<double, milli>(t1-t0).count();
        return evaluate(path, trans, 0.0, ms);
    }

    PlanningResult replanAfterTransitionAddition(const Transition& t) {
        auto t0 = chrono::steady_clock::now();
        transitions_[t.id] = t;
        outgoing_[t.from].push_back(t.id);
        incoming_[t.to].push_back(t.id);
        computeHeuristicScale();
        updateVertex(t.from);
        computeShortestPath();
        vector<uint64_t> path;
        vector<uint64_t> trans = reconstructTransitions(path);
        auto t1 = chrono::steady_clock::now();
        double ms = chrono::duration<double, milli>(t1-t0).count();
        return evaluate(path, trans, 0.0, ms);
    }
};

static State S(uint64_t id, double x, double y) { return {id, {x,y}}; }
static Transition T(uint64_t id, uint64_t a, uint64_t b, double c, double safety, double rel=1.0, bool av=true) {
    return {id,a,b,c,safety,rel,av};
}

void printResult(const string& name, const PlanningResult& r) {
    cout << "\n=== " << name << " ===\n";
    cout << "Success: " << (r.success ? "YES" : "NO") << "\n";
    cout << "State path: ";
    for (size_t i=0;i<r.statePath.size();++i) cout << (i?" -> ":"") << r.statePath[i];
    cout << "\nTransition path: ";
    for (size_t i=0;i<r.transitionPath.size();++i) cout << (i?" -> ":"") << r.transitionPath[i];
    cout << "\nTotal path cost: " << fixed << setprecision(3) << r.totalCost;
    cout << "\nMinimum safety distance: " << (r.minSafetyDistance < 0 ? string("N/A") : to_string(r.minSafetyDistance));
    cout << "\nCumulative reliability: " << r.cumulativeReliability;
    cout << "\nBad states visited: " << r.badStatesVisited;
    cout << "\nExplored states: " << r.exploredStates;
    cout << "\nPlanning time (ms): " << r.planningTimeMs;
    cout << "\nReplanning time (ms): " << r.replanningTimeMs;
    cout << "\nApprox. memory (bytes): " << r.memoryBytes;
    cout << "\nMessage: " << r.message << "\n";
}

int main() {
    cout << "PCCST503 Assignment 1 - Safe Semantic Planner\n";
    cout << "Algorithm: D* Lite with cost, reliability and safety-aware edge weighting\n";

    // Test Case 1: S -> A -> B -> G
    PlanningProblem p1;
    p1.initialState=1; p1.goalState=4;
    p1.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,3,0)};
    p1.transitions={T(1,1,2,1,10),T(2,2,3,1,10),T(3,3,4,1,10)};
    DStarLitePlanner planner1;
    printResult("Test Case 1: Basic Reachability", planner1.plan(p1));

    // Test Case 2: bad X blocks the cheaper route; S -> C -> D -> G is selected.
    PlanningProblem p2;
    p2.initialState=1; p2.goalState=5; p2.badStates={4};
    p2.states={S(1,0,0),S(2,1,0),S(3,1,1),S(4,2,0),S(5,3,0)};
    p2.transitions={T(1,1,2,1,1),T(2,2,4,1,1),T(3,4,5,1,1),T(4,1,3,1.2,2),T(5,3,5,1.2,2)};
    DStarLitePlanner planner2;
    printResult("Test Case 2: Bad State Avoidance", planner2.plan(p2));

    // Test Case 3: higher-cost path is safer. Safety penalty makes route via C preferred.
    PlanningProblem p3;
    p3.initialState=1; p3.goalState=6; p3.badStates={5};
    p3.states={S(1,0,0),S(2,1,0),S(3,1,3),S(4,2,3),S(5,2,0),S(6,3,1.5)};
    p3.transitions={T(1,1,2,1,0.5),T(2,2,5,1,0.5),T(3,5,6,1,0.5),
                    T(4,1,3,2,3.0),T(5,3,4,1,3.0),T(6,4,6,2,3.0)};
    DStarLitePlanner planner3(5.0);
    printResult("Test Case 3: Safety Margin", planner3.plan(p3));

    // Test Case 4: A->G becomes unavailable; planner replans to S->C->G.
    PlanningProblem p4;
    p4.initialState=1; p4.goalState=4;
    p4.states={S(1,0,0),S(2,1,0),S(3,1,1),S(4,2,0)};
    p4.transitions={T(1,1,2,1,2),T(2,2,4,1,2),T(3,1,3,1.5,2),T(4,3,4,1.5,2)};
    DStarLitePlanner planner4;
    auto first4=planner4.plan(p4);
    printResult("Test Case 4A: Before Transition Failure", first4);
    auto second4=planner4.replanAfterTransitionChange(2,false);
    printResult("Test Case 4B: After A->G Unavailable", second4);

    // Test Case 5: goal changes from G to D; graph is reused.
    PlanningProblem p5;
    p5.initialState=1; p5.goalState=4;
    p5.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,3,0),S(5,1,2)};
    p5.transitions={T(1,1,2,1,3),T(2,2,3,1,3),T(3,3,4,1,3),T(4,1,5,1.2,3),T(5,5,3,1.2,3)};
    DStarLitePlanner planner5;
    printResult("Test Case 5A: Original Goal", planner5.plan(p5));
    auto second5=planner5.replanAfterGoalChange(5);
    printResult("Test Case 5B: Updated Goal", second5);

    // Test Case 6: new shortcut transition is inserted.
    PlanningProblem p6;
    p6.initialState=1; p6.goalState=4;
    p6.states={S(1,0,0),S(2,1,0),S(3,2,0),S(4,3,0)};
    p6.transitions={T(1,1,2,2,3),T(2,2,3,2,3),T(3,3,4,2,3)};
    DStarLitePlanner planner6;
    printResult("Test Case 6A: Before Shortcut", planner6.plan(p6));
    auto second6=planner6.replanAfterTransitionAddition(T(4,1,4,0.8,3,1.0,true));
    printResult("Test Case 6B: After Shortcut Addition", second6);

    return 0;
}
