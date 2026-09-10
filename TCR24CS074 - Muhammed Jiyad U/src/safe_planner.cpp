#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <queue>
#include <random>
#include <sstream>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace std;

struct State {
    uint64_t id{};
    vector<double> embedding;
};

struct Transition {
    uint64_t id{}, from{}, to{};
    double cost{0}, safety{1}, reliability{1};
    bool available{true};
};

struct PlanningProblem {
    uint64_t initialState{}, goalState{};
    vector<uint64_t> badStates;
    vector<State> states;
    vector<Transition> transitions;
};

struct PlanningResult {
    bool success{false};
    vector<uint64_t> statePath, transitionPath;
    double totalCost{0}, safetyScore{0};
    size_t exploredStates{0};
    double planningTimeMs{0};
};

class Planner {
public:
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    virtual ~Planner() = default;
};

/*
 * Safe LPA*: Lifelong Planning A* on a directed finite graph.
 *
 * The planner removes bad states from the traversable graph and assigns
 * each available transition a scalarized objective:
 *
 *   w(e) = cost + safetyPenalty + reliabilityPenalty
 *
 * safetyPenalty grows as the destination gets closer to a bad state.
 * This converts the assignment's multi-objective optimization into a
 * single additive objective while retaining exact avoidance of bad states.
 *
 * Dynamic edge/goal changes are handled incrementally through updateVertex()
 * and repeated ComputeShortestPath(), rather than rebuilding the graph.
 */
class SafeLPAStar : public Planner {
    static constexpr double INF = numeric_limits<double>::infinity();
    static constexpr double EPS = 1e-12;

    struct Key {
        double k1, k2;
    };

    struct NodeKey {
        uint64_t id;
        double k1, k2;
        uint64_t serial;
    };

    struct Cmp {
        bool operator()(const NodeKey& a, const NodeKey& b) const {
            if (a.k1 != b.k1)
                return a.k1 > b.k1;

            if (a.k2 != b.k2)
                return a.k2 > b.k2;

            return a.serial > b.serial;
        }
    };

    unordered_map<uint64_t, State> states_;
    unordered_map<uint64_t, Transition> tr_;

    unordered_map<uint64_t, vector<uint64_t>> out_;
    unordered_map<uint64_t, vector<uint64_t>> in_;

    unordered_set<uint64_t> bad_;

    uint64_t start_ = 0;
    uint64_t goal_ = 0;

    double lambdaSafety_ = 2.0;
    double lambdaReliability_ = 0.25;

    double minBaseCost_ = 0.0;

    unordered_map<uint64_t, double> g_;
    unordered_map<uint64_t, double> rhs_;
    unordered_map<uint64_t, double> hopH_;

    priority_queue<NodeKey, vector<NodeKey>, Cmp> open_;

    uint64_t serial_ = 0;
    size_t explored_ = 0;

    double euclid(uint64_t a, uint64_t b) const {
        const auto& x = states_.at(a).embedding;
        const auto& y = states_.at(b).embedding;

        size_t n = min(x.size(), y.size());

        double s = 0.0;

        for (size_t i = 0; i < n; i++) {
            double d = x[i] - y[i];
            s += d * d;
        }

        return sqrt(s);
    }

    double nearestBadDistance(uint64_t s) const {
        if (bad_.empty())
            return numeric_limits<double>::infinity();

        double d = INF;

        for (auto b : bad_) {
            if (states_.count(b))
                d = min(d, euclid(s, b));
        }

        return d;
    }

    double edgeWeight(uint64_t tid) const {
        const auto& e = tr_.at(tid);

        if (!e.available ||
            bad_.count(e.from) ||
            bad_.count(e.to)) {
            return INF;
        }

        double d = nearestBadDistance(e.to);

        double safetyPenalty =
            isinf(d) ? 0.0 : lambdaSafety_ / (d + 1e-6);

        double reliabilityPenalty =
            lambdaReliability_ *
            (1.0 - max(0.0, min(1.0, e.reliability)));

        double transitionSafetyPenalty =
            lambdaSafety_ *
            (1.0 - max(0.0, min(1.0, e.safety)));

        return max(0.0, e.cost)
             + safetyPenalty
             + transitionSafetyPenalty
             + reliabilityPenalty;
    }

    double heuristic(uint64_t s) const {
        if (s == goal_)
            return 0.0;

        auto it = hopH_.find(s);

        if (it == hopH_.end() || minBaseCost_ <= 0.0)
            return 0.0;

        return it->second * minBaseCost_;
    }

    Key calculateKey(uint64_t u) const {
        double m = min(g_.at(u), rhs_.at(u));

        return {
            m + heuristic(u),
            m
        };
    }

    bool keyLess(const Key& a, const Key& b) const {
        if (a.k1 < b.k1 - EPS)
            return true;

        if (a.k1 > b.k1 + EPS)
            return false;

        return a.k2 < b.k2 - EPS;
    }

    void push(uint64_t u) {
        auto k = calculateKey(u);

        open_.push({
            u,
            k.k1,
            k.k2,
            ++serial_
        });
    }

    double rhsValue(uint64_t u) const {
        if (u == start_)
            return 0.0;

        double best = INF;

        auto it = in_.find(u);

        if (it == in_.end())
            return INF;

        for (auto tid : it->second) {
            const auto& e = tr_.at(tid);

            if (bad_.count(e.from) ||
                bad_.count(e.to)) {
                continue;
            }

            double w = edgeWeight(tid);

            if (isinf(w) ||
                isinf(g_.at(e.from))) {
                continue;
            }

            best = min(best, g_.at(e.from) + w);
        }

        return best;
    }

    void updateVertex(uint64_t u) {
        if (u != start_)
            rhs_[u] = rhsValue(u);

        // Stale queue entries are harmless;
        // keys are checked when popped.
        if (fabs(g_[u] - rhs_[u]) > EPS)
            push(u);
    }

    void computeHopHeuristic() {
        /*
         * Reverse unweighted BFS.
         *
         * Every edge has base cost >= minBaseCost_,
         * therefore hop-count * minBaseCost_ is an
         * admissible lower bound.
         */

        hopH_.clear();

        unordered_map<uint64_t, vector<uint64_t>> rev;

        /*
         * UPDATED:
         * The original version used structured bindings:
         *
         * for(auto &[tid,e] : tr_)
         *
         * This version uses iterators for wider compiler compatibility.
         */
        for (auto it = tr_.begin();
             it != tr_.end();
             ++it) {

            const Transition& e = it->second;

            if (e.available &&
                !bad_.count(e.from) &&
                !bad_.count(e.to)) {

                rev[e.to].push_back(e.from);
            }
        }

        queue<uint64_t> q;

        hopH_[goal_] = 0;

        q.push(goal_);

        while (!q.empty()) {
            auto u = q.front();

            q.pop();

            for (auto p : rev[u]) {
                if (!hopH_.count(p)) {

                    hopH_[p] = hopH_[u] + 1;

                    q.push(p);
                }
            }
        }
    }

    void initialize() {
        g_.clear();
        rhs_.clear();

        while (!open_.empty())
            open_.pop();

        serial_ = 0;
        explored_ = 0;

        minBaseCost_ = INF;

        /*
         * UPDATED:
         * Removed structured binding.
         */
        for (auto it = tr_.begin();
             it != tr_.end();
             ++it) {

            const Transition& e = it->second;

            if (e.available &&
                !bad_.count(e.from) &&
                !bad_.count(e.to)) {

                minBaseCost_ =
                    min(
                        minBaseCost_,
                        max(0.0, e.cost)
                    );
            }
        }

        if (isinf(minBaseCost_))
            minBaseCost_ = 0.0;

        computeHopHeuristic();

        /*
         * UPDATED:
         * Removed structured binding.
         */
        for (auto it = states_.begin();
             it != states_.end();
             ++it) {

            const uint64_t id = it->first;

            g_[id] = INF;
            rhs_[id] = INF;
        }

        if (!bad_.count(start_) &&
            !bad_.count(goal_)) {

            rhs_[start_] = 0.0;

            push(start_);
        }
    }

    void computeShortestPath() {
        while (!open_.empty()) {

            auto nk = open_.top();

            Key top{
                nk.k1,
                nk.k2
            };

            Key goalKey =
                calculateKey(goal_);

            if (!keyLess(top, goalKey) &&
                fabs(rhs_[goal_] - g_[goal_]) <= EPS) {

                break;
            }

            open_.pop();

            uint64_t u = nk.id;

            Key current =
                calculateKey(u);

            // Discard stale queue entries.
            if (fabs(current.k1 - nk.k1) > 1e-9 ||
                fabs(current.k2 - nk.k2) > 1e-9) {

                continue;
            }

            explored_++;

            if (g_[u] > rhs_[u]) {

                g_[u] = rhs_[u];

                for (auto tid : out_[u]) {
                    updateVertex(
                        tr_.at(tid).to
                    );
                }

            } else {

                g_[u] = INF;

                updateVertex(u);

                for (auto tid : out_[u]) {
                    updateVertex(
                        tr_.at(tid).to
                    );
                }
            }
        }
    }

    bool traversableState(uint64_t s) const {
        return states_.count(s) &&
               !bad_.count(s);
    }

public:

    SafeLPAStar(
        double ls = 2.0,
        double lr = 0.25
    )
        : lambdaSafety_(ls),
          lambdaReliability_(lr) {
    }

    void load(const PlanningProblem& p) {

        states_.clear();
        tr_.clear();
        out_.clear();
        in_.clear();
        bad_.clear();

        for (auto& s : p.states)
            states_[s.id] = s;

        for (auto& e : p.transitions) {

            tr_[e.id] = e;

            out_[e.from].push_back(e.id);

            in_[e.to].push_back(e.id);
        }

        bad_.insert(
            p.badStates.begin(),
            p.badStates.end()
        );

        start_ = p.initialState;
        goal_ = p.goalState;

        initialize();
    }

    // Change availability of one edge incrementally.
    void setTransitionAvailable(
        uint64_t tid,
        bool available
    ) {

        auto it = tr_.find(tid);

        if (it == tr_.end())
            return;

        it->second.available = available;

        updateVertex(
            it->second.to
        );

        computeHopHeuristic();

        // Affected descendants are propagated
        // by ComputeShortestPath().
    }

    void setGoal(uint64_t newGoal) {

        goal_ = newGoal;

        computeHopHeuristic();

        /*
         * Existing g/rhs values remain valid.
         * Only the termination target changes.
         */

        push(goal_);
    }

    void setBadStates(
        const vector<uint64_t>& badStates
    ) {

        bad_.clear();

        bad_.insert(
            badStates.begin(),
            badStates.end()
        );

        computeHopHeuristic();

        /*
         * UPDATED:
         * Removed structured binding.
         */
        for (auto it = tr_.begin();
             it != tr_.end();
             ++it) {

            const Transition& e = it->second;

            if (bad_.count(e.from) ||
                bad_.count(e.to) ||
                (!bad_.count(e.from) &&
                 !bad_.count(e.to))) {

                updateVertex(e.to);
            }
        }

        push(start_);
    }

    void addTransition(
        const Transition& e
    ) {

        tr_[e.id] = e;

        out_[e.from].push_back(e.id);

        in_[e.to].push_back(e.id);

        if (!bad_.count(e.from) &&
            !bad_.count(e.to)) {

            updateVertex(e.to);
        }

        computeHopHeuristic();
    }

    PlanningResult currentPlan() {

        PlanningResult r;

        if (!traversableState(start_) ||
            !traversableState(goal_)) {

            return r;
        }

        computeShortestPath();

        if (isinf(g_[goal_]))
            return r;

        r.success = true;

        uint64_t u = goal_;

        r.statePath.push_back(u);

        unordered_set<uint64_t> seen;

        while (u != start_) {

            if (seen.count(u)) {

                r.success = false;

                r.statePath.clear();
                r.transitionPath.clear();

                return r;
            }

            seen.insert(u);

            double best = INF;

            uint64_t bestTid = 0;

            for (auto tid : in_[u]) {

                double w =
                    edgeWeight(tid);

                auto& e =
                    tr_.at(tid);

                if (isinf(w) ||
                    isinf(g_[e.from])) {

                    continue;
                }

                double v =
                    g_[e.from] + w;

                if (v < best) {

                    best = v;

                    bestTid = tid;
                }
            }

            if (!bestTid) {

                r.success = false;

                return r;
            }

            auto& e =
                tr_.at(bestTid);

            r.transitionPath.push_back(
                bestTid
            );

            u = e.from;

            r.statePath.push_back(u);
        }

        reverse(
            r.statePath.begin(),
            r.statePath.end()
        );

        reverse(
            r.transitionPath.begin(),
            r.transitionPath.end()
        );

        r.totalCost = 0.0;

        double minD = INF;

        for (size_t i = 0;
             i < r.transitionPath.size();
             i++) {

            auto& e =
                tr_.at(
                    r.transitionPath[i]
                );

            r.totalCost += e.cost;

            minD =
                min(
                    minD,
                    nearestBadDistance(e.to)
                );
        }

        r.safetyScore =
            isinf(minD)
                ? numeric_limits<double>::infinity()
                : minD;

        r.exploredStates =
            explored_;

        return r;
    }

    PlanningResult plan(
        const PlanningProblem& p
    ) override {

        auto t0 =
            chrono::steady_clock::now();

        load(p);

        auto r =
            currentPlan();

        auto t1 =
            chrono::steady_clock::now();

        r.planningTimeMs =
            chrono::duration<double, milli>(
                t1 - t0
            ).count();

        return r;
    }
};


static void printResult(
    const string& name,
    const PlanningResult& r
) {

    cout << "\n"
         << name
         << "\n";

    cout << "success="
         << (r.success ? "true" : "false")
         << "\n";

    cout << "states=";

    for (size_t i = 0;
         i < r.statePath.size();
         ++i) {

        if (i)
            cout << " -> ";

        cout << r.statePath[i];
    }

    cout << "\ntransitions=";

    for (size_t i = 0;
         i < r.transitionPath.size();
         ++i) {

        if (i)
            cout << " -> ";

        cout << r.transitionPath[i];
    }

    cout << "\ntotalCost="
         << fixed
         << setprecision(4)
         << r.totalCost

         << " minBadDistance="
         << r.safetyScore

         << " explored="
         << r.exploredStates

         << " timeMs="
         << r.planningTimeMs

         << "\n";
}


static State S(
    uint64_t id,
    double x,
    double y
) {

    return {
        id,
        {x, y}
    };
}


static Transition E(
    uint64_t id,
    uint64_t a,
    uint64_t b,
    double c,
    double s = 1,
    double rel = 1
) {

    return {
        id,
        a,
        b,
        c,
        s,
        rel,
        true
    };
}


static PlanningProblem basic() {

    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 4;

    p.states = {
        S(1, 0, 0),
        S(2, 1, 0),
        S(3, 2, 0),
        S(4, 3, 0)
    };

    p.transitions = {
        E(1, 1, 2, 1),
        E(2, 2, 3, 1),
        E(3, 3, 4, 1)
    };

    return p;
}


static PlanningProblem badAvoid() {

    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 6;

    p.states = {
        S(1, 0, 0),
        S(2, 1, 0),
        S(3, 2, 0),
        S(4, 1, 1),
        S(5, 2, 1),
        S(6, 3, 0)
    };

    p.badStates = {
        3
    };

    p.transitions = {
        E(1, 1, 2, 1),
        E(2, 2, 3, 1),
        E(3, 3, 6, 1),

        E(4, 1, 4, 1.1),
        E(5, 4, 5, 1),
        E(6, 5, 6, 1)
    };

    return p;
}


static PlanningProblem safetyMargin() {

    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 6;

    p.badStates = {
        8
    };

    p.states = {
        S(1, 0, 0),
        S(2, 1, 0),
        S(3, 2, 0),
        S(4, 3, 0),
        S(5, 1, 2),
        S(6, 3, 2),
        S(8, 2, 1)
    };

    p.transitions = {
        E(1, 1, 2, 1),
        E(2, 2, 3, 1),
        E(3, 3, 4, 1),
        E(4, 4, 6, 1),

        E(5, 1, 5, 1.4),
        E(6, 5, 6, 1.4)
    };

    return p;
}


static PlanningProblem dynamicTransition() {

    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 4;

    p.states = {
        S(1, 0, 0),
        S(2, 1, 0),
        S(3, 1, 1),
        S(4, 2, 0)
    };

    p.transitions = {
        E(1, 1, 2, 1),
        E(2, 2, 4, 1),
        E(3, 1, 3, 1.2),
        E(4, 3, 4, 1.2)
    };

    return p;
}


static PlanningProblem goalUpdate() {

    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 4;

    p.states = {
        S(1, 0, 0),
        S(2, 1, 0),
        S(3, 2, 0),
        S(4, 3, 0),
        S(5, 1, 1)
    };

    p.transitions = {
        E(1, 1, 2, 1),
        E(2, 2, 3, 1),
        E(3, 3, 4, 1),

        E(4, 1, 5, 0.5),
        E(5, 5, 3, 0.5)
    };

    return p;
}


int main() {

    SafeLPAStar planner(
        0.0,
        0.0
    );

    printResult(
        "Test 1: Basic Reachability",
        planner.plan(basic())
    );

    printResult(
        "Test 2: Bad State Avoidance",
        planner.plan(badAvoid())
    );


    SafeLPAStar safe(
        2.0,
        0.0
    );

    printResult(
        "Test 3: Safety Margin",
        safe.plan(safetyMargin())
    );


    SafeLPAStar dyn(
        0.0,
        0.0
    );

    auto p =
        dynamicTransition();

    auto r1 =
        dyn.plan(p);

    printResult(
        "Test 4a: Before transition removal",
        r1
    );

    dyn.setTransitionAvailable(
        2,
        false
    );

    auto r2 =
        dyn.currentPlan();

    printResult(
        "Test 4b: After (2,4) becomes unavailable",
        r2
    );


    SafeLPAStar goal(
        0.0,
        0.0
    );

    auto pg =
        goalUpdate();

    auto rg =
        goal.plan(pg);

    printResult(
        "Test 5a: Original goal",
        rg
    );

    goal.setGoal(5);

    auto rg2 =
        goal.currentPlan();

    printResult(
        "Test 5b: Updated goal",
        rg2
    );


    SafeLPAStar add(
        0.0,
        0.0
    );

    auto pa =
        basic();

    // Remove 3 -> 4 so initially there is no route to 4.
    pa.transitions.pop_back();

    auto ra =
        add.plan(pa);

    printResult(
        "Test 6a: Before shortcut",
        ra
    );

    // Add shortcut 2 -> 4.
    add.addTransition(
        E(99, 2, 4, 0.2)
    );

    auto ra2 =
        add.currentPlan();

    printResult(
        "Test 6b: After shortcut insertion",
        ra2
    );

    return 0;
}
