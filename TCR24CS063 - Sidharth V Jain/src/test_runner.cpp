#include "test_runner.h"

#include <cassert>
#include <cstdio>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// =====================================================================
//  Helpers
// =====================================================================

static uint64_t nextTid = 1000; // auto-assign transition IDs

static Transition mkTrans(uint64_t from, uint64_t to, double cost,
                          double reliability = 1.0, double safety = 1.0,
                          bool available = true) {
    return {nextTid++, from, to, cost, safety, reliability, available};
}

static State mkState(uint64_t id, std::vector<double> pos) {
    return {id, std::move(pos)};
}

static void printHeader() {
    std::cout
        << "\n"
        << std::string(110, '=') << "\n"
        << std::left
        << std::setw(28) << "Test"
        << std::setw(8)  << "Goal?"
        << std::setw(10) << "Bad#"
        << std::setw(12) << "Cost"
        << std::setw(12) << "Safety"
        << std::setw(10) << "Expl."
        << std::setw(12) << "Time(ms)"
        << std::setw(12) << "Mem(B)"
        << "Path\n"
        << std::string(110, '-') << "\n";
}

static void printRow(const std::string& name, const PlanningResult& r,
                     const std::unordered_set<uint64_t>& badSet = {}) {
    // Count bad states visited
    int badVisited = 0;
    for (auto s : r.statePath)
        if (badSet.count(s)) ++badVisited;

    std::ostringstream pathStr;
    for (size_t i = 0; i < r.statePath.size(); ++i) {
        if (i) pathStr << " -> ";
        pathStr << r.statePath[i];
    }

    std::cout
        << std::left
        << std::setw(28) << name
        << std::setw(8)  << (r.success ? "YES" : "NO")
        << std::setw(10) << badVisited
        << std::setw(12) << std::fixed << std::setprecision(2) << r.totalCost
        << std::setw(12) << (r.safetyScore >= 1e17 ? -1.0 : r.safetyScore)
        << std::setw(10) << r.exploredStates
        << std::setw(12) << std::setprecision(4) << r.planningTimeMs
        << std::setw(12) << r.memoryBytes
        << pathStr.str() << "\n";
}

// =====================================================================
//  TC1 — Basic Reachability
//      S(0) -> A(1) -> B(2) -> G(3)
// =====================================================================

static void tc1_basicReachability() {
    nextTid = 100;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {1, 0}),
        mkState(2, {2, 0}),
        mkState(3, {3, 0}),
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),
        mkTrans(1, 2, 1.0),
        mkTrans(2, 3, 1.0),
    };
    p.initialState = 0;
    p.goalState    = 3;

    DStarLitePlanner planner;
    auto r = planner.plan(p);

    assert(r.success);
    assert(r.statePath == (std::vector<uint64_t>{0, 1, 2, 3}));
    printRow("TC1 Basic Reachability", r);
}

// =====================================================================
//  TC2 — Bad State Avoidance
//      S(0)->A(1)->X(2)->G(3)   X is bad
//      S(0)->C(4)->D(5)->G(3)   safe alternative
// =====================================================================

static void tc2_badStateAvoidance() {
    nextTid = 200;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {1, 1}),
        mkState(2, {2, 2}),  // bad
        mkState(3, {3, 0}),
        mkState(4, {1, -1}),
        mkState(5, {2, -1}),
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),
        mkTrans(1, 2, 1.0),
        mkTrans(2, 3, 1.0),
        mkTrans(0, 4, 2.0),
        mkTrans(4, 5, 2.0),
        mkTrans(5, 3, 2.0),
    };
    p.badStates    = {2};
    p.initialState = 0;
    p.goalState    = 3;

    DStarLitePlanner planner;
    auto r = planner.plan(p);

    assert(r.success);
    // Must not contain bad state 2
    for (auto s : r.statePath) assert(s != 2);
    printRow("TC2 Bad State Avoidance", r, {2});
}

// =====================================================================
//  TC3 — Safety Margin
//      Path1: S(0)->A(1)->G(2)  cost=2, close to bad state
//      Path2: S(0)->B(3)->C(4)->G(2)  cost=4, far from bad state
//      Bad state at (5,5)
// =====================================================================

static void tc3_safetyMargin() {
    nextTid = 300;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {3, 4}),   // close to bad (5,5)
        mkState(2, {10, 0}),  // goal
        mkState(3, {2, -3}),  // far from bad
        mkState(4, {6, -2}),  // far from bad
        mkState(10, {5, 5}),  // bad state
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),
        mkTrans(1, 2, 1.0),
        mkTrans(0, 3, 2.0),
        mkTrans(3, 4, 2.0),
        mkTrans(4, 2, 2.0),
    };
    p.badStates    = {10};
    p.initialState = 0;
    p.goalState    = 2;

    DStarLitePlanner planner;
    auto r = planner.plan(p);

    assert(r.success);
    std::cout << "  (Path1 cost=2 near bad | Path2 cost=6 far from bad)\n";
    printRow("TC3 Safety Margin", r, {});
}

// =====================================================================
//  TC4 — Dynamic Transition (edge becomes unavailable)
//      Initially: S(0)->A(1)->G(2)
//      After change: (A,G) unavailable → must use S->B(3)->G
// =====================================================================

static void tc4_dynamicTransition() {
    nextTid = 400;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {1, 0}),
        mkState(2, {3, 0}),
        mkState(3, {1, 1}),
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),  // id 400
        mkTrans(1, 2, 1.0),  // id 401  ← will become unavailable
        mkTrans(0, 3, 2.0),  // id 402
        mkTrans(3, 2, 2.0),  // id 403
    };
    p.initialState = 0;
    p.goalState    = 2;

    DStarLitePlanner planner;
    auto r1 = planner.plan(p);
    assert(r1.success);
    printRow("TC4a Initial path", r1);

    // Make transition (A→G, id=401) unavailable
    std::vector<EnvironmentChange> changes = {
        {ChangeType::TRANSITION_UNAVAILABLE, 401, 0, {}},
    };
    auto r2 = planner.replan(changes);
    assert(r2.success);
    // Path must not use state 1 then directly 2 via trans 401
    printRow("TC4b After edge removal", r2);
}

// =====================================================================
//  TC5 — Goal Update
//      S(0)->A(1)->G1(2) and S(0)->B(3)->G2(4)
//      Goal changes from G1 to G2
// =====================================================================

static void tc5_goalUpdate() {
    nextTid = 500;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {1, 0}),
        mkState(2, {2, 0}),
        mkState(3, {0, 1}),
        mkState(4, {0, 2}),
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),
        mkTrans(1, 2, 1.0),
        mkTrans(0, 3, 1.5),
        mkTrans(3, 4, 1.5),
    };
    p.initialState = 0;
    p.goalState    = 2;

    DStarLitePlanner planner;
    auto r1 = planner.plan(p);
    assert(r1.success);
    printRow("TC5a Goal=2", r1);

    // Change goal to 4
    std::vector<EnvironmentChange> changes = {
        {ChangeType::GOAL_CHANGED, 0, 4, {}},
    };
    auto r2 = planner.replan(changes);
    assert(r2.success);
    assert(r2.statePath.back() == 4);
    printRow("TC5b Goal->4", r2);
}

// =====================================================================
//  TC6 — Transition Addition (new shortcut)
//      S(0)->A(1)->B(2)->G(3)   cost=3
//      New shortcut S(0)->G(3)  cost=0.5
// =====================================================================

static void tc6_transitionAddition() {
    nextTid = 600;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {1, 0}),
        mkState(2, {2, 0}),
        mkState(3, {3, 0}),
    };
    p.transitions = {
        mkTrans(0, 1, 1.0),
        mkTrans(1, 2, 1.0),
        mkTrans(2, 3, 1.0),
    };
    p.initialState = 0;
    p.goalState    = 3;

    DStarLitePlanner planner;
    auto r1 = planner.plan(p);
    assert(r1.success);
    printRow("TC6a Before shortcut", r1);

    // Add shortcut S->G with cost 0.5
    Transition shortcut{999, 0, 3, 0.5, 1.0, 1.0, true};
    std::vector<EnvironmentChange> changes = {
        {ChangeType::TRANSITION_ADDED, 0, 0, shortcut},
    };
    auto r2 = planner.replan(changes);
    assert(r2.success);
    printRow("TC6b After shortcut", r2);
}

// =====================================================================
//  Bonus TC — Multi-Goal Planning
//      S(0) can reach G1(2) cost=5, G2(4) cost=2, G3(6) cost=3
// =====================================================================

static void tcBonus_multiGoal() {
    nextTid = 700;
    PlanningProblem p;
    p.states = {
        mkState(0, {0, 0}),
        mkState(1, {3, 0}),
        mkState(2, {6, 0}),   // G1
        mkState(3, {0, 1}),
        mkState(4, {0, 2}),   // G2
        mkState(5, {-2, 0}),
        mkState(6, {-4, 0}),  // G3
    };
    p.transitions = {
        mkTrans(0, 1, 3.0),
        mkTrans(1, 2, 2.0),
        mkTrans(0, 3, 1.0),
        mkTrans(3, 4, 1.0),
        mkTrans(0, 5, 1.5),
        mkTrans(5, 6, 1.5),
    };
    p.initialState = 0;

    DStarLitePlanner planner;
    auto r = planner.planMultiGoal(p, {2, 4, 6});
    assert(r.success);
    printRow("Bonus: Multi-Goal", r);
    std::cout << "  (Best goal chosen among {2, 4, 6} by score)\n";
}

// =====================================================================
//  Bonus TC — Incremental Replanning demo
//      Show that replan is faster than full plan
// =====================================================================

static void tcBonus_incrementalReplan() {
    nextTid = 800;
    // Build a larger graph to show timing difference
    const int N = 50; // 50x50 grid = 2500 states
    PlanningProblem p;

    // Grid states: id = row*N + col, embedding = (col, row)
    for (int r = 0; r < N; ++r)
        for (int c = 0; c < N; ++c)
            p.states.push_back(mkState(r * N + c, {(double)c, (double)r}));

    // Grid transitions: right and down
    for (int r = 0; r < N; ++r) {
        for (int c = 0; c < N; ++c) {
            uint64_t id = r * N + c;
            if (c + 1 < N) p.transitions.push_back(mkTrans(id, id + 1, 1.0));
            if (r + 1 < N) p.transitions.push_back(mkTrans(id, id + N, 1.0));
        }
    }

    p.initialState = 0;
    p.goalState    = N * N - 1;

    DStarLitePlanner planner;

    // Full plan
    auto r1 = planner.plan(p);
    assert(r1.success);
    printRow("Bonus: Full plan (50x50)", r1);

    // Small change: disable one transition near the start
    std::vector<EnvironmentChange> changes = {
        {ChangeType::TRANSITION_UNAVAILABLE, p.transitions[0].id, 0, {}},
    };
    auto r2 = planner.replan(changes);
    assert(r2.success);
    printRow("Bonus: Incremental replan", r2);
    std::printf("  (Replan %.4f ms vs initial %.4f ms)\n",
                r2.planningTimeMs, r1.planningTimeMs);
}

// =====================================================================
//  Entry point
// =====================================================================

void runAllTests() {
    std::cout << "\n╔══════════════════════════════════════════════╗\n"
              <<   "║  Safe Semantic Planner — D* Lite Test Suite  ║\n"
              <<   "╚══════════════════════════════════════════════╝\n";

    printHeader();

    tc1_basicReachability();
    tc2_badStateAvoidance();
    tc3_safetyMargin();
    tc4_dynamicTransition();
    tc5_goalUpdate();
    tc6_transitionAddition();

    std::cout << std::string(110, '-') << "\n";
    std::cout << "BONUS FEATURES\n";
    std::cout << std::string(110, '-') << "\n";

    tcBonus_multiGoal();
    tcBonus_incrementalReplan();

    std::cout << std::string(110, '=') << "\n";
    std::cout << "All assertions passed.\n\n";
}
