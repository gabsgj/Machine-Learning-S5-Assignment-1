#include "LPAStarPlanner.h"
#include <iostream>
#include <iomanip>
#include <string>

static void printResult(const std::string& label, const PlanningResult& r) {
    std::cout << "--- " << label << " ---\n";
    std::cout << "  success: " << (r.success ? "true" : "false") << "\n";
    if (r.success) {
        std::cout << "  path: ";
        for (size_t i = 0; i < r.statePath.size(); ++i) {
            std::cout << r.statePath[i] << (i + 1 < r.statePath.size() ? " -> " : "");
        }
        std::cout << "\n";
        std::cout << "  totalCost: " << r.totalCost << "\n";
        std::cout << "  minClearanceToBadState: " << r.safetyScore << "\n";
        std::cout << "  cumulativeReliability: " << r.cumulativeReliability << "\n";
    }
    std::cout << "  statesExplored: " << r.statesExplored << "\n";
    if (r.planningTimeMs >= 0)
        std::cout << "  planningTimeMs: " << std::fixed << std::setprecision(4) << r.planningTimeMs << "\n";
    if (r.replanTimeMs >= 0)
        std::cout << "  replanTimeMs: " << std::fixed << std::setprecision(4) << r.replanTimeMs << "\n";
    std::cout << std::endl;
}

// helper to build a simple 1-D chain of states at positions 0,1,2,...
static State mkState(uint64_t id, std::vector<double> emb) { return State{id, emb}; }
static Transition mkT(uint64_t id, uint64_t from, uint64_t to, double cost,
                       double safety = 1.0, double reliability = 1.0, bool avail = true) {
    return Transition{id, from, to, cost, safety, reliability, avail};
}

// ===================== Test Case 1: Basic Reachability =====================
// S -> A -> B -> G
void testCase1() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}), mkState(4,{3,0}) };
    p.transitions = { mkT(1,1,2,1.0), mkT(2,2,3,1.0), mkT(3,3,4,1.0) };
    p.initialState = 1; p.goalState = 4; p.badStates = {};

    LPAStarPlanner planner;
    auto r = planner.plan(p);
    printResult("Test 1: Basic Reachability", r);
}

// ===================== Test Case 2: Bad State Avoidance =====================
// S->A->X->G (X bad) vs S->C->D->G
void testCase2() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,1}), mkState(3,{2,1}) /*X*/, mkState(4,{3,0}),
                 mkState(5,{1,-1}), mkState(6,{2,-1}) };
    p.transitions = {
        mkT(1,1,2,1.0), mkT(2,2,3,1.0), mkT(3,3,4,1.0),   // via X (bad)
        mkT(4,1,5,1.0), mkT(5,5,6,1.0), mkT(6,6,4,1.0)    // safe detour
    };
    p.initialState = 1; p.goalState = 4; p.badStates = {3};

    LPAStarPlanner planner;
    auto r = planner.plan(p);
    printResult("Test 2: Bad State Avoidance (expect detour via 5,6)", r);
}

// ===================== Test Case 3: Safety Margin =====================
// Path 1: cheap but skims a bad state. Path 2: costlier but far from it.
void testCase3() {
    PlanningProblem p;
    p.states = {
        mkState(1,{0,0}),
        mkState(2,{2,0.3}),   // cheap route waypoint - skims right past the bad state
        mkState(3,{4,0}),     // goal, itself well clear of the bad state
        mkState(4,{0,3}),     // costlier detour waypoints, both well clear
        mkState(5,{4,3}),
        mkState(99,{2,0})     // bad state
    };
    p.transitions = {
        mkT(1,1,2,1.0),  mkT(2,2,3,1.0),                 // cheap (cost 2), passes 0.3 from bad state
        mkT(3,1,4,2.0),  mkT(4,4,5,2.0), mkT(5,5,3,2.0)   // costlier (cost 6), stays 3.6+ away throughout
    };
    p.initialState = 1; p.goalState = 3; p.badStates = {99};

    LPAStarPlanner planner;
    planner.SAFETY_RADIUS = 1.5;  // bad state's danger zone reaches the cheap path's waypoint
    planner.W_SAFETY = 6.0;       // weight safety heavily so it can outweigh the cost difference
    auto r = planner.plan(p);
    printResult("Test 3: Safety Margin (cheap-but-risky vs costly-but-safe)", r);
}

// ===================== Test Case 4: Dynamic Transition =====================
// S->A->G, then (A,G) becomes unavailable; must find alternative.
void testCase4() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}),
                 mkState(4,{1,-1}), mkState(5,{2,-1}) };
    p.transitions = {
        mkT(1,1,2,1.0), mkT(2,2,3,1.0),                  // S->A->G
        mkT(3,1,4,1.0), mkT(4,4,5,1.0), mkT(5,5,3,1.0)   // alternative route
    };
    p.initialState = 1; p.goalState = 3; p.badStates = {};

    LPAStarPlanner planner;
    auto r1 = planner.plan(p);
    printResult("Test 4a: Initial plan (S->A->G)", r1);

    planner.setTransitionAvailability(2, false); // (A,G) removed
    auto r2 = planner.replan();
    printResult("Test 4b: After (A,G) becomes unavailable", r2);
}

// ===================== Test Case 5: Goal Update =====================
void testCase5() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}), mkState(4,{3,0}) };
    p.transitions = { mkT(1,1,2,1.0), mkT(2,2,3,1.0), mkT(3,3,4,1.0) };
    p.initialState = 1; p.goalState = 4; p.badStates = {};

    LPAStarPlanner planner;
    auto r1 = planner.plan(p);
    printResult("Test 5a: Initial goal = state 4", r1);

    planner.setGoal(3); // goal moves to an intermediate state
    auto r2 = planner.replan();
    printResult("Test 5b: Goal updated to state 3", r2);
}

// ===================== Test Case 6: Transition Addition =====================
void testCase6() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}), mkState(4,{3,0}) };
    p.transitions = { mkT(1,1,2,1.0), mkT(2,2,3,1.0), mkT(3,3,4,1.0) }; // S->A->B->G, cost 3
    p.initialState = 1; p.goalState = 4; p.badStates = {};

    LPAStarPlanner planner;
    auto r1 = planner.plan(p);
    printResult("Test 6a: Initial plan (cost 3 via A,B)", r1);

    planner.addTransition(mkT(4,1,4,1.0)); // shortcut S->G directly, cost 1
    auto r2 = planner.replan();
    printResult("Test 6b: After shortcut transition added", r2);
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
