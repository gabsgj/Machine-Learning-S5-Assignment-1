#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include "PlanningProblem.hpp"
#include "DStarLitePlanner.hpp"

static void printHeader(const std::string& title) {
    std::cout << "\n========================================================\n";
    std::cout << " " << title << "\n";
    std::cout << "========================================================\n";
}

static void printResult(const PlanningResult& r) {
    std::cout << "  Success:            " << (r.success ? "YES" : "NO") << "\n";
    std::cout << "  State path:         ";
    if (r.statePath.empty()) std::cout << "(none)";
    for (size_t i = 0; i < r.statePath.size(); ++i) {
        std::cout << r.statePath[i];
        if (i + 1 < r.statePath.size()) std::cout << " -> ";
    }
    std::cout << "\n";
    std::cout << "  Transition path:    ";
    for (size_t i = 0; i < r.transitionPath.size(); ++i) {
        std::cout << "T" << r.transitionPath[i];
        if (i + 1 < r.transitionPath.size()) std::cout << ", ";
    }
    std::cout << "\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Total raw cost:     " << r.totalCost << "\n";
    std::cout << "  Min dist to bad st: " << r.safetyScore << "\n";
    std::cout << "  Bad states visited: " << r.badStatesVisited << "\n";
    std::cout << "  States expanded:    " << r.statesExpanded << "\n";
    std::cout << "  Peak open-set size: " << r.peakOpenSetSize << "\n";
    std::cout << "  Planning time (ms): " << r.planningTimeMs << "\n";
}

static State S(uint64_t id, std::vector<double> xy) { return State{id, xy}; }
static Transition Tr(uint64_t id, uint64_t from, uint64_t to, double cost,
                       double safety = 1.0, double reliability = 1.0, bool avail = true) {
    return Transition{id, from, to, cost, safety, reliability, avail};
}

// ---------------------------------------------------------------------------
void testCase1() {
    printHeader("TEST CASE 1: Basic Reachability  (S -> A -> B -> G)");
    PlanningProblem p;
    p.states = { S(1,{0,0}), S(2,{1,0}), S(3,{2,0}), S(4,{3,0}) };
    p.initialState = 1; p.goalState = 4;
    p.transitions = { Tr(101,1,2,1.0), Tr(102,2,3,1.0), Tr(103,3,4,1.0) };

    DStarLitePlanner planner;
    auto r = planner.plan(p);
    printResult(r);
    std::cout << "  Expected: unique path 1 -> 2 -> 3 -> 4. "
              << (r.statePath.size() == 4 ? "MATCH.\n" : "MISMATCH.\n");
}

// ---------------------------------------------------------------------------
void testCase2() {
    printHeader("TEST CASE 2: Bad State Avoidance (X is unsafe)");
    PlanningProblem p;
    // S=10, A=11, X=12(bad), G=13, C=14, D=15
    p.states = { S(10,{0,0}), S(11,{1,1}), S(12,{2,1}), S(13,{4,1}),
                  S(14,{1,-1}), S(15,{2,-1}) };
    p.initialState = 10; p.goalState = 13; p.badStates = {12};
    p.transitions = {
        Tr(201,10,11,1.0), Tr(202,11,12,1.0), Tr(203,12,13,1.0), // S->A->X->G (blocked)
        Tr(204,10,14,1.0), Tr(205,14,15,1.0), Tr(206,15,13,1.0)  // S->C->D->G
    };

    DStarLitePlanner planner;
    auto r = planner.plan(p);
    printResult(r);
    bool avoidsX = std::find(r.statePath.begin(), r.statePath.end(), 12) == r.statePath.end();
    std::cout << "  Expected: path via C,D avoiding bad state X(12). "
              << (avoidsX && r.success ? "MATCH.\n" : "MISMATCH.\n");
}

// ---------------------------------------------------------------------------
void testCase3() {
    printHeader("TEST CASE 3: Safety Margin (cheap-but-close vs pricier-but-far)");
    // S=20, G=21; close path P1a/P1b hugs bad state Z; far path P2a/P2b doesn't
    PlanningProblem p;
    p.states = { S(20,{0,0}), S(21,{6,0}),
                  S(22,{2,0.2}), S(23,{4,0.2}),   // close path
                  S(24,{2,3}),   S(25,{4,3}),     // far path
                  S(26,{3,0}) };                   // bad state Z
    p.initialState = 20; p.goalState = 21; p.badStates = {26};
    p.transitions = {
        Tr(301,20,22,1.0), Tr(302,22,23,1.0), Tr(303,23,21,1.0), // cheap, close: total cost 3
        Tr(304,20,24,2.0), Tr(305,24,25,2.0), Tr(306,25,21,2.0)  // pricier, far: total cost 6
    };

    std::cout << "\n-- Run A: cost-dominant weights (costWeight=1.0, marginWeight=0.05) --\n";
    DStarLitePlanner cheapFirst(1.0, 0.0, 0.05, 0.0);
    auto rA = cheapFirst.plan(p);
    printResult(rA);

    std::cout << "\n-- Run B: safety-dominant weights (costWeight=0.3, marginWeight=3.0) --\n";
    DStarLitePlanner safeFirst(0.3, 0.0, 3.0, 0.0);
    auto rB = safeFirst.plan(p);
    printResult(rB);

    std::cout << "\n  Interpretation: Run A minimizes cost and takes the low-margin\n"
                 "  path (small min-distance-to-bad-state). Run B raises the safety\n"
                 "  margin weight (gamma*D term in Score(P)) enough that the planner\n"
                 "  pays extra cost to keep " ;
    std::cout << std::fixed << std::setprecision(2) << rB.safetyScore
              << " units away from the bad state instead of " << rA.safetyScore << ".\n";
}

// ---------------------------------------------------------------------------
void testCase4() {
    printHeader("TEST CASE 4: Dynamic Transition Removal (A->G goes down)");
    PlanningProblem p;
    p.states = { S(30,{0,0}), S(31,{1,0}), S(32,{2,0}), S(33,{1,-2}) };
    p.initialState = 30; p.goalState = 32;
    p.transitions = {
        Tr(401,30,31,1.0), Tr(402,31,32,1.0),          // S->A->G  (cheap, initially chosen)
        Tr(403,30,33,1.0), Tr(404,33,32,3.0)             // S->B->G  (more expensive fallback)
    };

    DStarLitePlanner planner;
    std::cout << "-- Initial plan --\n";
    auto r1 = planner.plan(p);
    printResult(r1);

    std::cout << "\n-- Transition A->G (id 402) becomes unavailable; incremental replan --\n";
    auto r2 = planner.setTransitionAvailability(402, false);
    printResult(r2);
    std::cout << "  Expected: replans via S->B->G. "
              << (r2.success && r2.statePath.back() == 32 && r2.statePath[1] == 33 ? "MATCH.\n" : "MISMATCH.\n");
    std::cout << "  Note the low 'states expanded' count on the incremental call --\n"
                 "  that is D* Lite reusing prior g-values instead of resolving from scratch.\n";
}

// ---------------------------------------------------------------------------
void testCase5() {
    printHeader("TEST CASE 5: Goal Update Mid-Execution");
    PlanningProblem p;
    // S=40 -> A=41 -> G1=42
    // S=40 -> C=43 -> G2=44
    p.states = { S(40,{0,0}), S(41,{1,0}), S(42,{2,0}), S(43,{1,-1}), S(44,{2,-1}) };
    p.initialState = 40; p.goalState = 42;
    p.transitions = {
        Tr(501,40,41,1.0), Tr(502,41,42,1.0),
        Tr(503,40,43,1.0), Tr(504,43,44,1.0)
    };

    DStarLitePlanner planner;
    std::cout << "-- Initial plan toward G1(42) --\n";
    auto r1 = planner.plan(p);
    printResult(r1);

    std::cout << "\n-- Goal changes to G2(44) --\n";
    auto r2 = planner.updateGoal(44);
    printResult(r2);
    std::cout << "  Expected: revised path S->C->G2. "
              << (r2.success && r2.statePath.back() == 44 ? "MATCH.\n" : "MISMATCH.\n");
    std::cout << "  Note: unlike edge-availability changes, a goal change is handled\n"
                 "  by re-initializing g/rhs (see updateGoal() comments) -- this is an\n"
                 "  intentional, documented limitation of D* Lite, not a bug.\n";
}

// ---------------------------------------------------------------------------
void testCase6() {
    printHeader("TEST CASE 6: Transition Addition (new shortcut appears)");
    PlanningProblem p;
    p.states = { S(50,{0,0}), S(51,{1,0}), S(52,{2,0}), S(53,{3,0}), S(54,{4,0}) };
    p.initialState = 50; p.goalState = 54;
    p.transitions = {
        Tr(601,50,51,1.0), Tr(602,51,52,1.0), Tr(603,52,53,1.0), Tr(604,53,54,1.0) // total cost 4
    };

    DStarLitePlanner planner;
    std::cout << "-- Initial plan (long way round, cost 4) --\n";
    auto r1 = planner.plan(p);
    printResult(r1);

    std::cout << "\n-- Shortcut S->G (cost 1.5) is added; incremental replan --\n";
    auto r2 = planner.addTransition(Tr(605,50,54,1.5));
    printResult(r2);
    std::cout << "  Expected: planner now takes the direct shortcut. "
              << (r2.success && r2.statePath.size() == 2 ? "MATCH.\n" : "MISMATCH.\n");
}

// ---------------------------------------------------------------------------
// Bonus: unannounced bad-state discovery, exercising addBadState().
void bonusDynamicObstacle() {
    printHeader("BONUS: New Bad State Discovered Mid-Route");
    PlanningProblem p;
    p.states = { S(70,{0,0}), S(71,{1,0}), S(72,{2,0}), S(73,{1,1}), S(74,{2,1}) };
    p.initialState = 70; p.goalState = 72;
    p.transitions = {
        Tr(701,70,71,1.0), Tr(702,71,72,1.0),   // straight through 71
        Tr(703,70,73,1.5), Tr(704,73,74,1.5), Tr(705,74,72,1.5) // detour, costlier
    };

    DStarLitePlanner planner;
    std::cout << "-- Initial plan --\n";
    auto r1 = planner.plan(p);
    printResult(r1);

    std::cout << "\n-- State 71 is discovered to be a bad state; incremental replan --\n";
    auto r2 = planner.addBadState(71);
    printResult(r2);
    std::cout << "  Expected: reroutes via 73->74 detour, avoiding 71. "
              << (r2.success && std::find(r2.statePath.begin(), r2.statePath.end(), 71) == r2.statePath.end()
                  ? "MATCH.\n" : "MISMATCH.\n");
}

int main() {
    std::cout << "D* Lite Safe Semantic Planner -- Test Suite\n";
    testCase1();
    testCase2();
    testCase3();
    testCase4();
    testCase5();
    testCase6();
    bonusDynamicObstacle();
    std::cout << "\nAll test cases complete.\n";
    return 0;
}
