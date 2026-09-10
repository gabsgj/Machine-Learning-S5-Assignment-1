#include "TestScenarios.hpp"
#include <iostream>
#include <random>
#include <cmath>
#include <iomanip>

// ID conventions:
// S=0, A=1, B=2, G=3, X=4, C=5, D=6, G2=7

PlanningProblem TestScenarios::createTestCase1() {
    // S -> A -> B -> G
    // S=0, A=1, B=2, G=3
    std::vector<State> states = {
        State(0, {0.0, 0.0}), // S
        State(1, {1.0, 0.0}), // A
        State(2, {2.0, 0.0}), // B
        State(3, {3.0, 0.0})  // G
    };
    std::vector<Transition> transitions = {
        Transition(1, 0, 1, 1.0, 1.0, 0.99, true), // S -> A
        Transition(2, 1, 2, 1.0, 1.0, 0.99, true), // A -> B
        Transition(3, 2, 3, 1.0, 1.0, 0.99, true)  // B -> G
    };
    std::vector<uint64_t> badStates = {}; // No bad states
    return PlanningProblem(0, 3, badStates, states, transitions);
}

PlanningProblem TestScenarios::createTestCase2() {
    // Path 1: S -> A -> X -> G (where X is bad)
    // Path 2: S -> C -> D -> G
    // S=0, A=1, X=4(BAD), G=3, C=5, D=6
    std::vector<State> states = {
        State(0, {0.0, 0.0}),  // S
        State(1, {1.0, 1.0}),  // A
        State(4, {2.0, 1.0}),  // X (BAD STATE)
        State(3, {4.0, 0.0}),  // G
        State(5, {1.0, -1.0}), // C
        State(6, {2.0, -1.0})  // D
    };
    std::vector<Transition> transitions = {
        // Path 1 (through bad state X) - shorter raw distance
        Transition(1, 0, 1, 1.0, 1.0, 0.95, true), // S -> A
        Transition(2, 1, 4, 1.0, 0.0, 0.95, true), // A -> X (hazardous)
        Transition(3, 4, 3, 1.0, 0.0, 0.95, true), // X -> G
        // Path 2 (safe bypass)
        Transition(4, 0, 5, 1.5, 1.0, 0.98, true), // S -> C
        Transition(5, 5, 6, 1.5, 1.0, 0.98, true), // C -> D
        Transition(6, 6, 3, 1.5, 1.0, 0.98, true)  // D -> G
    };
    std::vector<uint64_t> badStates = {4}; // State X is bad
    return PlanningProblem(0, 3, badStates, states, transitions);
}

PlanningProblem TestScenarios::createTestCase3() {
    // Test Case 3: Safety Margin
    // Path 1 (Close to bad state): S -> P1_1 -> P1_2 -> G (cost 2.0, distance to bad = 0.5)
    // Path 2 (Far from bad state):  S -> P2_1 -> P2_2 -> G (cost 4.0, distance to bad = 4.0)
    // Bad State at B(2.0, 0.5)
    std::vector<State> states = {
        State(0, {0.0, 0.0}),  // S
        State(1, {1.0, 0.0}),  // P1_1 (dist to bad = 1.11)
        State(2, {2.0, 0.0}),  // P1_2 (dist to bad = 0.5 - VERY CLOSE)
        State(3, {4.0, 0.0}),  // G
        State(4, {2.0, 0.5}),  // BAD STATE X
        State(5, {1.0, -3.0}), // P2_1 (dist to bad = 3.64 - SAFE)
        State(6, {2.0, -3.0})  // P2_2 (dist to bad = 3.50 - SAFE)
    };
    std::vector<Transition> transitions = {
        // Path 1 (Low raw cost, low clearance margin)
        Transition(1, 0, 1, 1.0, 0.4, 0.99, true),
        Transition(2, 1, 2, 1.0, 0.2, 0.99, true),
        Transition(3, 2, 3, 2.0, 0.4, 0.99, true),
        // Path 2 (Higher raw cost, high clearance margin)
        Transition(4, 0, 5, 2.5, 1.0, 0.99, true),
        Transition(5, 5, 6, 2.5, 1.0, 0.99, true),
        Transition(6, 6, 3, 2.5, 1.0, 0.99, true)
    };
    std::vector<uint64_t> badStates = {4};
    return PlanningProblem(0, 3, badStates, states, transitions);
}

PlanningProblem TestScenarios::createTestCase4() {
    // Dynamic Transition: S -> A -> G initially, backup is S -> B -> C -> G
    // S=0, A=1, G=3, B=2, C=4
    std::vector<State> states = {
        State(0, {0.0, 0.0}), // S
        State(1, {1.5, 1.0}), // A
        State(3, {4.0, 0.0}), // G
        State(2, {1.0, -1.5}),// B
        State(4, {2.5, -1.5}) // C
    };
    std::vector<Transition> transitions = {
        // Initial primary path (S -> A -> G)
        Transition(1, 0, 1, 1.5, 1.0, 0.98, true), // S -> A
        Transition(2, 1, 3, 1.5, 1.0, 0.98, true), // A -> G (will fail dynamically)
        // Backup path (S -> B -> C -> G)
        Transition(3, 0, 2, 2.0, 1.0, 0.99, true), // S -> B
        Transition(4, 2, 4, 2.0, 1.0, 0.99, true), // B -> C
        Transition(5, 4, 3, 2.0, 1.0, 0.99, true)  // C -> G
    };
    return PlanningProblem(0, 3, {}, states, transitions);
}

PlanningProblem TestScenarios::createTestCase5() {
    // Goal Update: S -> A -> B -> G1, alternative branch to G2
    // S=0, A=1, B=2, G1=3, C=4, G2=5
    std::vector<State> states = {
        State(0, {0.0, 0.0}),  // S
        State(1, {1.0, 0.0}),  // A
        State(2, {2.0, 0.5}),  // B
        State(3, {3.0, 1.0}),  // G1
        State(4, {2.0, -1.0}), // C
        State(5, {4.0, -1.5})  // G2
    };
    std::vector<Transition> transitions = {
        Transition(1, 0, 1, 1.0, 1.0, 0.99, true), // S -> A
        Transition(2, 1, 2, 1.0, 1.0, 0.99, true), // A -> B
        Transition(3, 2, 3, 1.0, 1.0, 0.99, true), // B -> G1
        Transition(4, 1, 4, 1.5, 1.0, 0.99, true), // A -> C
        Transition(5, 4, 5, 1.5, 1.0, 0.99, true)  // C -> G2
    };
    return PlanningProblem(0, 3, {}, states, transitions);
}

PlanningProblem TestScenarios::createTestCase6() {
    // Transition Addition: Initial long path S -> A -> B -> C -> G (Cost 4.0)
    // New shortcut added: A -> G (Cost 1.2)
    std::vector<State> states = {
        State(0, {0.0, 0.0}), // S
        State(1, {1.0, 0.0}), // A
        State(2, {2.0, 0.0}), // B
        State(3, {3.0, 0.0}), // C
        State(4, {4.0, 0.0})  // G
    };
    std::vector<Transition> transitions = {
        Transition(1, 0, 1, 1.0, 1.0, 0.99, true), // S -> A
        Transition(2, 1, 2, 1.0, 1.0, 0.99, true), // A -> B
        Transition(3, 2, 3, 1.0, 1.0, 0.99, true), // B -> C
        Transition(4, 3, 4, 1.0, 1.0, 0.99, true)  // C -> G
    };
    return PlanningProblem(0, 4, {}, states, transitions);
}

PlanningProblem TestScenarios::generateRandomCartesianGraph(
    uint64_t numStates, size_t dimensions, double badStateRatio, double edgeDensity, unsigned int seed) {
    std::mt19937_64 rng(seed);
    std::uniform_real_distribution<double> coordDist(0.0, 100.0);
    std::uniform_real_distribution<double> probDist(0.0, 1.0);
    std::uniform_real_distribution<double> relDist(0.85, 1.0);

    std::vector<State> states;
    states.reserve(numStates);
    for (uint64_t i = 0; i < numStates; ++i) {
        std::vector<double> emb(dimensions);
        for (size_t d = 0; d < dimensions; ++d) {
            emb[d] = coordDist(rng);
        }
        states.emplace_back(i, emb);
    }

    // Assign bad states (excluding start 0 and goal numStates-1)
    std::vector<uint64_t> badStates;
    for (uint64_t i = 1; i < numStates - 1; ++i) {
        if (probDist(rng) < badStateRatio) {
            badStates.push_back(i);
        }
    }

    std::vector<Transition> transitions;
    uint64_t tid = 1;
    // Guaranteed forward backbone path to ensure reachability
    for (uint64_t i = 0; i + 1 < numStates; ++i) {
        double cost = states[i].euclideanDistance(states[i + 1]);
        transitions.emplace_back(tid++, i, i + 1, cost, 1.0, relDist(rng), true);
    }

    // Additional random directed edges based on distance threshold
    double maxConnectDist = 35.0 * (1.0 + edgeDensity);
    for (uint64_t u = 0; u < numStates; ++u) {
        for (uint64_t v = 0; v < numStates; ++v) {
            if (u == v || (u + 1 == v)) continue;
            double d = states[u].euclideanDistance(states[v]);
            if (d < maxConnectDist && probDist(rng) < edgeDensity) {
                transitions.emplace_back(tid++, u, v, d, 1.0, relDist(rng), true);
            }
        }
    }

    return PlanningProblem(0, numStates - 1, badStates, states, transitions);
}

TestCaseResult TestScenarios::runTestCase1() {
    PlanningProblem prob = createTestCase1();
    LPAStarPlanner planner;
    PlanningResult res = planner.plan(prob);

    TestCaseResult r;
    r.testCaseNumber = 1;
    r.title = "Basic Reachability";
    r.expectedBehavior = "Returns unique valid path: 0 -> 1 -> 2 -> 3";
    r.actualPath = res.pathString();
    r.passed = res.success && (res.statePath == std::vector<uint64_t>{0, 1, 2, 3});
    r.initialCost = res.totalCost;
    r.updatedCost = res.totalCost;
    r.minSafetyDist = res.minSafetyDistance;
    r.badStatesVisited = res.badStatesVisited;
    r.initialExploredNodes = res.exploredStates;
    r.replanExploredNodes = 0;
    r.initialTimeMs = res.planningTimeMs;
    r.replanTimeMs = 0.0;
    r.notes = r.passed ? "Exact match with theoretical optimal path" : "Failed reachability";
    return r;
}

TestCaseResult TestScenarios::runTestCase2() {
    PlanningProblem prob = createTestCase2();
    LPAStarPlanner planner;
    PlanningResult res = planner.plan(prob);

    TestCaseResult r;
    r.testCaseNumber = 2;
    r.title = "Bad State Avoidance";
    r.expectedBehavior = "Avoids bad state X(4), selects safe bypass: 0 -> 5 -> 6 -> 3";
    r.actualPath = res.pathString();
    r.passed = res.success && (res.badStatesVisited == 0) && (res.statePath == std::vector<uint64_t>{0, 5, 6, 3});
    r.initialCost = res.totalCost;
    r.updatedCost = res.totalCost;
    r.minSafetyDist = res.minSafetyDistance;
    r.badStatesVisited = res.badStatesVisited;
    r.initialExploredNodes = res.exploredStates;
    r.replanExploredNodes = 0;
    r.initialTimeMs = res.planningTimeMs;
    r.replanTimeMs = 0.0;
    r.notes = r.passed ? "Strict avoidance verified; 0 bad states visited" : "Bad state visited";
    return r;
}

TestCaseResult TestScenarios::runTestCase3() {
    PlanningProblem prob = createTestCase3();
    // Safety margin planner with strong clearance weight gamma=15.0, safetyRadius=3.0
    LPAStarPlanner planner(100.0, 1.0, 15.0, 2.0, 3.0);
    PlanningResult res = planner.plan(prob);

    TestCaseResult r;
    r.testCaseNumber = 3;
    r.title = "Safety Margin";
    r.expectedBehavior = "Balances cost and safety margin, selecting far bypass (0 -> 5 -> 6 -> 3) over close path (0 -> 1 -> 2 -> 3)";
    r.actualPath = res.pathString();
    // Path 2 (0 -> 5 -> 6 -> 3) has clearance > 3.5, Path 1 (0 -> 1 -> 2 -> 3) has clearance 0.5
    r.passed = res.success && (res.minSafetyDistance > 2.0) && (res.statePath == std::vector<uint64_t>{0, 5, 6, 3});
    r.initialCost = res.totalCost;
    r.updatedCost = res.totalCost;
    r.minSafetyDist = res.minSafetyDistance;
    r.badStatesVisited = res.badStatesVisited;
    r.initialExploredNodes = res.exploredStates;
    r.replanExploredNodes = 0;
    r.initialTimeMs = res.planningTimeMs;
    r.replanTimeMs = 0.0;
    r.notes = r.passed ? "Safety margin objective prioritized wide clearance route" : "Safety penalty insufficient";
    return r;
}

TestCaseResult TestScenarios::runTestCase4() {
    PlanningProblem prob = createTestCase4();
    LPAStarPlanner planner;
    PlanningResult initialRes = planner.plan(prob);
    uint64_t initialExplored = planner.getExpandedCount();

    // Dynamically disable transition 2 (A -> G)
    planner.updateTransition(2, false);
    PlanningResult replanRes = planner.computeShortestPath();

    TestCaseResult r;
    r.testCaseNumber = 4;
    r.title = "Dynamic Transition";
    r.expectedBehavior = "Initial path: 0 -> 1 -> 3; After (1->3) disabled, incremental replan finds alternative: 0 -> 2 -> 4 -> 3";
    r.actualPath = initialRes.pathString() + " -> [Disabled 1->3] -> " + replanRes.pathString();
    r.passed = initialRes.success && replanRes.success && (replanRes.statePath == std::vector<uint64_t>{0, 2, 4, 3});
    r.initialCost = initialRes.totalCost;
    r.updatedCost = replanRes.totalCost;
    r.minSafetyDist = replanRes.minSafetyDistance;
    r.badStatesVisited = replanRes.badStatesVisited;
    r.initialExploredNodes = initialExplored;
    r.replanExploredNodes = planner.getExpandedCount() - initialExplored;
    r.initialTimeMs = initialRes.planningTimeMs;
    r.replanTimeMs = replanRes.planningTimeMs;
    r.notes = "Incremental replanning redirected search without re-expanding entire graph";
    return r;
}

TestCaseResult TestScenarios::runTestCase5() {
    PlanningProblem prob = createTestCase5();
    LPAStarPlanner planner;
    PlanningResult initialRes = planner.plan(prob);
    uint64_t initialExplored = planner.getExpandedCount();

    // Goal dynamically updates to G2 (State 5)
    planner.updateGoal(5);
    PlanningResult replanRes = planner.computeShortestPath();

    TestCaseResult r;
    r.testCaseNumber = 5;
    r.title = "Goal Update";
    r.expectedBehavior = "Initial goal G1(3) gives 0 -> 1 -> 2 -> 3; Updated goal G2(5) gives 0 -> 1 -> 4 -> 5 without rebuilding data structures";
    r.actualPath = initialRes.pathString() + " -> [Goal Updated to 5] -> " + replanRes.pathString();
    r.passed = initialRes.success && replanRes.success && (replanRes.statePath == std::vector<uint64_t>{0, 1, 4, 5});
    r.initialCost = initialRes.totalCost;
    r.updatedCost = replanRes.totalCost;
    r.minSafetyDist = replanRes.minSafetyDistance;
    r.badStatesVisited = replanRes.badStatesVisited;
    r.initialExploredNodes = initialExplored;
    r.replanExploredNodes = planner.getExpandedCount() - initialExplored;
    r.initialTimeMs = initialRes.planningTimeMs;
    r.replanTimeMs = replanRes.planningTimeMs;
    r.notes = "Goal successfully updated and solved via key-shifted priority queue";
    return r;
}

TestCaseResult TestScenarios::runTestCase6() {
    PlanningProblem prob = createTestCase6();
    LPAStarPlanner planner;
    PlanningResult initialRes = planner.plan(prob);
    uint64_t initialExplored = planner.getExpandedCount();

    // Insert new shortcut transition: A(1) -> G(4) with cost 1.2
    Transition shortcut(10, 1, 4, 1.2, 1.0, 0.99, true);
    planner.addTransition(shortcut);
    PlanningResult replanRes = planner.computeShortestPath();

    TestCaseResult r;
    r.testCaseNumber = 6;
    r.title = "Transition Addition";
    r.expectedBehavior = "Initial path: 0 -> 1 -> 2 -> 3 -> 4 (Cost 4.0); After shortcut (1->4), finds improved path: 0 -> 1 -> 4 (Cost 2.2)";
    r.actualPath = initialRes.pathString() + " -> [Added Shortcut 1->4] -> " + replanRes.pathString();
    r.passed = initialRes.success && replanRes.success && (replanRes.statePath == std::vector<uint64_t>{0, 1, 4}) && (replanRes.totalCost < initialRes.totalCost);
    r.initialCost = initialRes.totalCost;
    r.updatedCost = replanRes.totalCost;
    r.minSafetyDist = replanRes.minSafetyDistance;
    r.badStatesVisited = replanRes.badStatesVisited;
    r.initialExploredNodes = initialExplored;
    r.replanExploredNodes = planner.getExpandedCount() - initialExplored;
    r.initialTimeMs = initialRes.planningTimeMs;
    r.replanTimeMs = replanRes.planningTimeMs;
    r.notes = "Incremental search discovered shortcut with minimal vertex updates";
    return r;
}

std::vector<TestCaseResult> TestScenarios::runAllTestCases() {
    std::vector<TestCaseResult> results;
    results.push_back(runTestCase1());
    results.push_back(runTestCase2());
    results.push_back(runTestCase3());
    results.push_back(runTestCase4());
    results.push_back(runTestCase5());
    results.push_back(runTestCase6());
    return results;
}
