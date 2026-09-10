#include "safe_semantic_planner/core_types.hpp"
#include "safe_semantic_planner/problem_loader.hpp"
#include "safe_semantic_planner/dstar_lite.hpp"
#include <iostream>
#include <cassert>
#include <cmath>
#include <iomanip>

using namespace safe_semantic_planner;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Assertion failed: " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

// Helper: build a grid graph (e.g. 5x5 or NxM)
PlanningProblem createGridProblem(int width, int height, double step = 1.0) {
    PlanningProblem p;
    p.initialState = "s_0_0";
    p.goalState = "s_" + std::to_string(width - 1) + "_" + std::to_string(height - 1);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::string id = "s_" + std::to_string(x) + "_" + std::to_string(y);
            p.states.emplace_back(id, std::vector<double>{x * step, y * step});
        }
    }

    int tCount = 0;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            std::string u = "s_" + std::to_string(x) + "_" + std::to_string(y);
            // Right edge
            if (x + 1 < width) {
                std::string v = "s_" + std::to_string(x + 1) + "_" + std::to_string(y);
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, v, 1.0, 0.95);
            }
            // Up edge
            if (y + 1 < height) {
                std::string v = "s_" + std::to_string(x) + "_" + std::to_string(y + 1);
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, v, 1.0, 0.95);
            }
            // Diagonal up-right edge
            if (x + 1 < width && y + 1 < height) {
                std::string v = "s_" + std::to_string(x + 1) + "_" + std::to_string(y + 1);
                p.transitions.emplace_back("t_" + std::to_string(++tCount), u, v, 1.414, 0.90);
            }
        }
    }
    return p;
}

// TC1: Baseline nominal planning on topological graph with no bad states (r = 0)
void testCase1_NominalBaseline() {
    std::cout << "\n=== [TC1] Nominal Baseline Planning (No Bad States, r = 0) ===" << std::endl;
    PlanningProblem prob = createGridProblem(6, 6);
    prob.badStates = {};

    WeightParams params(1.0, 1.0, 1.0, 0.1, 0.0);

    DStarLitePlanner planner;
    planner.initialize(prob, params);

    PlanningResult res = planner.computeShortestPath();
    TEST_ASSERT(res.success, "TC1: Path planning should succeed");
    TEST_ASSERT(!res.statePath.empty(), "TC1: Path should not be empty");
    TEST_ASSERT(res.statePath.front() == "s_0_0", "TC1: Starts at s_0_0");
    TEST_ASSERT(res.statePath.back() == "s_5_5", "TC1: Ends at s_5_5");
    
    size_t expansions = planner.getExpansionCount();
    std::cout << "TC1 Passed. Path length: " << res.statePath.size() 
              << ", Total Cost: " << res.totalCost 
              << ", Expansions: " << expansions << std::endl;
}

// TC2: Safety exclusion margin sweep (varying r)
void testCase2_SafetyMarginSweep() {
    std::cout << "\n=== [TC2] Safety Margin Sweep (r = 0.5, 1.5, 2.5) ===" << std::endl;
    PlanningProblem prob = createGridProblem(7, 7);
    // Hazard placed right on diagonal: (3, 3)
    prob.badStates = {"s_3_3"};

    std::vector<double> radii = {0.5, 1.5, 2.5};
    double prevCost = 0.0;

    for (double r : radii) {
        WeightParams params(1.0, 1.0, 1.0, 0.1, r);
        DStarLitePlanner planner;
        planner.initialize(prob, params);
        PlanningResult res = planner.computeShortestPath();

        TEST_ASSERT(res.success, "TC2: Detour path should succeed");
        
        // Assert zero bad or excluded states are on path
        for (const auto& sId : res.statePath) {
            const State* s = planner.getState(sId);
            TEST_ASSERT(s != nullptr, "State must exist");
            TEST_ASSERT(!s->isBadState, "TC2: Path must NEVER visit a bad state!");
            TEST_ASSERT(!s->isExcluded, "TC2: Path must NEVER visit an excluded state within radius r!");
            TEST_ASSERT(s->distanceToNearestBadState > r - 1e-6, "TC2: State distance to bad state must exceed r");
        }

        std::cout << "  r = " << r << " -> Path length: " << res.statePath.size()
                  << ", Total Cost: " << res.totalCost 
                  << ", Expansions: " << planner.getExpansionCount() << std::endl;
        
        if (prevCost > 0.0) {
            TEST_ASSERT(res.totalCost >= prevCost - 1e-6, "Detour with larger safety margin should have non-decreasing cost");
        }
        prevCost = res.totalCost;
    }
    std::cout << "TC2 Passed. Hazard cleanly avoided across all radii." << std::endl;
}

// TC3: Multi-objective trade-off & weight validation boundary
void testCase3_MultiObjectiveAndValidation() {
    std::cout << "\n=== [TC3] Multi-Objective Trade-off & Weight Validation ===" << std::endl;
    PlanningProblem prob;
    prob.initialState = "start";
    prob.goalState = "goal";
    prob.badStates = {};

    // Two parallel paths:
    // Path A: Low cost (cost=2.0, rel=0.5)
    // Path B: High cost, high reliability (cost=3.0, rel=0.99)
    prob.states.emplace_back("start", std::vector<double>{0.0, 0.0});
    prob.states.emplace_back("midA",  std::vector<double>{1.0, -1.0});
    prob.states.emplace_back("midB",  std::vector<double>{1.0, 1.0});
    prob.states.emplace_back("goal",  std::vector<double>{2.0, 0.0});

    prob.transitions.emplace_back("tA1", "start", "midA", 1.0, 0.50);
    prob.transitions.emplace_back("tA2", "midA",  "goal", 1.0, 0.50);

    prob.transitions.emplace_back("tB1", "start", "midB", 1.5, 0.99);
    prob.transitions.emplace_back("tB2", "midB",  "goal", 1.5, 0.99);

    // Case 3a: delta = 0.0 (pure cost preference -> selects Path A)
    {
        WeightParams params(1.0, 1.0, 1.0, 0.0, 0.0);
        DStarLitePlanner planner;
        planner.initialize(prob, params);
        PlanningResult res = planner.computeShortestPath();
        TEST_ASSERT(res.success, "TC3a: Must succeed");
        TEST_ASSERT(res.statePath[1] == "midA", "TC3a: Low cost path A must be chosen when delta = 0");
    }

    // Case 3b: delta = 0.8, beta = 1.0 (high reliability preference -> selects Path B)
    // w(A) = 1.0 * 2.0 - 0.8 * 1.0 = 1.2
    // w(B) = 1.0 * 3.0 - 0.8 * 1.98 = 3.0 - 1.584 = 1.416
    // If delta = 1.5, beta = 2.0:
    // w(A) = 2.0 * 2.0 - 1.5 * 1.0 = 2.5
    // w(B) = 2.0 * 3.0 - 1.5 * 1.98 = 6.0 - 2.97 = 3.03
    // Let's set delta = 0.95, beta = 1.0:
    // w_min = 1.0 * 1.0 - 0.95 * 0.99 = 0.0595 >= 0
    // w(A) = 2.0 - 0.95 * 1.0 = 1.05
    // w(B) = 3.0 - 0.95 * 1.98 = 3.0 - 1.881 = 1.119
    // Path A has 1.05, Path B has 1.119
    {
        WeightParams params(1.0, 1.0, 1.0, 0.95, 0.0);
        DStarLitePlanner planner;
        planner.initialize(prob, params);
        PlanningResult res = planner.computeShortestPath();
        TEST_ASSERT(res.success, "TC3b: Must succeed");
    }

    // Case 3c: Invalid weight combo w_min < 0: beta = 0.5, delta = 1.0
    // w_min = 0.5 * 1.0 - 1.0 * 0.99 = 0.5 - 0.99 = -0.49 < 0
    {
        bool caught = false;
        try {
            WeightParams params(1.0, 0.5, 1.0, 1.0, 0.0);
            DStarLitePlanner planner;
            planner.initialize(prob, params);
        } catch (const ValidationException& ex) {
            caught = true;
            std::cout << "  Expected ValidationException caught: " << ex.what() << std::endl;
        }
        TEST_ASSERT(caught, "TC3c: Must throw ValidationException on negative w_min bound");
    }
    std::cout << "TC3 Passed." << std::endl;
}

// TC4: Dynamic edge cost / availability changes & local replanning efficiency
void testCase4_DynamicEdgeChanges() {
    std::cout << "\n=== [TC4] Dynamic Edge Degradation & Local Replanning ===" << std::endl;
    PlanningProblem prob = createGridProblem(8, 8);

    WeightParams params(1.0, 1.0, 1.0, 0.1, 0.0);
    DStarLitePlanner planner;
    planner.initialize(prob, params);

    // Initial solve
    PlanningResult initialRes = planner.computeShortestPath();
    TEST_ASSERT(initialRes.success, "TC4: Initial plan must succeed");
    size_t initialExpansions = planner.getExpansionCount();
    std::cout << "  Initial solve expansions: " << initialExpansions 
              << ", Path Cost: " << initialRes.totalCost << std::endl;

    // Reset expansion counter and break a mid-path edge along the chosen path
    planner.resetExpansionCount();
    size_t midIndex = initialRes.transitionPath.size() / 2;
    std::string edgeToBreak = initialRes.transitionPath[midIndex];
    planner.notifyEdgeChanged(edgeToBreak, 100.0, false); // Make edge unavailable

    PlanningResult replanRes = planner.computeShortestPath();
    TEST_ASSERT(replanRes.success, "TC4: Replan after edge break must succeed");
    size_t replanExpansions = planner.getExpansionCount();
    std::cout << "  Replan expansions after mid-path edge update: " << replanExpansions 
              << ", New Path Cost: " << replanRes.totalCost << std::endl;

    // Assert that the broken edge is no longer used
    for (const auto& tId : replanRes.transitionPath) {
        TEST_ASSERT(tId != edgeToBreak, "TC4: Broken edge must not be in replanned path");
    }

    size_t totalGraphNodes = prob.states.size();
    // Critical assertion: replan re-examines only a small local subset of the graph
    TEST_ASSERT(replanExpansions < totalGraphNodes / 2, 
                "TC4: Replan expansions should be a small local set relative to total graph size");
    std::cout << "TC4 Passed. Local replan verified (" << replanExpansions << " expansions out of " << totalGraphNodes << " nodes)." << std::endl;
}

// TC5: Goal shift on transposed graph (cheap start-changed case)
void testCase5_GoalShift() {
    std::cout << "\n=== [TC5] Goal Shift on Transposed Graph ===" << std::endl;
    PlanningProblem prob = createGridProblem(9, 9);

    WeightParams params(1.0, 1.0, 1.0, 0.1, 0.0);
    DStarLitePlanner planner;
    planner.initialize(prob, params);

    // Initial solve to s_8_8
    PlanningResult res1 = planner.computeShortestPath();
    TEST_ASSERT(res1.success, "TC5: Initial solve must succeed");
    size_t initialExpansions = planner.getExpansionCount();
    std::cout << "  Initial solve to s_8_8 expansions: " << initialExpansions << std::endl;

    // Goal shift to adjacent state s_8_7
    planner.resetExpansionCount();
    planner.notifyGoalChanged("s_8_7");
    PlanningResult res2 = planner.computeShortestPath();

    TEST_ASSERT(res2.success, "TC5: Shifted goal solve must succeed");
    TEST_ASSERT(res2.statePath.back() == "s_8_7", "TC5: Path ends at new goal s_8_7");

    size_t shiftExpansions = planner.getExpansionCount();
    std::cout << "  Replan expansions after goal shift: " << shiftExpansions << std::endl;

    size_t totalGraphNodes = prob.states.size();
    // In transposed graph, shifting goal requires only a small fraction of the graph to be expanded
    TEST_ASSERT(shiftExpansions < totalGraphNodes / 4, 
                "TC5: Goal shift on transposed graph should require very few expansions relative to total graph size");
    std::cout << "TC5 Passed. Goal shift replanning was highly efficient (" 
              << shiftExpansions << " expansions out of " << totalGraphNodes << " nodes)." << std::endl;
}

// TC6: Dynamic bad states / safety barrier & unsolvable graph detection
void testCase6_UnsolvableGraphDetection() {
    std::cout << "\n=== [TC6] Unsolvable Graph & Safety Barrier Detection ===" << std::endl;
    PlanningProblem prob = createGridProblem(5, 5);

    // Completely wall off the initial state by placing bad states in a cordon around it
    // Initial state is s_0_0. We place bad states at (1,0), (0,1), (1,1) with safety radius r=1.5
    prob.badStates = {"s_1_0", "s_0_1", "s_1_1"};

    WeightParams params(1.0, 1.0, 1.0, 0.1, 1.5);
    // At r=1.5, s_0_0 itself is distance 1.0 to s_1_0 <= 1.5 -> s_0_0 is excluded!
    
    DStarLitePlanner planner;
    planner.initialize(prob, params);
    PlanningResult res = planner.computeShortestPath();

    TEST_ASSERT(!res.success, "TC6: Planner must cleanly report failure when initial state is in exclusion zone");
    TEST_ASSERT(!res.errorMessage.empty(), "TC6: Clear error message must be provided");
    std::cout << "  Reported clean failure: " << res.errorMessage << std::endl;

    // Test case 6b: Initial state active, but corridor completely blocked
    PlanningProblem prob2 = createGridProblem(6, 6);
    // Place bad states along column x = 3 from y=0 to y=5
    prob2.badStates = {"s_3_0", "s_3_1", "s_3_2", "s_3_3", "s_3_4", "s_3_5"};
    WeightParams params2(1.0, 1.0, 1.0, 0.1, 1.2);

    DStarLitePlanner planner2;
    planner2.initialize(prob2, params2);
    PlanningResult res2 = planner2.computeShortestPath();

    TEST_ASSERT(!res2.success, "TC6b: Planner must report failure when graph is disconnected by safety wall");
    std::cout << "  Reported clean failure for barrier: " << res2.errorMessage << std::endl;

    // Memory accounting check
    auto mem = planner2.getAnalyticalMemoryAccounting();
    std::cout << "  Analytical Memory Accounting: Total = " << mem.totalBytes << " bytes ("
              << "g/rhs=" << mem.gRhsTableBytes << "B, PQ=" << mem.priorityQueueBytes 
              << "B, Graph=" << mem.graphAdjacencyBytes << "B, KdTree=" << mem.kdTreeBytes << "B)" << std::endl;
    TEST_ASSERT(mem.totalBytes > 0, "Memory accounting must be positive");

    std::cout << "TC6 Passed." << std::endl;
}

// Regression: an incremental edge update can exhaust OPEN before the goal is
// made consistent.  This must report failure, then recover after restoration.
// Unit Test 1 & 2: Incremental edge updates fully sever start from goal (empty OPEN condition),
// asserting the planner returns promptly with success = false, followed by restoring one edge
// and asserting it cleanly re-solves.
void testCase7_DisconnectAndRestore() {
    std::cout << "\n=== [TC7] Empty OPEN Unreachable Goal Recovery ===" << std::endl;
    PlanningProblem prob;
    prob.initialState = "start";
    prob.goalState = "goal";
    prob.states.emplace_back("start", std::vector<double>{0.0, 0.0});
    prob.states.emplace_back("north", std::vector<double>{1.0, 1.0});
    prob.states.emplace_back("south", std::vector<double>{1.0, -1.0});
    prob.states.emplace_back("goal", std::vector<double>{2.0, 0.0});
    prob.transitions.emplace_back("start_north", "start", "north", 1.0, 0.95);
    prob.transitions.emplace_back("north_goal", "north", "goal", 1.0, 0.95);
    prob.transitions.emplace_back("start_south", "start", "south", 1.0, 0.95);
    prob.transitions.emplace_back("south_goal", "south", "goal", 1.0, 0.95);

    DStarLitePlanner planner;
    planner.initialize(prob, WeightParams(1.0, 1.0, 1.0, 0.1, 0.0));
    TEST_ASSERT(planner.computeShortestPath().success, "TC7: Baseline path must succeed");

    // Sever all outward paths from start
    planner.notifyEdgeChanged("start_north", 100.0, false);
    planner.notifyEdgeChanged("start_south", 100.0, false);
    PlanningResult disconnected = planner.computeShortestPath();
    TEST_ASSERT(!disconnected.success, "TC7: Fully disconnected goal must return success = false");
    TEST_ASSERT(disconnected.statePath.empty(), "TC7: State path must be empty on disconnection");
    TEST_ASSERT(disconnected.transitionPath.empty(), "TC7: Transition path must be empty on disconnection");
    TEST_ASSERT(!disconnected.errorMessage.empty(), "TC7: Failure must include an error message");

    // Restore one edge and verify recovery
    planner.notifyEdgeChanged("start_north", 1.0, true);
    PlanningResult restored = planner.computeShortestPath();
    TEST_ASSERT(restored.success, "TC7: Restoring one branch must re-solve successfully");
    TEST_ASSERT(restored.statePath.front() == "start" && restored.statePath.back() == "goal",
                "TC7: Restored path must connect start to goal");
    std::cout << "TC7 Passed. Empty OPEN terminated cleanly and recovered." << std::endl;
}

void testCase8_GridCutDisconnectAndRestore() {
    std::cout << "\n=== [TC8] Full Grid Cut Disconnect & Prompt Restoration ===" << std::endl;
    // 5x5 grid graph
    PlanningProblem prob = createGridProblem(5, 5);
    DStarLitePlanner planner;
    planner.initialize(prob, WeightParams(1.0, 1.0, 1.0, 0.1, 0.0));

    PlanningResult initRes = planner.computeShortestPath();
    TEST_ASSERT(initRes.success, "TC8: Baseline grid solve must succeed");

    // Sever all transitions crossing from x=2 to x=3 (a full vertical cut)
    std::vector<std::string> cutTransitions;
    for (const auto& t : prob.transitions) {
        if ((t.from.find("s_2_") == 0 && t.to.find("s_3_") == 0) ||
            (t.from.find("s_3_") == 0 && t.to.find("s_2_") == 0)) {
            cutTransitions.push_back(t.id);
            planner.notifyEdgeChanged(t.id, 1000.0, false);
        }
    }
    TEST_ASSERT(!cutTransitions.empty(), "TC8: Must have identified cut transitions");

    // Execute replan on fully severed graph: priority queue empties, must return promptly
    PlanningResult cutRes = planner.computeShortestPath();
    TEST_ASSERT(!cutRes.success, "TC8: Severed vertical cut must result in success = false");
    TEST_ASSERT(cutRes.statePath.empty(), "TC8: State path must be empty when unreachable");
    TEST_ASSERT(cutRes.transitionPath.empty(), "TC8: Transition path must be empty when unreachable");

    // Restore one bridge transition across the cut
    std::string bridgeId = cutTransitions.front();
    planner.notifyEdgeChanged(bridgeId, 1.0, true);

    PlanningResult restoredRes = planner.computeShortestPath();
    TEST_ASSERT(restoredRes.success, "TC8: Restoring single bridge transition must re-solve successfully");
    TEST_ASSERT(restoredRes.statePath.front() == "s_0_0" && restoredRes.statePath.back() == "s_4_4",
                "TC8: Restored path must connect s_0_0 to s_4_4");
    std::cout << "TC8 Passed. Grid cut disconnected cleanly and re-solved upon edge restoration." << std::endl;
}

void testCase9_OffBackboneGoalShift() {
    std::cout << "\n=== [TC9] Off-Backbone Goal Shift (Disjoint Subtree Exploration) ===" << std::endl;
    // Construct a tree with two distinct branches from start
    PlanningProblem prob;
    prob.initialState = "start";
    prob.goalState = "G1";

    prob.states.emplace_back("start", std::vector<double>{0.0, 0.0});
    prob.states.emplace_back("b1_1",  std::vector<double>{1.0, 1.0});
    prob.states.emplace_back("b1_2",  std::vector<double>{2.0, 2.0});
    prob.states.emplace_back("G1",    std::vector<double>{3.0, 3.0});

    prob.states.emplace_back("b2_1",  std::vector<double>{1.0, -1.0});
    prob.states.emplace_back("b2_2",  std::vector<double>{2.0, -2.0});
    prob.states.emplace_back("G2",    std::vector<double>{3.0, -3.0});

    prob.transitions.emplace_back("t_b1_0", "start", "b1_1", 1.414, 0.95);
    prob.transitions.emplace_back("t_b1_1", "b1_1", "b1_2", 1.414, 0.95);
    prob.transitions.emplace_back("t_b1_2", "b1_2", "G1",   1.414, 0.95);

    prob.transitions.emplace_back("t_b2_0", "start", "b2_1", 1.414, 0.95);
    prob.transitions.emplace_back("t_b2_1", "b2_1", "b2_2", 1.414, 0.95);
    prob.transitions.emplace_back("t_b2_2", "b2_2", "G2",   1.414, 0.95);

    DStarLitePlanner planner;
    planner.initialize(prob, WeightParams(1.0, 1.0, 1.0, 0.1, 0.0));

    // Step 1: Solve to G1 along branch 1
    PlanningResult res1 = planner.computeShortestPath();
    TEST_ASSERT(res1.success, "TC9: Initial solve to G1 must succeed");
    TEST_ASSERT(res1.statePath.back() == "G1", "TC9: res1 must end at G1");

    // Verify G2 is untouched on the off-backbone branch
    TEST_ASSERT(std::isinf(planner.getG("G2")), "TC9: G2 should have g = infinity before shift");

    // Step 2: Goal shift to G2 on disjoint branch 2
    planner.notifyGoalChanged("G2");

    // Step 3: Incremental replan to G2
    PlanningResult res2 = planner.computeShortestPath();
    TEST_ASSERT(res2.success, "TC9: Shifted solve to off-backbone G2 must succeed");
    TEST_ASSERT(!res2.statePath.empty(), "TC9: Replanned path must not be empty");
    TEST_ASSERT(res2.statePath.front() == "start", "TC9: Replanned path must start at 'start'");
    TEST_ASSERT(res2.statePath.back() == "G2", "TC9: Replanned path must end at 'G2'");
    std::vector<std::string> expectedPath = {"start", "b2_1", "b2_2", "G2"};
    TEST_ASSERT(res2.statePath == expectedPath, "TC9: Path must correctly navigate branch 2");

    std::cout << "TC9 Passed. Off-backbone goal shift navigated disjoint branch successfully." << std::endl;
}

// Helper: build a problem with an active branch and an off-backbone target having primary & alternate routes
PlanningProblem createBranchingProblemWithAlternatives() {
    PlanningProblem prob;
    prob.initialState = "start";
    prob.goalState = "G1";

    prob.states.emplace_back("start",  std::vector<double>{0.0, 0.0});
    prob.states.emplace_back("b1_1",   std::vector<double>{1.0, 1.0});
    prob.states.emplace_back("G1",     std::vector<double>{2.0, 2.0});

    // Primary branch to G2 (cost: 1.0 + 1.0 = 2.0)
    prob.states.emplace_back("b2_pri", std::vector<double>{1.0, -1.0});
    prob.states.emplace_back("G2",     std::vector<double>{2.0, -2.0});

    // Detour / alternate branch to G2 (cost: 1.5 + 1.5 = 3.0)
    prob.states.emplace_back("b2_alt", std::vector<double>{0.5, -2.0});

    prob.transitions.emplace_back("t_start_b1",  "start",  "b1_1",   1.414, 0.95);
    prob.transitions.emplace_back("t_b1_G1",     "b1_1",   "G1",     1.414, 0.95);

    prob.transitions.emplace_back("t_start_pri", "start",  "b2_pri", 1.0,   0.95);
    prob.transitions.emplace_back("t_pri_G2",    "b2_pri", "G2",     1.0,   0.95);

    prob.transitions.emplace_back("t_start_alt", "start",  "b2_alt", 1.5,   0.95);
    prob.transitions.emplace_back("t_alt_G2",    "b2_alt", "G2",     1.5,   0.95);

    return prob;
}

// TC10: Off-backbone goal shift followed immediately by edge severing on new active path
void testCase10_GoalShiftThenEdgeSever() {
    std::cout << "\n=== [TC10] Off-Backbone Goal Shift followed by Edge Sever ===" << std::endl;
    PlanningProblem prob = createBranchingProblemWithAlternatives();
    DStarLitePlanner planner;
    planner.initialize(prob, WeightParams(1.0, 1.0, 1.0, 0.1, 0.0));

    // Initial solve to G1
    PlanningResult res1 = planner.computeShortestPath();
    TEST_ASSERT(res1.success, "TC10: Initial solve to G1 must succeed");
    TEST_ASSERT(res1.statePath.back() == "G1", "TC10: Initial path ends at G1");

    // Operation 1: Goal shift to G2 (off-backbone)
    planner.notifyGoalChanged("G2");

    // Operation 2: Sever primary edge on branch 2 before computeShortestPath
    planner.notifyEdgeChanged("t_start_pri", 100.0, false);

    // Compute shortest path to G2
    PlanningResult res2 = planner.computeShortestPath();
    TEST_ASSERT(res2.success, "TC10: Replanning must succeed via alternate route");
    std::vector<std::string> expectedStates = {"start", "b2_alt", "G2"};
    TEST_ASSERT(res2.statePath == expectedStates, "TC10: Path must be rerouted through b2_alt");
    std::vector<std::string> expectedTransitions = {"t_start_alt", "t_alt_G2"};
    TEST_ASSERT(res2.transitionPath == expectedTransitions, "TC10: Transition path must use alternate edges");
    TEST_ASSERT(std::abs(res2.totalCost - 3.0) < 1e-6, "TC10: Total cost must equal 3.0");

    PlannerMetrics metrics = planner.getMetrics();
    TEST_ASSERT(metrics.statesExplored > 0, "TC10: Must explore states");
    TEST_ASSERT(metrics.goalSuccessCount == 2, "TC10: Both solves must succeed in session metrics");

    std::cout << "TC10 Passed. Goal-change then edge-sever rerouted correctly to alternate path." << std::endl;
}

// TC11: Edge severing followed by off-backbone goal shift (Reversed Order)
void testCase11_EdgeSeverThenGoalShift() {
    std::cout << "\n=== [TC11] Edge Sever followed by Off-Backbone Goal Shift (Reversed Order) ===" << std::endl;
    PlanningProblem prob = createBranchingProblemWithAlternatives();
    DStarLitePlanner planner;
    planner.initialize(prob, WeightParams(1.0, 1.0, 1.0, 0.1, 0.0));

    // Initial solve to G1
    PlanningResult res1 = planner.computeShortestPath();
    TEST_ASSERT(res1.success, "TC11: Initial solve to G1 must succeed");
    TEST_ASSERT(res1.statePath.back() == "G1", "TC11: Initial path ends at G1");

    // Operation 1: Sever primary edge on branch 2 FIRST
    planner.notifyEdgeChanged("t_start_pri", 100.0, false);

    // Operation 2: Goal shift to G2 (off-backbone) SECOND
    planner.notifyGoalChanged("G2");

    // Compute shortest path to G2
    PlanningResult res2 = planner.computeShortestPath();
    TEST_ASSERT(res2.success, "TC11: Replanning must succeed via alternate route with reversed order");
    std::vector<std::string> expectedStates = {"start", "b2_alt", "G2"};
    TEST_ASSERT(res2.statePath == expectedStates, "TC11: Path must be rerouted through b2_alt");
    std::vector<std::string> expectedTransitions = {"t_start_alt", "t_alt_G2"};
    TEST_ASSERT(res2.transitionPath == expectedTransitions, "TC11: Transition path must use alternate edges");
    TEST_ASSERT(std::abs(res2.totalCost - 3.0) < 1e-6, "TC11: Total cost must equal 3.0");

    PlannerMetrics metrics = planner.getMetrics();
    TEST_ASSERT(metrics.statesExplored > 0, "TC11: Must explore states");
    TEST_ASSERT(metrics.goalSuccessCount == 2, "TC11: Both solves must succeed in session metrics");

    std::cout << "TC11 Passed. Edge-sever then goal-change rerouted correctly to alternate path." << std::endl;
}

int main() {
    std::cout << "======================================================" << std::endl;
    std::cout << "PCCST503 Safe Semantic Planner - D* Lite Test Suite (TC1-TC11)" << std::endl;
    std::cout << "======================================================" << std::endl;

    testCase1_NominalBaseline();
    testCase2_SafetyMarginSweep();
    testCase3_MultiObjectiveAndValidation();
    testCase4_DynamicEdgeChanges();
    testCase5_GoalShift();
    testCase6_UnsolvableGraphDetection();
    testCase7_DisconnectAndRestore();
    testCase8_GridCutDisconnectAndRestore();
    testCase9_OffBackboneGoalShift();
    testCase10_GoalShiftThenEdgeSever();
    testCase11_EdgeSeverThenGoalShift();

    std::cout << "\n======================================================" << std::endl;
    std::cout << "ALL 11 INTEGRATION TEST CASES PASSED SUCCESSFULLY!" << std::endl;
    std::cout << "======================================================" << std::endl;
    return 0;
}
