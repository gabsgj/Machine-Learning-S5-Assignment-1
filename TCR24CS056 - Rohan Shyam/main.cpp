#include <iostream>
#include <iomanip>
#include <string>
#include <unordered_map>
#include "planner.hpp"

// Helper to format state path with user-friendly names
std::string formatPath(const std::vector<uint64_t>& path, const std::unordered_map<uint64_t, std::string>& names) {
    if (path.empty()) return "(None)";
    std::string out;
    for (size_t i = 0; i < path.size(); ++i) {
        auto it = names.find(path[i]);
        if (it != names.end()) {
            out += it->second;
        } else {
            out += std::to_string(path[i]);
        }
        if (i + 1 < path.size()) out += " -> ";
    }
    return out;
}

void printResult(const std::string& prefix, const PlanningResult& res, const std::unordered_map<uint64_t, std::string>& names) {
    std::cout << prefix << "Path: " << formatPath(res.statePath, names) << "\n";
    std::cout << prefix << "Cost: " << std::fixed << std::setprecision(2) << res.totalCost << "\n";
    std::cout << prefix << "Safety Score: ";
    if (std::isinf(res.safetyScore)) {
        std::cout << "INF (No bad states)";
    } else {
        std::cout << std::fixed << std::setprecision(2) << res.safetyScore;
    }
    std::cout << "\n";
    std::cout << prefix << "Status: " << (res.success ? "SUCCESS" : "FAILURE") << "\n";
    std::cout << prefix << "Execution Time: " << std::fixed << std::setprecision(1) << res.planningTimeUs << " us\n";
    std::cout << prefix << "States Explored: " << res.nodesExplored << "\n";
    std::cout << prefix << "Bad States Visited: " << res.badStatesVisited << "\n";
}

void runScenario1() {
    std::cout << "\n========================================\n";
    std::cout << "Test 1: Basic Reachability\n";
    std::cout << "========================================\n";
    std::cout << "Graph: S -> A -> B -> G\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {3.0, 0.0}}
    };

    problem.transitions = {
        {101, 1, 2, 2.0, 1.0, 1.0, true}, // S -> A
        {102, 2, 3, 2.0, 1.0, 1.0, true}, // A -> B
        {103, 3, 4, 2.0, 1.0, 1.0, true}  // B -> G
    };

    Planner planner;
    PlanningResult res = planner.plan(problem);
    printResult("", res, names);
}

void runScenario2() {
    std::cout << "\n========================================\n";
    std::cout << "Test 2: Bad State Avoidance\n";
    std::cout << "========================================\n";
    std::cout << "Primary Route: S -> A -> X -> G (X is BAD)\n";
    std::cout << "Alternative Route: S -> C -> D -> G\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "X (BAD)"}, {4, "G"}, {5, "C"}, {6, "D"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {3}; // X is bad

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}}, // Bad state X
        {4, {3.0, 0.0}},
        {5, {0.0, 1.0}},
        {6, {3.0, 1.0}}
    };

    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true}, // S -> A
        {102, 2, 3, 1.0, 0.0, 1.0, true}, // A -> X
        {103, 3, 4, 1.0, 0.0, 1.0, true}, // X -> G
        {104, 1, 5, 2.0, 1.0, 1.0, true}, // S -> C
        {105, 5, 6, 3.0, 1.0, 1.0, true}, // C -> D
        {106, 6, 4, 2.0, 1.0, 1.0, true}  // D -> G
    };

    Planner planner;
    PlanningResult res = planner.plan(problem);
    printResult("", res, names);
}

void runScenario3() {
    std::cout << "\n========================================\n";
    std::cout << "Test 3: Safety Margin (Cost vs Safety Trade-off)\n";
    std::cout << "========================================\n";
    std::cout << "Path 1 (Cheaper, near bad state): S -> N_cheap -> G\n";
    std::cout << "Path 2 (Safer, higher cost):      S -> N_safe  -> G\n";
    std::cout << "Bad state B_bad is at (2.0, 0.0)\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "N_cheap"}, {3, "N_safe"}, {4, "G"}, {5, "B_bad"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {5};

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {2.0, 0.2}}, // Close to B_bad (dist = 0.2)
        {3, {2.0, 3.0}}, // Far from B_bad (dist = 3.0)
        {4, {4.0, 0.0}},
        {5, {2.0, 0.0}}  // Bad state
    };

    problem.transitions = {
        {101, 1, 2, 2.5, 0.2, 1.0, true}, // S -> N_cheap (cost 2.5)
        {102, 2, 4, 2.5, 0.2, 1.0, true}, // N_cheap -> G (cost 2.5)
        {103, 1, 3, 4.0, 3.0, 1.0, true}, // S -> N_safe  (cost 4.0)
        {104, 3, 4, 4.0, 3.0, 1.0, true}  // N_safe -> G  (cost 4.0)
    };

    std::cout << "--- [3A] Pure Transition Cost Minimization (Safety Weight = 0.0) ---\n";
    Planner plannerCostOnly;
    plannerCostOnly.setSafetyWeight(0.0);
    PlanningResult res1 = plannerCostOnly.plan(problem);
    printResult("  ", res1, names);

    std::cout << "\n--- [3B] Safety-Aware Path Selection (Safety Weight = 1.0) ---\n";
    Planner plannerSafe;
    plannerSafe.setSafetyWeight(1.0);
    PlanningResult res2 = plannerSafe.plan(problem);
    printResult("  ", res2, names);
}

void runScenario4() {
    std::cout << "\n========================================\n";
    std::cout << "Test 4: Dynamic Transition (Incremental Replanning)\n";
    std::cout << "========================================\n";
    std::cout << "Initial options: S -> A -> G (cost 2.0) OR S -> B -> G (cost 4.0)\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {1.0, 1.0}},
        {4, {2.0, 0.0}}
    };

    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true}, // S -> A (cost 1.0)
        {102, 2, 4, 1.0, 1.0, 1.0, true}, // A -> G (cost 1.0)
        {103, 1, 3, 2.0, 1.0, 1.0, true}, // S -> B (cost 2.0)
        {104, 3, 4, 2.0, 1.0, 1.0, true}  // B -> G (cost 2.0)
    };

    Planner planner;
    std::cout << "--- Initial Plan ---\n";
    PlanningResult resInitial = planner.plan(problem);
    printResult("  ", resInitial, names);

    std::cout << "\nEvent: Transition A -> G (id: 102) becomes unavailable.\n";
    planner.updateTransition(102, false);

    std::cout << "--- Replanned Path ---\n";
    PlanningResult resReplan = planner.replan();
    printResult("  ", resReplan, names);
}

void runScenario5() {
    std::cout << "\n========================================\n";
    std::cout << "Test 5: Goal Update (Incremental Replanning)\n";
    std::cout << "========================================\n";
    std::cout << "Route to G1: S -> A -> G1 (cost 2.0)\n";
    std::cout << "Route to G2: S -> B -> G2 (cost 3.0)\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "G1"}, {4, "B"}, {5, "G2"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 3; // Initially G1
    problem.badStates = {};

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}}, // G1
        {4, {0.0, 1.0}},
        {5, {0.0, 2.0}}  // G2
    };

    problem.transitions = {
        {101, 1, 2, 1.0, 1.0, 1.0, true}, // S -> A
        {102, 2, 3, 1.0, 1.0, 1.0, true}, // A -> G1
        {103, 1, 4, 1.5, 1.0, 1.0, true}, // S -> B
        {104, 4, 5, 1.5, 1.0, 1.0, true}  // B -> G2
    };

    Planner planner;
    std::cout << "--- Initial Plan to G1 ---\n";
    PlanningResult res1 = planner.plan(problem);
    printResult("  ", res1, names);

    std::cout << "\nEvent: Goal updated to G2 (id: 5).\n";
    planner.updateGoal(5);

    std::cout << "--- Replanned Path to G2 ---\n";
    PlanningResult res2 = planner.replan();
    printResult("  ", res2, names);
}

void runScenario6() {
    std::cout << "\n========================================\n";
    std::cout << "Test 6: Transition Addition (Incremental Replanning)\n";
    std::cout << "========================================\n";
    std::cout << "Initial Route: S -> A -> B -> G (cost 6.0)\n\n";

    std::unordered_map<uint64_t, std::string> names = {
        {1, "S"}, {2, "A"}, {3, "B"}, {4, "G"}
    };

    PlanningProblem problem;
    problem.initialState = 1;
    problem.goalState = 4;
    problem.badStates = {};

    problem.states = {
        {1, {0.0, 0.0}},
        {2, {1.0, 0.0}},
        {3, {2.0, 0.0}},
        {4, {2.5, 0.0}}
    };

    problem.transitions = {
        {101, 1, 2, 2.0, 1.0, 1.0, true}, // S -> A
        {102, 2, 3, 2.0, 1.0, 1.0, true}, // A -> B
        {103, 3, 4, 2.0, 1.0, 1.0, true}  // B -> G
    };

    Planner planner;
    std::cout << "--- Initial Plan ---\n";
    PlanningResult res1 = planner.plan(problem);
    printResult("  ", res1, names);

    std::cout << "\nEvent: Discovered shortcut transition S -> G (cost 2.5).\n";
    Transition shortcut = {104, 1, 4, 2.5, 1.0, 1.0, true};
    planner.addTransition(shortcut);

    std::cout << "--- Replanned Path with Shortcut ---\n";
    PlanningResult res2 = planner.replan();
    printResult("  ", res2, names);
}

int main() {
    std::cout << "====================================================\n";
    std::cout << "        SAFE SEMANTIC PLANNER (D* LITE)            \n";
    std::cout << "====================================================\n";

    runScenario1();
    runScenario2();
    runScenario3();
    runScenario4();
    runScenario5();
    runScenario6();

    std::cout << "\n====================================================\n";
    std::cout << "        ALL 6 DEMONSTRATION SCENARIOS COMPLETED     \n";
    std::cout << "====================================================\n";
    return 0;
}
