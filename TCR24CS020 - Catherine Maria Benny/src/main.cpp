#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "DStarLite.h"
#include "GraphGenerator.h"
#include "Metrics.h"
#include "TestScenarios.h"
#include "VisualisationData.h"

static void printHeader(const std::string& title) {
    std::cout << "========================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "========================================\n";
}

static std::string pathToString(const std::vector<uint64_t>& path) {
    std::ostringstream oss;
    for (size_t i = 0; i < path.size(); ++i) {
        oss << path[i];
        if (i + 1 < path.size()) oss << " -> ";
    }
    return oss.str();
}

static void printResult(const PlanningResult& r) {
    std::cout << "Success                 : " << (r.success ? "YES" : "NO") << "\n";
    if (!r.success) {
        std::cout << "Error                   : " << r.errorMessage << "\n";
        return;
    }
    std::cout << "Path (states)           : " << pathToString(r.statePath) << "\n";
    std::cout << "Total Cost              : " << std::fixed << std::setprecision(3) << r.totalCost << "\n";
    std::cout << "Minimum Safety Distance : " << r.safetyScore << "\n";
    std::cout << "Reliability             : " << r.reliability << "\n";
    std::cout << "Objective Score         : " << r.objectiveScore << "\n";
    std::cout << "Explored States         : " << r.exploredStates << "\n";
    std::cout << "Planning Time           : " << r.planningTimeMs << " ms\n";
    std::cout << "Bad States Visited      : " << r.badStatesVisited << "\n";
}

// ---------------------------------------------------------------------------
// Test Case 1
// ---------------------------------------------------------------------------
static void runTest1(const std::string& exportPath) {
    printHeader("TEST CASE 1: Basic Reachability");
    std::cout << "Graph: S(0) -> A(1) -> B(2) -> G(3)\n\n";
    DStarLite planner;
    auto problem = scenarios::testCase1_BasicReachability();
    auto result = planner.plan(problem);
    printResult(result);
    if (!exportPath.empty()) {
        visualisation::exportToJson(exportPath, problem, result, "Test Case 1");
        std::cout << "\nExported visualization data to " << exportPath << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test Case 2
// ---------------------------------------------------------------------------
static void runTest2(const std::string& exportPath) {
    printHeader("TEST CASE 2: Bad State Avoidance");
    std::cout << "S(0)->A(1)->X(2,BAD)->G(4)  vs  S(0)->C(3)->D(5)->G(4)\n\n";
    DStarLite planner;
    auto problem = scenarios::testCase2_BadStateAvoidance();
    auto result = planner.plan(problem);
    printResult(result);
    bool avoidedBad = result.success &&
        std::find(result.statePath.begin(), result.statePath.end(), 2) == result.statePath.end();
    std::cout << "Bad state X avoided      : " << (avoidedBad ? "YES" : "NO") << "\n";
    if (!exportPath.empty()) {
        visualisation::exportToJson(exportPath, problem, result, "Test Case 2");
        std::cout << "\nExported visualization data to " << exportPath << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test Case 3
// ---------------------------------------------------------------------------
static void runTest3(const std::string& exportPath) {
    printHeader("TEST CASE 3: Safety Margin (Cost vs Safety Tradeoff)");
    std::cout << "Path 1: S->Pnear->G  (cost 2, close to bad state)\n";
    std::cout << "Path 2: S->Pfar->G   (cost 4, far from bad state)\n\n";
    auto problem = scenarios::testCase3_SafetyMargin();

    std::vector<double> gammas = {0.0, 1.0, 5.0, 20.0};
    for (double gamma : gammas) {
        DStarLite planner;
        planner.weights.gamma = gamma;
        auto result = planner.plan(problem);
        std::cout << "--- Safety weight (gamma) = " << gamma << " ---\n";
        printResult(result);
        std::cout << "\n";
        if (!exportPath.empty() && gamma == gammas.back()) {
            visualisation::exportToJson(exportPath, problem, result, "Test Case 3 (gamma=" + std::to_string(gamma) + ")");
        }
    }
    std::cout << "Interpretation: D* Lite here minimizes a COMPOSITE edge weight\n"
                 "beta*cost + gamma*safetyPenalty + delta*reliabilityPenalty (see\n"
                 "report.md section 11), so gamma does not merely change how the final\n"
                 "path is *scored* -- it changes which path the search actually selects.\n"
                 "At gamma=0 the search is pure cost minimization and picks the cheap,\n"
                 "unsafe Path 1. As gamma grows, the safety penalty on the near-miss\n"
                 "state dominates and the search switches to the more expensive but\n"
                 "safer Path 2. Bad-state avoidance itself remains a HARD constraint at\n"
                 "every gamma (edges into a bad state are always +inf, never merely\n"
                 "penalized) -- gamma only trades off cost against *proximity* to bad\n"
                 "states among paths that already avoid them entirely.\n";
    if (!exportPath.empty()) {
        std::cout << "\nExported visualization data to " << exportPath << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test Case 4
// ---------------------------------------------------------------------------
static void runTest4(const std::string& exportPath) {
    printHeader("TEST CASE 4: Dynamic Transition (Edge Removal)");
    auto problem = scenarios::testCase4_DynamicTransition();
    DStarLite planner;
    auto initial = planner.plan(problem);
    std::cout << "Initial plan:\n";
    printResult(initial);

    std::cout << "\nChange: transition A(1) -> G(2) [id=1] becomes unavailable\n\n";
    auto replanned = planner.setTransitionAvailability(1, false);

    printHeader("DYNAMIC REPLANNING");
    std::cout << "Previous Path : " << pathToString(initial.statePath) << "\n";
    std::cout << "New Path      : " << pathToString(replanned.statePath) << "\n\n";
    std::cout << "Initial Planning Time : " << initial.planningTimeMs << " ms\n";
    std::cout << "Replanning Time       : " << replanned.planningTimeMs << " ms\n";
    std::cout << "States Explored (replan): " << replanned.exploredStates
              << " (initial: " << initial.exploredStates << ")\n";
    std::cout << "Bad States Visited     : " << replanned.badStatesVisited << "\n";

    if (!exportPath.empty()) {
        PlanningProblem afterProblem = problem;
        for (auto& t : afterProblem.transitions) if (t.id == 1) t.available = false;
        std::string beforePath = exportPath.substr(0, exportPath.find_last_of('.')) + "_before.json";
        std::string afterPath = exportPath.substr(0, exportPath.find_last_of('.')) + "_after.json";
        visualisation::exportToJson(beforePath, problem, initial, "Test Case 4 (before: A->G available)");
        visualisation::exportToJson(afterPath, afterProblem, replanned, "Test Case 4 (after: A->G unavailable)");
        std::cout << "\nExported visualization data to " << beforePath << " and " << afterPath << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test Case 5
// ---------------------------------------------------------------------------
static void runTest5(const std::string& exportPath) {
    printHeader("TEST CASE 5: Goal Update");
    auto problem = scenarios::testCase5_GoalUpdate();
    DStarLite planner;
    auto initial = planner.plan(problem);
    std::cout << "Initial plan (goal = G1 = state 2):\n";
    printResult(initial);

    std::cout << "\nChange: goal updated from G1(2) to G2(5)\n\n";
    auto replanned = planner.updateGoal(5);

    printHeader("DYNAMIC REPLANNING (GOAL CHANGE)");
    std::cout << "Previous Path : " << pathToString(initial.statePath) << "\n";
    std::cout << "New Path      : " << pathToString(replanned.statePath) << "\n\n";
    printResult(replanned);

    if (!exportPath.empty()) {
        PlanningProblem afterProblem = problem;
        afterProblem.goalState = 5;
        std::string beforePath = exportPath.substr(0, exportPath.find_last_of('.')) + "_before.json";
        std::string afterPath = exportPath.substr(0, exportPath.find_last_of('.')) + "_after.json";
        visualisation::exportToJson(beforePath, problem, initial, "Test Case 5 (before: goal = G1)");
        visualisation::exportToJson(afterPath, afterProblem, replanned, "Test Case 5 (after: goal = G2)");
        std::cout << "\nExported visualization data to " << beforePath << " and " << afterPath << "\n";
    }
}

// ---------------------------------------------------------------------------
// Test Case 6
// ---------------------------------------------------------------------------
static void runTest6(const std::string& exportPath) {
    printHeader("TEST CASE 6: Transition Addition (Shortcut)");
    auto problem = scenarios::testCase6_TransitionAddition();
    DStarLite planner;
    auto initial = planner.plan(problem);
    std::cout << "Initial plan (no shortcut):\n";
    printResult(initial);

    std::cout << "\nChange: new shortcut transition S(0) -> G(3) [id=10, cost=3.0,\n"
                 "the direct Euclidean distance] added -- cheaper than the 4.0-cost detour\n\n";
    Transition shortcut(10, 0, 3, 3.0, 0.9, 0.9, true);
    auto replanned = planner.addTransition(shortcut);

    printHeader("DYNAMIC REPLANNING (NEW TRANSITION)");
    std::cout << "Previous Path : " << pathToString(initial.statePath) << "\n";
    std::cout << "New Path      : " << pathToString(replanned.statePath) << "\n\n";
    printResult(replanned);

    if (!exportPath.empty()) {
        PlanningProblem afterProblem = problem;
        afterProblem.transitions.push_back(shortcut);
        std::string beforePath = exportPath.substr(0, exportPath.find_last_of('.')) + "_before.json";
        std::string afterPath = exportPath.substr(0, exportPath.find_last_of('.')) + "_after.json";
        visualisation::exportToJson(beforePath, problem, initial, "Test Case 6 (before: no shortcut)");
        visualisation::exportToJson(afterPath, afterProblem, replanned, "Test Case 6 (after: shortcut added)");
        std::cout << "\nExported visualization data to " << beforePath << " and " << afterPath << "\n";
    }
}

static void runTestByNumber(int n, const std::string& exportPath) {
    switch (n) {
        case 1: runTest1(exportPath); break;
        case 2: runTest2(exportPath); break;
        case 3: runTest3(exportPath); break;
        case 4: runTest4(exportPath); break;
        case 5: runTest5(exportPath); break;
        case 6: runTest6(exportPath); break;
        default:
            std::cerr << "Unknown test case: " << n << " (valid: 1-6)\n";
    }
}

static void runDemo() {
    std::cout << "############################################\n";
    std::cout << "#   SAFEPATH -- DEMO MODE                    #\n";
    std::cout << "############################################\n\n";
    for (int i = 1; i <= 6; ++i) {
        runTestByNumber(i, "");
        std::cout << "\n";
    }
    std::cout << "============ SUMMARY ============\n";
    std::cout << "All six assignment test cases executed above.\n";
    std::cout << "Re-run any individual case with --test N --export results/graphs/testN.json\n";
    std::cout << "then visualize it with:\n";
    std::cout << "  python3 visualization/visualize_graph.py results/graphs/testN.json\n";
}

// ---------------------------------------------------------------------------
// Experiments
// ---------------------------------------------------------------------------
static void runExperiments(unsigned int seed) {
    printHeader("EXPERIMENTAL EVALUATION");
    std::vector<int> sizes = {100, 500, 1000, 5000};
    std::ofstream csv("results/experiments.csv");
    csv << "numStates,success,totalCost,minSafetyDistance,reliability,objectiveScore,"
           "badStatesVisited,exploredStates,planningTimeMs,memoryBytesEstimate,"
           "replanningTimeMs,replanExploredStates\n";

    for (int n : sizes) {
        GraphGenConfig cfg;
        cfg.numStates = n;
        cfg.seed = seed;
        cfg.numBadStates = std::max(1, n / 20);
        cfg.edgeProbability = n <= 500 ? 0.02 : (n <= 1000 ? 0.01 : 0.003);
        auto problem = graphgen::generate(cfg);

        DStarLite planner;
        auto result = planner.plan(problem);

        // Dynamic replanning experiment: disable the first backbone transition
        // used by the found path (if any) and measure replanning time.
        double replanMs = 0.0;
        int replanExplored = 0;
        if (result.success && !result.transitionPath.empty()) {
            uint64_t tid = result.transitionPath.front();
            auto replanned = planner.setTransitionAvailability(tid, false);
            replanMs = replanned.planningTimeMs;
            replanExplored = replanned.exploredStates;
        }

        std::cout << "n=" << n << "  success=" << result.success
                  << "  cost=" << result.totalCost
                  << "  planningTime=" << result.planningTimeMs << "ms"
                  << "  replanTime=" << replanMs << "ms"
                  << "  explored=" << result.exploredStates << "\n";

        csv << n << "," << result.success << "," << result.totalCost << ","
            << (std::isfinite(result.safetyScore) ? result.safetyScore : -1.0) << ","
            << result.reliability << "," << result.objectiveScore << ","
            << result.badStatesVisited << "," << result.exploredStates << ","
            << result.planningTimeMs << "," << result.memoryBytesEstimate << ","
            << replanMs << "," << replanExplored << "\n";
    }
    csv.close();
    std::cout << "\nResults written to results/experiments.csv\n";
    std::cout << "Generate plots with: python3 visualization/plot_experiments.py results/experiments.csv\n";

    // Cost-vs-safety-weight sweep, reusing Test Case 3's two-path scenario at
    // a slightly larger reproducible random scale for a clearer signal too.
    printHeader("SAFETY WEIGHT SWEEP (Test Case 3 scenario)");
    std::ofstream sweepCsv("results/safety_weight_sweep.csv");
    sweepCsv << "gamma,totalCost,minSafetyDistance,objectiveScore\n";
    auto p3 = scenarios::testCase3_SafetyMargin();
    for (double gamma = 0.0; gamma <= 20.0; gamma += 2.0) {
        DStarLite planner;
        planner.weights.gamma = gamma;
        auto r = planner.plan(p3);
        sweepCsv << gamma << "," << r.totalCost << "," << r.safetyScore << "," << r.objectiveScore << "\n";
    }
    sweepCsv.close();
    std::cout << "Safety-weight sweep written to results/safety_weight_sweep.csv\n";
}

static void printUsage() {
    std::cout << "Usage:\n"
              << "  safe_planner --test N [--export FILE.json]   Run test case N (1-6)\n"
              << "  safe_planner --demo                           Run all six test cases\n"
              << "  safe_planner --experiment [--seed N]          Run experimental evaluation\n"
              << "  safe_planner --help                           Show this message\n";
}

int main(int argc, char** argv) {
    std::vector<std::string> args(argv + 1, argv + argc);
    if (args.empty()) { printUsage(); return 0; }

    int testNum = -1;
    std::string exportPath;
    bool demo = false, experiment = false;
    unsigned int seed = 42;

    for (size_t i = 0; i < args.size(); ++i) {
        if (args[i] == "--test" && i + 1 < args.size()) {
            testNum = std::stoi(args[++i]);
        } else if (args[i] == "--export" && i + 1 < args.size()) {
            exportPath = args[++i];
        } else if (args[i] == "--demo") {
            demo = true;
        } else if (args[i] == "--experiment") {
            experiment = true;
        } else if (args[i] == "--seed" && i + 1 < args.size()) {
            seed = static_cast<unsigned int>(std::stoul(args[++i]));
        } else if (args[i] == "--help") {
            printUsage();
            return 0;
        }
    }

    if (demo) { runDemo(); return 0; }
    if (experiment) { runExperiments(seed); return 0; }
    if (testNum >= 1 && testNum <= 6) { runTestByNumber(testNum, exportPath); return 0; }

    printUsage();
    return 0;
}
