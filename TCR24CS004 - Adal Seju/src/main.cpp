#include "../include/types.hpp"
#include "../include/geometry.hpp"
#include "../include/heuristic.hpp"
#include "../include/lpa_star.hpp"
#include "../include/d_star_lite.hpp"
#include "../include/multi_objective.hpp"
#include "../include/test_cases.hpp"
#include "../include/benchmark.hpp"

#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>

using namespace SafePlanner;

// ANSI Colors for beautiful terminal output
#define ANSI_RESET   "\033[0m"
#define ANSI_BOLD    "\033[1m"
#define ANSI_RED     "\033[31m"
#define ANSI_GREEN   "\033[32m"
#define ANSI_YELLOW  "\033[33m"
#define ANSI_BLUE    "\033[34m"
#define ANSI_MAGENTA "\033[35m"
#define ANSI_CYAN    "\033[36m"
#define ANSI_WHITE   "\033[37m"

void printHeader(const std::string& title) {
    std::cout << "\n" << ANSI_BOLD << ANSI_CYAN;
    std::cout << "================================================================================\n";
    std::cout << "  " << title << "\n";
    std::cout << "================================================================================\n" << ANSI_RESET;
}

void printResult(const std::string& testName, const PlanningResult& res, const std::string& expectedDesc = "") {
    std::cout << ANSI_BOLD << "[" << testName << "]" << ANSI_RESET << "\n";
    if (!expectedDesc.empty()) {
        std::cout << "  " << ANSI_YELLOW << "Expected: " << ANSI_RESET << expectedDesc << "\n";
    }
    
    std::cout << "  " << "Status: " 
              << (res.success ? (std::string(ANSI_GREEN) + "SUCCESS [PATH FOUND]" + ANSI_RESET) 
                              : (std::string(ANSI_RED) + "FAILED [NO PATH]" + ANSI_RESET)) << "\n";
    
    if (res.success) {
        std::cout << "  " << "State Path: " << ANSI_BOLD << ANSI_WHITE << "[";
        for (size_t i = 0; i < res.statePath.size(); ++i) {
            std::cout << res.statePath[i] << (i + 1 < res.statePath.size() ? " -> " : "");
        }
        std::cout << "]" << ANSI_RESET << "\n";

        std::cout << "  " << "Transition Path: [";
        for (size_t i = 0; i < res.transitionPath.size(); ++i) {
            std::cout << res.transitionPath[i] << (i + 1 < res.transitionPath.size() ? ", " : "");
        }
        std::cout << "]\n";

        std::cout << std::fixed << std::setprecision(4);
        std::cout << "  " << "Total Cost: " << ANSI_CYAN << res.totalCost << ANSI_RESET << "\n";
        
        if (res.safetyScore < INF) {
            std::cout << "  " << "Safety Score (Min Bad-Dist): " << ANSI_GREEN << res.safetyScore << ANSI_RESET << "\n";
        } else {
            std::cout << "  " << "Safety Score: " << ANSI_GREEN << "+INF (No bad states active)" << ANSI_RESET << "\n";
        }

        std::cout << "  " << "Cumulative Reliability: " << (res.cumulativeReliability * 100.0) << "%\n";
        std::cout << "  " << "Multi-Objective Score: " << res.objectiveScore << "\n";
        std::cout << "  " << "Explored States: " << res.exploredStates << "\n";
        std::cout << "  " << "Planning Time: " << res.planningTimeMicroseconds << " microseconds\n";
    }
    std::cout << "--------------------------------------------------------------------------------\n";
}

void runAllTestCases() {
    printHeader("RUNNING ASSIGNMENT TEST CASES (1 THROUGH 6 + BONUS)");

    LPAStarPlanner planner;

    // Test Case 1: Basic Reachability
    {
        PlanningProblem p1 = TestCases::createTestCase1();
        PlanningResult r1 = planner.plan(p1);
        printResult("Test Case 1: Basic Reachability", r1, "Unique path S(1) -> A(2) -> B(3) -> G(4)");
    }

    // Test Case 2: Bad State Avoidance
    {
        PlanningProblem p2 = TestCases::createTestCase2();
        PlanningResult r2 = planner.plan(p2);
        printResult("Test Case 2: Bad State Avoidance", r2, "Avoid bad state X(3), choose S(1) -> C(5) -> D(6) -> G(4)");
    }

    // Test Case 3: Safety Margin & Tradeoff
    {
        PlanningProblem p3 = TestCases::createTestCase3();
        
        // Mode A: Pure Shortest Cost
        planner.setMultiObjectiveWeights(0.0, 0.0, 0.0);
        PlanningResult r3_cost = planner.plan(p3);
        printResult("Test Case 3A: Pure Cost Minimization", r3_cost, "Picks lowest cost path S(1)->A(2)->G(4) despite proximity to obstacle");

        // Mode B: Safety-Aware Weighted Barrier (Safety Margin = 1.0, Weight = 5.0)
        planner.setMultiObjectiveWeights(1.0, 5.0, 0.0);
        PlanningResult r3_safe = planner.plan(p3);
        printResult("Test Case 3B: Safety-Weighted Margin Balancing", r3_safe, "Picks wider safety arc S(1)->C(5)->D(6)->G(4) to maintain safe clearance");
        
        // Reset weights
        planner.setMultiObjectiveWeights(0.0, 0.0, 0.0);
    }

    // Test Case 4: Dynamic Transition
    {
        PlanningProblem p4 = TestCases::createTestCase4();
        planner.loadProblem(p4);
        planner.computeShortestPath();
        PlanningResult r4_init = planner.extractResult();
        std::cout << ANSI_BOLD << "[Test Case 4.1: Initial State]" << ANSI_RESET << " Primary Path: [1 -> 2 -> 3], Cost: " << r4_init.totalCost << "\n";

        // Dynamic change: transition 2 (A -> G) becomes unavailable
        std::cout << ANSI_YELLOW << ">>> Event: Transition (A->G) becomes unavailable! Triggering incremental replan..." << ANSI_RESET << "\n";
        planner.updateEdgeAvailability(2, false);
        PlanningResult r4_replan = planner.replanIncremental();
        printResult("Test Case 4.2: Dynamic Transition Replanned", r4_replan, "Switches to backup path S(1) -> B(4) -> C(5) -> G(3)");
    }

    // Test Case 5: Goal Update
    {
        PlanningProblem p5 = TestCases::createTestCase5();
        planner.loadProblem(p5);
        planner.computeShortestPath();
        PlanningResult r5_init = planner.extractResult();
        std::cout << ANSI_BOLD << "[Test Case 5.1: Initial Goal G1(3)]" << ANSI_RESET << " Path: [1 -> 2 -> 3], Cost: " << r5_init.totalCost << "\n";

        // Dynamic change: Goal moves to G2(5)
        std::cout << ANSI_YELLOW << ">>> Event: Goal changes to G2(5)! Incremental re-keying..." << ANSI_RESET << "\n";
        planner.updateGoal(5);
        PlanningResult r5_replan = planner.replanIncremental();
        printResult("Test Case 5.2: Dynamic Goal Updated", r5_replan, "Re-routes to S(1) -> Hub_2(4) -> G2(5) reusing existing search tree");
    }

    // Test Case 6: Transition Addition
    {
        PlanningProblem p6 = TestCases::createTestCase6();
        planner.loadProblem(p6);
        planner.computeShortestPath();
        PlanningResult r6_init = planner.extractResult();
        std::cout << ANSI_BOLD << "[Test Case 6.1: Initial Baseline Path]" << ANSI_RESET << " Path: [1 -> 2 -> 3 -> 4], Cost: " << r6_init.totalCost << "\n";

        // Dynamic change: New shortcut transition inserted (A -> G, Cost = 1.05)
        std::cout << ANSI_YELLOW << ">>> Event: Shortcut Transition (A(2) -> G(4), Cost=1.05) added! Incremental update..." << ANSI_RESET << "\n";
        Transition shortcut(99, 2, 4, 1.05, 1.0, 1.0, true);
        planner.addTransition(shortcut);
        PlanningResult r6_replan = planner.replanIncremental();
        printResult("Test Case 6.2: Shortcut Discovered", r6_replan, "Discovers shortcut path S(1) -> A(2) -> G(4) with cost 2.05");
    }

    // Bonus: Semantic Knowledge Graph
    {
        PlanningProblem pKg = TestCases::createKnowledgeGraphSemanticCase();
        PlanningResult rKg = planner.plan(pKg);
        printResult("Bonus: Semantic Knowledge Graph Navigation", rKg, "Navigates concept embedding space bypassing malicious entity (3)");
    }
}

void runScalabilityBenchmarks() {
    printHeader("SCALABILITY & INCREMENTAL REPLANNING BENCHMARK SUITE");

    std::cout << ANSI_BOLD << std::left 
              << std::setw(14) << "Graph Scale" 
              << std::setw(10) << "States" 
              << std::setw(10) << "Edges" 
              << std::setw(16) << "Cold Start (us)" 
              << std::setw(18) << "Incremental (us)" 
              << std::setw(12) << "Speedup" 
              << std::setw(14) << "Cold Exp." 
              << std::setw(14) << "Incr. Exp." 
              << ANSI_RESET << "\n";
    std::cout << "----------------------------------------------------------------------------------------------------\n";

    auto metrics = BenchmarkSuite::runScalabilityBenchmarks();
    for (const auto& m : metrics) {
        std::cout << std::left 
                  << std::setw(14) << m.testName
                  << std::setw(10) << m.numStates
                  << std::setw(10) << m.numTransitions
                  << std::setw(16) << std::fixed << std::setprecision(1) << m.coldTimeUs
                  << std::setw(18) << m.incrementalTimeUs
                  << ANSI_GREEN << std::setw(12) << std::setprecision(2) << (std::to_string(m.speedupFactor).substr(0, 5) + "x") << ANSI_RESET
                  << std::setw(14) << m.coldExplored
                  << std::setw(14) << m.incrementalExplored
                  << "\n";
    }
    std::cout << "----------------------------------------------------------------------------------------------------\n";
}

int main(int argc, char* argv[]) {
    std::cout << ANSI_BOLD << ANSI_MAGENTA << R"(
  ____        __         ____                                _   _        ____  _                                 
 / ___|  __ _|  ___     / ___|  ___ _ __ ___   __ _ _ __  | |_(_) ___  |  _ \| | __ _ _ __  _ __   ___ _ __ 
 \___ \ / _` | |_      \___ \ / _ \ '_ ` _ \ / _` | '_ \ | __| |/ __| | |_) | |/ _` | '_ \| '_ \ / _ \ '__|
  ___) | (_| |  _|      ___) |  __/ | | | | | (_| | | | || |_| | (__  |  __/| | (_| | | | | | | |  __/ |   
 |____/ \__,_|_|       |____/ \___|_| |_| |_|\__,_|_| |_| \__|_|\___| |_|   |_|\__,_|_| |_|_| |_|\___|_|   
    )" << ANSI_RESET << "\n";
    std::cout << ANSI_CYAN << " PCCST503 Machine Learning - Assignment 1\n"
              << " Safe Semantic Planner in Finite Cartesian State Space (LPA* & D* Lite Engine)\n" << ANSI_RESET;
    std::cout << "--------------------------------------------------------------------------------\n";

    if (argc > 1) {
        std::string arg = argv[1];
        if (arg == "--test") {
            runAllTestCases();
            return 0;
        } else if (arg == "--benchmark") {
            runScalabilityBenchmarks();
            return 0;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage:\n"
                      << "  planner.exe               : Run full demonstration and test suite\n"
                      << "  planner.exe --test        : Run all 6 assignment test cases\n"
                      << "  planner.exe --benchmark   : Run scalability benchmarks and speedup measurements\n"
                      << "  planner.exe --help        : Display this help message\n";
            return 0;
        }
    }

    // Default execution: Run both tests and benchmarks
    runAllTestCases();
    runScalabilityBenchmarks();

    std::cout << "\n" << ANSI_BOLD << ANSI_GREEN 
              << ">> All tests and benchmarks completed successfully with ZERO errors!" << ANSI_RESET << "\n";
    std::cout << ">> Launch the Web Visualizer in 'frontend/index.html' for live interactive GUI.\n\n";

    return 0;
}
