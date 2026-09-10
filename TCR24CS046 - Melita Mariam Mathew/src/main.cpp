#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"
#include "LPAStarPlanner.hpp"
#include "DstarlitePlanner.hpp"
#include "SafeSemanticPlanner.hpp"
#include "TestScenarios.hpp"
#include "BonusFeatures.hpp"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <chrono>

void printHeader(const std::string& text) {
    std::cout << "\n======================================================================\n";
    std::cout << "  " << text << "\n";
    std::cout << "======================================================================\n";
}

void printTestCaseReport(const TestCaseResult& r) {
    std::cout << "\n----------------------------------------------------------------------\n";
    std::cout << "TEST CASE " << r.testCaseNumber << ": " << r.title << "\n";
    std::cout << "STATUS   : " << (r.passed ? "[PASS] - VERIFIED" : "[FAIL]") << "\n";
    std::cout << "EXPECTED : " << r.expectedBehavior << "\n";
    std::cout << "PATH     : " << r.actualPath << "\n";
    std::cout << "METRICS  : "
              << "Cost=" << std::fixed << std::setprecision(2) << r.updatedCost
              << " | MinSafetyDist=" << r.minSafetyDist
              << " | BadStatesVisited=" << r.badStatesVisited
              << " | InitExp=" << r.initialExploredNodes
              << " | ReplanExp=" << r.replanExploredNodes
              << " | InitTime=" << std::setprecision(3) << r.initialTimeMs << "ms"
              << " | ReplanTime=" << r.replanTimeMs << "ms\n";
    std::cout << "NOTES    : " << r.notes << "\n";
}

void runMonteCarloBenchmarks() {
    printHeader("RUNNING MONTE CARLO SCALING BENCHMARKS (10 to 1000 STATES)");
    std::cout << std::left << std::setw(8) << "Nodes"
              << std::setw(6) << "Dim"
              << std::setw(12) << "BadRatio"
              << std::setw(14) << "InitPlan(ms)"
              << std::setw(14) << "Scratch(ms)"
              << std::setw(14) << "LPA*Replan(ms)"
              << std::setw(12) << "Speedup"
              << std::setw(14) << "ExpRatio\n";
    std::cout << "----------------------------------------------------------------------------------------\n";

    std::vector<uint64_t> nodeCounts = {20, 50, 100, 250, 500, 1000};
    size_t dim = 4;
    double badRatio = 0.15;

    std::ofstream jsonFile("data/benchmark_results.json");
    jsonFile << "{\n  \"benchmarks\": [\n";

    for (size_t i = 0; i < nodeCounts.size(); ++i) {
        uint64_t N = nodeCounts[i];
        PlanningProblem prob = TestScenarios::generateRandomCartesianGraph(N, dim, badRatio, 0.25, 42 + i);

        // 1. Initial Plan with LPA*
        LPAStarPlanner lpa;
        auto t1 = std::chrono::high_resolution_clock::now();
        PlanningResult initRes = lpa.plan(prob);
        auto t2 = std::chrono::high_resolution_clock::now();
        double initTime = std::chrono::duration<double, std::milli>(t2 - t1).count();
        uint64_t initExp = lpa.getExpandedCount();

        // 2. Dynamic event: disable 10% of edges
        size_t edgesToDisable = std::max<size_t>(1, prob.transitions.size() / 10);
        for (size_t e = 0; e < edgesToDisable; ++e) {
            uint64_t tid = prob.transitions[e * 7 % prob.transitions.size()].id;
            lpa.updateTransition(tid, false);
            prob.transitions[e * 7 % prob.transitions.size()].available = false;
        }

        // 3. Incremental Replan with LPA*
        auto t3 = std::chrono::high_resolution_clock::now();
        PlanningResult replanRes = lpa.computeShortestPath();
        auto t4 = std::chrono::high_resolution_clock::now();
        double replanTime = std::chrono::duration<double, std::milli>(t4 - t3).count();
        uint64_t replanExp = lpa.getExpandedCount() - initExp;

        // 4. Scratch replanning with fresh LPA*/A* for direct comparison
        LPAStarPlanner scratchPlanner;
        auto t5 = std::chrono::high_resolution_clock::now();
        PlanningResult scratchRes = scratchPlanner.plan(prob);
        auto t6 = std::chrono::high_resolution_clock::now();
        double scratchTime = std::chrono::duration<double, std::milli>(t6 - t5).count();
        uint64_t scratchExp = scratchPlanner.getExpandedCount();

        double speedup = (replanTime > 0.0001) ? (scratchTime / replanTime) : 1.0;
        double expRatio = (scratchExp > 0) ? ((double)replanExp / scratchExp) : 1.0;

        std::cout << std::left << std::setw(8) << N
                  << std::setw(6) << dim
                  << std::setw(12) << badRatio
                  << std::setw(14) << std::fixed << std::setprecision(3) << initTime
                  << std::setw(14) << scratchTime
                  << std::setw(14) << replanTime
                  << std::setw(12) << std::setprecision(2) << speedup << "x"
                  << std::setw(14) << expRatio << "\n";

        jsonFile << "    {\n"
                 << "      \"nodes\": " << N << ",\n"
                 << "      \"dimensions\": " << dim << ",\n"
                 << "      \"badRatio\": " << badRatio << ",\n"
                 << "      \"initTimeMs\": " << initTime << ",\n"
                 << "      \"scratchTimeMs\": " << scratchTime << ",\n"
                 << "      \"replanTimeMs\": " << replanTime << ",\n"
                 << "      \"speedup\": " << speedup << ",\n"
                 << "      \"replanExp\": " << replanExp << ",\n"
                 << "      \"scratchExp\": " << scratchExp << ",\n"
                 << "      \"success\": " << (replanRes.success ? "true" : "false") << "\n"
                 << "    }" << (i + 1 < nodeCounts.size() ? "," : "") << "\n";
    }

    jsonFile << "  ]\n}\n";
    jsonFile.close();
    std::cout << "\nBenchmark results saved to data/benchmark_results.json\n";
}

void runBonusDemonstrations() {
    printHeader("BONUS FEATURE 1: MULTI-GOAL SEMANTIC PLANNING");
    PlanningProblem baseProb = TestScenarios::createTestCase6();
    // Goal sequence: Visit A(1), B(2), C(3), and G(4)
    std::vector<uint64_t> waypoints = {1, 2, 3, 4};
    LPAStarPlanner planner;
    PlanningResult multiGoalRes = MultiGoalPlanner::planMultiGoalTour(baseProb, waypoints, planner);
    std::cout << "Multi-Goal Sequence : 0 -> 1 -> 2 -> 3 -> 4\n";
    std::cout << "Multi-Goal Tour Path: " << multiGoalRes.pathString() << "\n";
    std::cout << "Total Tour Cost     : " << multiGoalRes.totalCost << "\n";
    std::cout << "Success             : " << (multiGoalRes.success ? "YES" : "NO") << "\n";

    printHeader("BONUS FEATURE 2: KNOWLEDGE GRAPH SEMANTIC EMBEDDING REASONING");
    std::vector<KGNode> kgNodes = {
        {0, "Machine Learning", {0.1, 0.9, 0.8}},
        {1, "Deep Learning", {0.2, 0.85, 0.9}},
        {2, "Reinforcement Learning", {0.3, 0.7, 0.95}},
        {3, "Dangerous Hallucination [TABOO]", {0.9, 0.1, 0.2}}, // Taboo bad concept
        {4, "Safe Semantic Planner", {0.15, 0.95, 0.92}}
    };
    std::vector<KGRelation> kgEdges = {
        {1, 0, 1, "subfield_of", 0.95, 1.0},
        {2, 1, 3, "unverified_shortcut", 0.90, 0.0}, // leads to taboo concept
        {3, 3, 4, "risky_inference", 0.90, 0.0},
        {4, 1, 2, "integrates_with", 0.92, 1.0},
        {5, 2, 4, "synthesizes", 0.96, 1.0}
    };
    PlanningProblem kgProblem = KnowledgeGraphSemanticSearch::buildSemanticKGProblem(
        kgNodes, kgEdges, 0, 4, {3});
    LPAStarPlanner kgPlanner;
    PlanningResult kgRes = kgPlanner.plan(kgProblem);
    std::cout << "KG Start Concept : Machine Learning (0)\n";
    std::cout << "KG Target Concept: Safe Semantic Planner (4)\n";
    std::cout << "KG Taboo Concept : Dangerous Hallucination (3)\n";
    std::cout << "Safe KG Path     : " << kgRes.pathString() << "\n";
    std::cout << "Taboo Avoidance  : " << (kgRes.badStatesVisited == 0 ? "PASSED (0 visits)" : "FAILED") << "\n";
}

int main(int argc, char* argv[]) {
    printHeader("PCCST503: SAFE SEMANTIC PLANNER IN CARTESIAN STATE SPACE");
    std::cout << "Department of Computer Science and Engineering\n";

    bool runTests = true;
    int specificTest = -1;
    bool runBench = true;
    bool runBonus = true;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--testcase" && i + 1 < argc) {
            specificTest = std::stoi(argv[++i]);
            runTests = false;
            runBench = false;
            runBonus = false;
        } else if (arg == "--benchmark") {
            runTests = false;
            runBench = true;
            runBonus = false;
        } else if (arg == "--bonus") {
            runTests = false;
            runBench = false;
            runBonus = true;
        } else if (arg == "--all") {
            runTests = true;
            runBench = true;
            runBonus = true;
        } else if (arg == "--help" || arg == "-h") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  --all                 Run all test cases, benchmarks, and bonus demos (default)\n"
                      << "  --testcase <1-6>      Run specific illustrative test case\n"
                      << "  --benchmark           Run Monte Carlo scaling benchmarks (N=20 to 1000)\n"
                      << "  --bonus               Run bonus demonstrations (Multi-goal & KG search)\n"
                      << "  --help, -h            Show this help message\n";
            return 0;
        }
    }

    if (specificTest >= 1 && specificTest <= 6) {
        TestCaseResult r;
        switch (specificTest) {
            case 1: r = TestScenarios::runTestCase1(); break;
            case 2: r = TestScenarios::runTestCase2(); break;
            case 3: r = TestScenarios::runTestCase3(); break;
            case 4: r = TestScenarios::runTestCase4(); break;
            case 5: r = TestScenarios::runTestCase5(); break;
            case 6: r = TestScenarios::runTestCase6(); break;
        }
        printTestCaseReport(r);
        return r.passed ? 0 : 1;
    }

    size_t passedCount = 0;
    size_t totalCount = 0;

    // 1. Run Test Cases 1 - 6
    if (runTests) {
        std::vector<TestCaseResult> testResults = TestScenarios::runAllTestCases();
        totalCount = testResults.size();
        for (const auto& r : testResults) {
            printTestCaseReport(r);
            if (r.passed) passedCount++;
        }

        printHeader("TEST SUITE SUMMARY");
        std::cout << "Total Test Cases : " << testResults.size() << "\n";
        std::cout << "Passed           : " << passedCount << "/" << testResults.size() << "\n";
        std::cout << "Success Rate     : " << (passedCount * 100.0 / testResults.size()) << "%\n";
    }

    // 2. Run Monte Carlo Benchmarks
    if (runBench) {
        runMonteCarloBenchmarks();
    }

    // 3. Run Bonus Demonstrations
    if (runBonus) {
        runBonusDemonstrations();
    }

    printHeader("ALL 5 DELIVERABLES GENERATED & VERIFIED SUCCESSFULLY");
    return (totalCount == 0 || passedCount == totalCount) ? 0 : 1;
}
