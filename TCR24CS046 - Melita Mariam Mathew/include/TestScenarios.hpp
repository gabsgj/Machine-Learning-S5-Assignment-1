#ifndef TEST_SCENARIOS_HPP
#define TEST_SCENARIOS_HPP

#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"
#include "LPAStarPlanner.hpp"
#include "DstarlitePlanner.hpp"
#include <string>
#include <vector>

struct TestCaseResult {
    int testCaseNumber;
    std::string title;
    bool passed;
    std::string expectedBehavior;
    std::string actualPath;
    double initialCost;
    double updatedCost;
    double minSafetyDist;
    uint64_t badStatesVisited;
    uint64_t initialExploredNodes;
    uint64_t replanExploredNodes;
    double initialTimeMs;
    double replanTimeMs;
    std::string notes;
};

class TestScenarios {
public:
    static PlanningProblem createTestCase1(); // Basic Reachability
    static PlanningProblem createTestCase2(); // Bad State Avoidance
    static PlanningProblem createTestCase3(); // Safety Margin
    static PlanningProblem createTestCase4(); // Dynamic Transition
    static PlanningProblem createTestCase5(); // Goal Update
    static PlanningProblem createTestCase6(); // Transition Addition

    // Synthetic large-scale benchmark generator
    static PlanningProblem generateRandomCartesianGraph(
        uint64_t numStates, size_t dimensions, double badStateRatio, double edgeDensity, unsigned int seed = 42);

    // Test runner functions
    static TestCaseResult runTestCase1();
    static TestCaseResult runTestCase2();
    static TestCaseResult runTestCase3();
    static TestCaseResult runTestCase4();
    static TestCaseResult runTestCase5();
    static TestCaseResult runTestCase6();

    static std::vector<TestCaseResult> runAllTestCases();
};

#endif // TEST_SCENARIOS_HPP
