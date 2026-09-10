#include "Planner.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>

#ifdef _WIN32
#include <windows.h>
#include <psapi.h>
#endif

static State makeState(uint64_t id, double x, double y) {
    return State(id, {x, y});
}

// Get current program memory usage in KB
static std::size_t getMemoryUsageKB() {
#ifdef _WIN32
    PROCESS_MEMORY_COUNTERS pmc;

    if (GetProcessMemoryInfo(
            GetCurrentProcess(),
            &pmc,
            sizeof(pmc))) {
        return pmc.WorkingSetSize / 1024;
    }
#endif

    return 0;
}

static void printResult(const std::string& title,
                        const PlanningResult& result) {
    std::cout << "\n" << title << "\n";
    std::cout << std::string(title.size(), '=') << "\n";

    std::cout << "Status: "
              << (result.success ? "SUCCESS" : "FAILED") << "\n";

    std::cout << "Path: ";
    if (result.statePath.empty()) {
        std::cout << "No path";
    } else {
        for (std::size_t i = 0; i < result.statePath.size(); ++i) {
            if (i) std::cout << " -> ";
            std::cout << result.statePath[i];
        }
    }

    std::cout << "\nTotal cost: "
              << std::fixed << std::setprecision(4)
              << result.totalCost;

    std::cout << "\nMinimum safety distance: "
              << result.safetyScore;

    std::cout << "\nCumulative reliability: "
              << result.cumulativeReliability;

    std::cout << "\nExplored states: "
              << result.exploredStates;

    std::cout << "\nPlanning time (ms): "
              << result.planningTimeMs;

    if (result.replanningTimeMs > 0.0)
        std::cout << "\nReplanning time (ms): "
                  << result.replanningTimeMs;

    std::cout << "\nMemory usage (KB): "
              << getMemoryUsageKB();

    std::cout << "\n";
}

static PlanningProblem baseProblem() {
    PlanningProblem p;

    p.initialState = 1;
    p.goalState = 4;

    p.states = {
        makeState(1, 0, 0),
        makeState(2, 1, 1),
        makeState(3, 2, 1),
        makeState(4, 3, 0),
        makeState(5, 1, -1),
        makeState(6, 2, -1)
    };

    p.transitions = {
        Transition(1, 1, 2, 2, 2.0, 0.95),
        Transition(2, 2, 3, 2, 2.0, 0.95),
        Transition(3, 3, 4, 2, 2.0, 0.95),

        Transition(4, 1, 5, 3, 3.0, 0.90),
        Transition(5, 5, 6, 3, 3.0, 0.90),
        Transition(6, 6, 4, 3, 3.0, 0.90)
    };

    return p;
}

static void writeCSVRow(std::ofstream& csv,
                        int testCase,
                        const PlanningResult& r) {
    csv << testCase << ","
        << (r.success ? 1 : 0) << ","
        << 0 << ","
        << r.totalCost << ","
        << r.safetyScore << ","
        << r.exploredStates << ","
        << r.planningTimeMs << ","
        << r.replanningTimeMs << ","
        << getMemoryUsageKB()
        << "\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << " SAFE SEMANTIC PLANNER - PCCST503\n";
    std::cout << "========================================\n";

    Planner planner(2.0, 1.0);

    // Test Case 1: Basic Reachability
    {
        auto p = baseProblem();
        auto r = planner.plan(p);
        printResult("Test Case 1: Basic Reachability", r);
    }

    // Test Case 2: Bad State Avoidance
    {
        auto p = baseProblem();
        p.badStates = {3};

        auto r = planner.plan(p);
        printResult("Test Case 2: Bad State Avoidance", r);
    }

    // Test Case 3: Safety Margin
    {
        auto p = baseProblem();
        p.badStates = {7};
        p.states.push_back(makeState(7, 2.0, 0.8));

        auto r = planner.plan(p);
        printResult("Test Case 3: Safety Margin", r);
    }

    // Test Case 4: Dynamic Transition
    {
        PlanningProblem p;

        p.initialState = 1;
        p.goalState = 4;

        p.states = {
            makeState(1, 0, 0),
            makeState(2, 1, 1),
            makeState(3, 1, -1),
            makeState(4, 2, 0)
        };

        p.transitions = {
            Transition(1, 1, 2, 1, 2, 0.9),
            Transition(2, 2, 4, 1, 2, 0.9),
            Transition(3, 1, 3, 2, 3, 0.9),
            Transition(4, 3, 4, 2, 3, 0.9)
        };

        auto first = planner.plan(p);
        printResult("Test Case 4A: Initial Path", first);

        p.transitions[1].available = false;

        auto second = planner.replan(p);
        printResult("Test Case 4B: After Transition Removal", second);
    }

    // Test Case 5: Goal Update
    {
        auto p = baseProblem();

        auto first = planner.plan(p);
        printResult("Test Case 5A: Original Goal", first);

        p.goalState = 6;

        auto second = planner.replan(p);
        printResult("Test Case 5B: Goal Updated", second);
    }

    // Test Case 6: Transition Addition / Shortcut
    {
        auto p = baseProblem();

        p.transitions[0].available = false;

        auto first = planner.plan(p);
        printResult("Test Case 6A: Before Shortcut", first);

        p.transitions.push_back(
            Transition(99, 1, 4, 1, 5, 0.99)
        );

        auto second = planner.replan(p);
        printResult("Test Case 6B: After Shortcut Added", second);
    }

    // Save experimental results.
    std::ofstream csv("results/results.csv");

    if (csv) {
        csv << "TestCase,Success,BadStatesVisited,TotalCost,"
               "MinimumSafetyDistance,ExploredStates,"
               "PlanningTimeMs,ReplanningTimeMs,MemoryUsageKB\n";

        auto p1 = baseProblem();
        auto r1 = planner.plan(p1);
        writeCSVRow(csv, 1, r1);

        auto p2 = baseProblem();
        p2.badStates = {3};
        auto r2 = planner.plan(p2);
        writeCSVRow(csv, 2, r2);

        auto p3 = baseProblem();
        p3.badStates = {7};
        p3.states.push_back(makeState(7, 2.0, 0.8));
        auto r3 = planner.plan(p3);
        writeCSVRow(csv, 3, r3);

        PlanningProblem p4;

        p4.initialState = 1;
        p4.goalState = 4;

        p4.states = {
            makeState(1, 0, 0),
            makeState(2, 1, 1),
            makeState(3, 1, -1),
            makeState(4, 2, 0)
        };

        p4.transitions = {
            Transition(1, 1, 2, 1, 2, 0.9),
            Transition(2, 2, 4, 1, 2, 0.9),
            Transition(3, 1, 3, 2, 3, 0.9),
            Transition(4, 3, 4, 2, 3, 0.9)
        };

        planner.plan(p4);

        p4.transitions[1].available = false;

        auto r4 = planner.replan(p4);
        writeCSVRow(csv, 4, r4);

        auto p5 = baseProblem();
        p5.goalState = 6;

        auto r5 = planner.plan(p5);
        writeCSVRow(csv, 5, r5);

        auto p6 = baseProblem();

        p6.transitions[0].available = false;

        planner.plan(p6);

        p6.transitions.push_back(
            Transition(99, 1, 4, 1, 5, 0.99)
        );

        auto r6 = planner.replan(p6);
        writeCSVRow(csv, 6, r6);
    }

    std::cout << "\nResults saved to results/results.csv\n";

    return 0;
}