#ifndef TEST_CASES_HPP
#define TEST_CASES_HPP

#include "Types.hpp"
#include "SafeSemanticPlanner.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <memory>

namespace TestSuite {

/**
 * Helper to serialize planning problem and results to JSON for the visualizer.
 */
inline std::string escapeJson(const std::string& s) {
    std::ostringstream o;
    for (char c : s) {
        if (c == '"') o << "\\\"";
        else if (c == '\\') o << "\\\\";
        else if (c == '\b') o << "\\b";
        else if (c == '\f') o << "\\f";
        else if (c == '\n') o << "\\n";
        else if (c == '\r') o << "\\r";
        else if (c == '\t') o << "\\t";
        else o << c;
    }
    return o.str();
}

inline std::string serializeProblemToJson(
    const std::string& testName,
    const std::string& description,
    const PlanningProblem& problem,
    const PlanningResult& initialResult,
    const PlanningResult* replanResult = nullptr,
    const std::string& dynamicAction = ""
) {
    std::ostringstream oss;
    oss << "{\n";
    oss << "  \"testName\": \"" << escapeJson(testName) << "\",\n";
    oss << "  \"description\": \"" << escapeJson(description) << "\",\n";
    oss << "  \"initialState\": " << problem.initialState << ",\n";
    oss << "  \"goalState\": " << problem.goalState << ",\n";
    
    // Bad states
    oss << "  \"badStates\": [";
    for (size_t i = 0; i < problem.badStates.size(); ++i) {
        oss << problem.badStates[i] << (i + 1 < problem.badStates.size() ? ", " : "");
    }
    oss << "],\n";

    // States
    oss << "  \"states\": [\n";
    for (size_t i = 0; i < problem.states.size(); ++i) {
        const auto& s = problem.states[i];
        oss << "    {\"id\": " << s.id << ", \"embedding\": [";
        for (size_t j = 0; j < s.embedding.size(); ++j) {
            oss << s.embedding[j] << (j + 1 < s.embedding.size() ? ", " : "");
        }
        oss << "]}" << (i + 1 < problem.states.size() ? ",\n" : "\n");
    }
    oss << "  ],\n";

    // Transitions
    oss << "  \"transitions\": [\n";
    for (size_t i = 0; i < problem.transitions.size(); ++i) {
        const auto& t = problem.transitions[i];
        oss << "    {\"id\": " << t.id << ", \"from\": " << t.from << ", \"to\": " << t.to 
            << ", \"cost\": " << t.cost << ", \"safety\": " << t.safety 
            << ", \"reliability\": " << t.reliability << ", \"available\": " << (t.available ? "true" : "false")
            << "}" << (i + 1 < problem.transitions.size() ? ",\n" : "\n");
    }
    oss << "  ],\n";

    // Initial Result
    oss << "  \"initialResult\": {\n";
    oss << "    \"success\": " << (initialResult.success ? "true" : "false") << ",\n";
    oss << "    \"totalCost\": " << initialResult.totalCost << ",\n";
    oss << "    \"safetyScore\": " << initialResult.safetyScore << ",\n";
    oss << "    \"minSafetyDistance\": " << initialResult.minSafetyDistance << ",\n";
    oss << "    \"cumulativeReliability\": " << initialResult.cumulativeReliability << ",\n";
    oss << "    \"multiObjectiveScore\": " << initialResult.multiObjectiveScore << ",\n";
    oss << "    \"exploredStatesCount\": " << initialResult.exploredStatesCount << ",\n";
    oss << "    \"executionTimeMicroseconds\": " << initialResult.executionTimeMicroseconds << ",\n";
    oss << "    \"statePath\": [";
    for (size_t i = 0; i < initialResult.statePath.size(); ++i) {
        oss << initialResult.statePath[i] << (i + 1 < initialResult.statePath.size() ? ", " : "");
    }
    oss << "],\n";
    oss << "    \"transitionPath\": [";
    for (size_t i = 0; i < initialResult.transitionPath.size(); ++i) {
        oss << initialResult.transitionPath[i] << (i + 1 < initialResult.transitionPath.size() ? ", " : "");
    }
    oss << "]\n";
    oss << "  }";

    // Optional Replan Result
    if (replanResult != nullptr) {
        oss << ",\n  \"dynamicAction\": \"" << escapeJson(dynamicAction) << "\",\n";
        oss << "  \"replanResult\": {\n";
        oss << "    \"success\": " << (replanResult->success ? "true" : "false") << ",\n";
        oss << "    \"totalCost\": " << replanResult->totalCost << ",\n";
        oss << "    \"safetyScore\": " << replanResult->safetyScore << ",\n";
        oss << "    \"minSafetyDistance\": " << replanResult->minSafetyDistance << ",\n";
        oss << "    \"cumulativeReliability\": " << replanResult->cumulativeReliability << ",\n";
        oss << "    \"multiObjectiveScore\": " << replanResult->multiObjectiveScore << ",\n";
        oss << "    \"exploredStatesCount\": " << replanResult->exploredStatesCount << ",\n";
        oss << "    \"executionTimeMicroseconds\": " << replanResult->executionTimeMicroseconds << ",\n";
        oss << "    \"statePath\": [";
        for (size_t i = 0; i < replanResult->statePath.size(); ++i) {
            oss << replanResult->statePath[i] << (i + 1 < replanResult->statePath.size() ? ", " : "");
        }
        oss << "],\n";
        oss << "    \"transitionPath\": [";
        for (size_t i = 0; i < replanResult->transitionPath.size(); ++i) {
            oss << replanResult->transitionPath[i] << (i + 1 < replanResult->transitionPath.size() ? ", " : "");
        }
        oss << "]\n";
        oss << "  }";
    }

    oss << "\n}";
    return oss.str();
}

/**
 * Test Case 1: Basic Reachability
 * Graph: S (0) -> A (1) -> B (2) -> G (3)
 * Expected result: planner returns unique valid path [0, 1, 2, 3]
 */
inline PlanningProblem createTestCase1() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {};

    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {10.0, 0.0}),  // A
        State(2, {20.0, 0.0}),  // B
        State(3, {30.0, 0.0})   // G
    };

    p.transitions = {
        Transition(1, 0, 1, 5.0, 1.0, 0.98, true),
        Transition(2, 1, 2, 5.0, 1.0, 0.95, true),
        Transition(3, 2, 3, 5.0, 1.0, 0.99, true)
    };

    return p;
}

/**
 * Test Case 2: Bad State Avoidance
 * Path 1: S (0) -> A (1) -> X (2) -> G (3) [where X is Bad]
 * Path 2: S (0) -> C (4) -> D (5) -> G (3)
 * Expected result: second path [0, 4, 5, 3] must be selected.
 */
inline PlanningProblem createTestCase2() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {2}; // X is a bad state

    p.states = {
        State(0, {0.0, 10.0}),   // S
        State(1, {10.0, 15.0}),  // A
        State(2, {20.0, 15.0}),  // X (Bad)
        State(3, {30.0, 10.0}),  // G
        State(4, {10.0, 5.0}),   // C
        State(5, {20.0, 5.0})    // D
    };

    p.transitions = {
        // Path 1 (through bad state X)
        Transition(1, 0, 1, 2.0, 0.5, 0.99, true),
        Transition(2, 1, 2, 2.0, 0.1, 0.99, true),
        Transition(3, 2, 3, 2.0, 0.1, 0.99, true),

        // Path 2 (safe alternative)
        Transition(4, 0, 4, 4.0, 1.0, 0.95, true),
        Transition(5, 4, 5, 4.0, 1.0, 0.95, true),
        Transition(6, 5, 3, 4.0, 1.0, 0.95, true)
    };

    return p;
}

/**
 * Test Case 3: Safety Margin
 * Path 1: S (0) -> N1 (1) -> N2 (2) -> G (5) (Low cost = 6, close to Bad State B1 at (15, 11))
 * Path 2: S (0) -> F1 (3) -> F2 (4) -> G (5) (Higher cost = 10, far away at Y = -10)
 * Expected result: When safety barrier field is enabled, planner selects the safer path Path 2.
 */
inline PlanningProblem createTestCase3() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 5;
    p.badStates = {6}; // B1

    p.states = {
        State(0, {0.0, 10.0}),   // S
        State(1, {10.0, 11.0}),  // N1 (Near obstacle)
        State(2, {20.0, 11.0}),  // N2 (Near obstacle)
        State(3, {10.0, -10.0}), // F1 (Far from obstacle)
        State(4, {20.0, -10.0}), // F2 (Far from obstacle)
        State(5, {30.0, 10.0}),  // G
        State(6, {15.0, 11.5})   // B1 (Bad state, right between N1 and N2)
    };

    p.transitions = {
        // Path 1 (Close to bad state, nominal cost = 6)
        Transition(1, 0, 1, 2.0, 0.4, 0.98, true),
        Transition(2, 1, 2, 2.0, 0.3, 0.98, true),
        Transition(3, 2, 5, 2.0, 0.4, 0.98, true),

        // Path 2 (Far from bad state, nominal cost = 10)
        Transition(4, 0, 3, 3.5, 1.0, 0.98, true),
        Transition(5, 3, 4, 3.0, 1.0, 0.98, true),
        Transition(6, 4, 5, 3.5, 1.0, 0.98, true)
    };

    return p;
}

/**
 * Test Case 4: Dynamic Transition
 * Initially: S (0) -> A (1) -> G (3) (Cost = 4)
 * Alternate: S (0) -> B (2) -> G (3) (Cost = 8)
 * Dynamic event: Transition (A, G) becomes unavailable.
 * Expected result: Incremental LPA* replanning reroutes to S -> B -> G without full graph rebuild.
 */
inline PlanningProblem createTestCase4() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {};

    p.states = {
        State(0, {0.0, 10.0}),   // S
        State(1, {15.0, 15.0}),  // A
        State(2, {15.0, 5.0}),   // B
        State(3, {30.0, 10.0})   // G
    };

    p.transitions = {
        Transition(1, 0, 1, 2.0, 1.0, 0.95, true),
        Transition(2, 1, 3, 2.0, 1.0, 0.95, true), // Will be disabled
        Transition(3, 0, 2, 4.0, 1.0, 0.95, true),
        Transition(4, 2, 3, 4.0, 1.0, 0.95, true)
    };

    return p;
}

/**
 * Test Case 5: Goal Update
 * Goal changes during execution from G1 (3) to G2 (4).
 * Expected result: Planner revises path incrementally to new goal G2.
 */
inline PlanningProblem createTestCase5() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3; // Initially G1
    p.badStates = {};

    p.states = {
        State(0, {0.0, 10.0}),   // S
        State(1, {10.0, 15.0}),  // A
        State(2, {10.0, 5.0}),   // B
        State(3, {25.0, 15.0}),  // G1
        State(4, {25.0, 5.0})    // G2
    };

    p.transitions = {
        Transition(1, 0, 1, 3.0, 1.0, 0.99, true),
        Transition(2, 1, 3, 3.0, 1.0, 0.99, true),
        Transition(3, 0, 2, 3.0, 1.0, 0.99, true),
        Transition(4, 2, 4, 3.0, 1.0, 0.99, true),
        Transition(5, 1, 2, 2.0, 1.0, 0.99, true)
    };

    return p;
}

/**
 * Test Case 6: Transition Addition
 * Base graph: S (0) -> A (1) -> B (2) -> G (3) (Cost = 9)
 * Dynamic event: New shortcut transition S (0) -> G (3) (Cost = 4) inserted.
 * Expected result: Incremental LPA* planner discovers improved direct shortcut.
 */
inline PlanningProblem createTestCase6() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {};

    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {10.0, 5.0}),  // A
        State(2, {20.0, 5.0}),  // B
        State(3, {30.0, 0.0})   // G
    };

    p.transitions = {
        Transition(1, 0, 1, 3.0, 1.0, 0.95, true),
        Transition(2, 1, 2, 3.0, 1.0, 0.95, true),
        Transition(3, 2, 3, 3.0, 1.0, 0.95, true)
    };

    return p;
}

/**
 * Bonus Benchmark: 2D Grid Environment with Obstacle Field
 */
inline PlanningProblem createGridBenchmark(int width = 8, int height = 8) {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = (width - 1) + (height - 1) * width;

    // Generate grid states
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint64_t id = x + y * width;
            p.states.push_back(State(id, {static_cast<double>(x * 10), static_cast<double>(y * 10)}));
        }
    }

    // Add obstacles in center
    for (int y = 2; y <= 5; ++y) {
        p.badStates.push_back(3 + y * width);
    }

    // Directed grid transitions (4-connected)
    uint64_t transId = 1;
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            uint64_t u = x + y * width;
            int dx[4] = {1, -1, 0, 0};
            int dy[4] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                int nx = x + dx[k];
                int ny = y + dy[k];
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    uint64_t v = nx + ny * width;
                    p.transitions.push_back(Transition(transId++, u, v, 10.0, 1.0, 0.99, true));
                }
            }
        }
    }

    return p;
}

/**
 * Bonus Benchmark: 8D Semantic Knowledge Graph Embeddings
 */
inline PlanningProblem createSemanticKGProblem() {
    PlanningProblem p;
    p.initialState = 0; // "User_Query"
    p.goalState = 5;    // "Verified_Decision"
    p.badStates = {3};  // "Hallucinated_Fact"

    p.states = {
        State(0, {0.1, 0.2, 0.8, 0.0, 0.5, 0.1, 0.4, 0.9}), // User_Query
        State(1, {0.3, 0.4, 0.7, 0.1, 0.6, 0.2, 0.5, 0.8}), // Entity_Linking
        State(2, {0.5, 0.6, 0.6, 0.2, 0.7, 0.3, 0.6, 0.7}), // Knowledge_Retrieval
        State(3, {0.9, 0.9, 0.1, 0.8, 0.1, 0.9, 0.2, 0.1}), // Hallucinated_Fact (BAD)
        State(4, {0.6, 0.7, 0.5, 0.3, 0.8, 0.4, 0.7, 0.6}), // Cross_Verification
        State(5, {0.8, 0.9, 0.4, 0.5, 0.9, 0.5, 0.8, 0.5})  // Verified_Decision (GOAL)
    };

    p.transitions = {
        Transition(1, 0, 1, 1.5, 0.95, 0.99, true),
        Transition(2, 1, 2, 2.0, 0.90, 0.98, true),
        Transition(3, 2, 3, 1.0, 0.20, 0.70, true), // Leads to bad state
        Transition(4, 3, 5, 1.0, 0.20, 0.70, true),
        Transition(5, 2, 4, 2.5, 0.99, 0.99, true), // Safe verification path
        Transition(6, 4, 5, 2.0, 0.98, 0.99, true)
    };

    return p;
}

} // namespace TestSuite

#endif // TEST_CASES_HPP
