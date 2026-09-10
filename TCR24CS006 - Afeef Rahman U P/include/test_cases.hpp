#pragma once

#include "types.hpp"
#include "lpa_star.hpp"
#include "d_star_lite.hpp"
#include "multi_objective.hpp"

#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <iomanip>

namespace SafePlanner {

/**
 * @brief Illustrative Test Cases from Assignment Specification + Comprehensive Edge Cases.
 */
class TestCases {
public:
    /**
     * @brief Test Case 1: Basic Reachability
     * Graph: S -> A -> B -> G
     * Expected result: planner returns unique valid path [S, A, B, G].
     */
    static PlanningProblem createTestCase1() {
        PlanningProblem p;
        // States: S(1), A(2), B(3), G(4) in R^2
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 0.0}, "A"),
            State(3, {2.0, 0.0}, "B"),
            State(4, {3.0, 0.0}, "G")
        };
        p.initialState = 1;
        p.goalState = 4;
        p.badStates = {}; // No bad states

        p.transitions = {
            Transition(1, 1, 2, 1.0, 1.0, 1.0, true), // S -> A
            Transition(2, 2, 3, 1.0, 1.0, 1.0, true), // A -> B
            Transition(3, 3, 4, 1.0, 1.0, 1.0, true)  // B -> G
        };
        return p;
    }

    /**
     * @brief Test Case 2: Bad State Avoidance
     * Paths: S -> A -> X -> G (X is bad state) and S -> C -> D -> G
     * Expected result: second path [S, C, D, G] must be selected.
     */
    static PlanningProblem createTestCase2() {
        PlanningProblem p;
        // States: S(1), A(2), X(3-bad), G(4), C(5), D(6)
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 1.0}, "A"),
            State(3, {2.0, 1.0}, "X (BAD)"),
            State(4, {3.0, 0.0}, "G"),
            State(5, {1.0, -1.0}, "C"),
            State(6, {2.0, -1.0}, "D")
        };
        p.initialState = 1;
        p.goalState = 4;
        p.badStates = {3}; // X is bad

        p.transitions = {
            // Path 1 (through bad state X, lower nominal cost = 3.0)
            Transition(1, 1, 2, 1.0, 0.2, 0.9, true), // S -> A
            Transition(2, 2, 3, 1.0, 0.0, 0.9, true), // A -> X
            Transition(3, 3, 4, 1.0, 0.0, 0.9, true), // X -> G

            // Path 2 (safe bypass, nominal cost = 4.0)
            Transition(4, 1, 5, 1.3, 1.0, 0.99, true), // S -> C
            Transition(5, 5, 6, 1.4, 1.0, 0.99, true), // C -> D
            Transition(6, 6, 4, 1.3, 1.0, 0.99, true)  // D -> G
        };
        return p;
    }

    /**
     * @brief Test Case 3: Safety Margin
     * Path 1 (Close to bad state): S -> A -> G (Cost 2.0, passes adjacent to Bad State B)
     * Path 2 (Far from bad state): S -> C -> D -> G (Cost 3.5, large safety clearance)
     * Demonstrates both pure cost minimization and safety-weighted objective balancing.
     */
    static PlanningProblem createTestCase3() {
        PlanningProblem p;
        // Bad state placed at (1.0, 0.5)
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 0.2}, "A (Close to Bad)"),
            State(3, {1.0, 0.8}, "B_BAD (Obstacle)"),
            State(4, {2.0, 0.0}, "G"),
            State(5, {0.5, -2.0}, "C (Wide Safe Arc)"),
            State(6, {1.5, -2.0}, "D (Wide Safe Arc)")
        };
        p.initialState = 1;
        p.goalState = 4;
        p.badStates = {3};

        p.transitions = {
            // Path 1: low cost (2.0), low safety clearance (dist = 0.6)
            Transition(1, 1, 2, 1.0, 0.5, 0.9, true), // S -> A
            Transition(2, 2, 4, 1.0, 0.5, 0.9, true), // A -> G

            // Path 2: higher cost (3.5), high safety clearance (dist >= 2.8)
            Transition(3, 1, 5, 1.2, 1.0, 0.99, true), // S -> C
            Transition(4, 5, 6, 1.1, 1.0, 0.99, true), // C -> D
            Transition(5, 6, 4, 1.2, 1.0, 0.99, true)  // D -> G
        };
        return p;
    }

    /**
     * @brief Test Case 4: Dynamic Transition
     * Initially: S -> A -> G (cost 2.0)
     * Later: Transition (A, G) becomes unavailable -> Incremental replan finds S -> B -> C -> G
     */
    static PlanningProblem createTestCase4() {
        PlanningProblem p;
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 1.0}, "A"),
            State(3, {2.0, 0.0}, "G"),
            State(4, {0.8, -1.0}, "B"),
            State(5, {1.4, -1.0}, "C")
        };
        p.initialState = 1;
        p.goalState = 3;
        p.badStates = {};

        p.transitions = {
            // Primary path (initially available)
            Transition(1, 1, 2, 1.0, 1.0, 1.0, true), // S -> A
            Transition(2, 2, 3, 1.0, 1.0, 1.0, true), // A -> G (will fail)

            // Backup path
            Transition(3, 1, 4, 1.2, 1.0, 1.0, true), // S -> B
            Transition(4, 4, 5, 1.0, 1.0, 1.0, true), // B -> C
            Transition(5, 5, 3, 1.2, 1.0, 1.0, true)  // C -> G
        };
        return p;
    }

    /**
     * @brief Test Case 5: Goal Update
     * Initially Goal = G1. Later Goal changes to G2.
     * Planner produces revised path without rebuilding all data structures.
     */
    static PlanningProblem createTestCase5() {
        PlanningProblem p;
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 0.5}, "Hub_1"),
            State(3, {2.0, 1.0}, "G1 (Old Goal)"),
            State(4, {1.0, -0.5}, "Hub_2"),
            State(5, {2.5, -1.0}, "G2 (New Goal)")
        };
        p.initialState = 1;
        p.goalState = 3; // Initially G1
        p.badStates = {};

        p.transitions = {
            Transition(1, 1, 2, 1.1, 1.0, 1.0, true), // S -> Hub_1
            Transition(2, 2, 3, 1.1, 1.0, 1.0, true), // Hub_1 -> G1
            Transition(3, 1, 4, 1.1, 1.0, 1.0, true), // S -> Hub_2
            Transition(4, 4, 5, 1.6, 1.0, 1.0, true), // Hub_2 -> G2
            Transition(5, 2, 4, 1.0, 1.0, 1.0, true)  // Hub_1 -> Hub_2
        };
        return p;
    }

    /**
     * @brief Test Case 6: Transition Addition
     * Initial path: S -> A -> B -> G (Cost 3.0)
     * New shortcut transition (A -> G, Cost 1.1) is added dynamically.
     * Planner discovers improved solution S -> A -> G with minimal vertex updates.
     */
    static PlanningProblem createTestCase6() {
        PlanningProblem p;
        p.states = {
            State(1, {0.0, 0.0}, "S"),
            State(2, {1.0, 0.0}, "A"),
            State(3, {2.0, 0.0}, "B"),
            State(4, {3.0, 0.0}, "G")
        };
        p.initialState = 1;
        p.goalState = 4;
        p.badStates = {};

        p.transitions = {
            Transition(1, 1, 2, 1.0, 1.0, 1.0, true), // S -> A
            Transition(2, 2, 3, 1.0, 1.0, 1.0, true), // A -> B
            Transition(3, 3, 4, 1.0, 1.0, 1.0, true)  // B -> G
        };
        return p;
    }

    /**
     * @brief Bonus: High-dimensional Semantic Knowledge Graph Navigation
     * States embedded in R^64 semantic vector space.
     */
    static PlanningProblem createKnowledgeGraphSemanticCase() {
        PlanningProblem p;
        // 8 semantic concepts in R^4 for easy inspection
        p.states = {
            State(1, {0.9, 0.1, 0.0, 0.2}, "Concept_Query"),
            State(2, {0.7, 0.3, 0.1, 0.2}, "Entity_Resolver"),
            State(3, {0.1, 0.8, 0.9, 0.4}, "Malicious_Entity (BAD)"),
            State(4, {0.6, 0.2, 0.2, 0.7}, "Knowledge_Fusion"),
            State(5, {0.5, 0.1, 0.3, 0.8}, "Verified_Fact"),
            State(6, {0.2, 0.1, 0.4, 0.9}, "Goal_Insight")
        };
        p.initialState = 1;
        p.goalState = 6;
        p.badStates = {3};

        p.transitions = {
            Transition(1, 1, 2, 1.2, 0.9, 0.98, true),
            Transition(2, 2, 3, 0.8, 0.1, 0.50, true), // Unsafe transition
            Transition(3, 3, 6, 0.9, 0.1, 0.50, true),
            Transition(4, 2, 4, 1.4, 0.95, 0.99, true),
            Transition(5, 4, 5, 1.1, 0.99, 0.99, true),
            Transition(6, 5, 6, 1.0, 0.99, 0.99, true)
        };
        return p;
    }
};

} // namespace SafePlanner
