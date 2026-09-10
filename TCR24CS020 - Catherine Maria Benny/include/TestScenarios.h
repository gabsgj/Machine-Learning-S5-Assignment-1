#pragma once
#include "PlanningProblem.h"

// Builders for the six illustrative test cases from the assignment PDF.
// State IDs are documented next to each function; embeddings are simple 2D
// Cartesian coordinates chosen so the plotted layout matches the assignment's
// informal ASCII diagrams as closely as possible.
namespace scenarios {

// S(0) -> A(1) -> B(2) -> G(3). Single unique path.
PlanningProblem testCase1_BasicReachability();

// S(0) -> A(1) -> X(2,bad) -> G(4)   [must be rejected]
// S(0) -> C(3) -> D(5) -> G(4)       [must be selected]
PlanningProblem testCase2_BadStateAvoidance();

// Two valid paths from S to G: a cheap path close to a bad state, and a
// pricier path that stays much farther away. Returns the problem; callers
// vary ObjectiveWeights::gamma (safety weight) to observe the tradeoff.
PlanningProblem testCase3_SafetyMargin();

// S(0) -> A(1) -> G(2) directly, plus a longer alternate S->C->D->G route.
// Caller disables transition (A->G) after the first plan() to trigger
// dynamic replanning.
PlanningProblem testCase4_DynamicTransition();

// S(0) -> A(1) -> G1(2), and a disjoint route S(0) -> C(3) -> D(4) -> G2(5).
// Caller plans to G1 first, then calls updateGoal(G2).
PlanningProblem testCase5_GoalUpdate();

// S(0) -> A(1) -> B(2) -> G(3), a longer valid route, with NO direct S->G
// shortcut initially. Caller adds a shortcut transition afterward and
// expects the planner to use it once it improves the objective.
PlanningProblem testCase6_TransitionAddition();

} // namespace scenarios
