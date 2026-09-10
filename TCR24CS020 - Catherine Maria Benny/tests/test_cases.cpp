// Unit tests for the Safe Semantic Planner (assignment section 31).
// Minimal, dependency-free assert-based test runner -- no external test
// framework required, matching the "do not overengineer" guidance (section 41).
//
// Covers: basic reachability, unreachable goal, bad initial state, bad goal,
// bad-state avoidance, unavailable transition, transition addition,
// transition removal, goal update, safety-distance calculation, path
// validation, and dynamic replanning.
#include <cmath>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

#include "DStarLite.h"
#include "Metrics.h"
#include "TestScenarios.h"

namespace {
int g_passed = 0;
int g_failed = 0;

void check(bool cond, const std::string& testName, const std::string& detail = "") {
    if (cond) {
        ++g_passed;
        std::cout << "[PASS] " << testName << "\n";
    } else {
        ++g_failed;
        std::cout << "[FAIL] " << testName;
        if (!detail.empty()) std::cout << " -- " << detail;
        std::cout << "\n";
    }
}
} // namespace

static void test_BasicReachability() {
    DStarLite planner;
    auto r = planner.plan(scenarios::testCase1_BasicReachability());
    check(r.success, "BasicReachability: success");
    check(r.statePath == std::vector<uint64_t>({0, 1, 2, 3}), "BasicReachability: correct path");
    check(r.totalCost == 3.0, "BasicReachability: correct cost");
}

static void test_UnreachableGoal() {
    PlanningProblem p;
    p.states = {State(0, {0, 0}), State(1, {5, 5})};
    // No transitions at all -> goal unreachable.
    p.initialState = 0;
    p.goalState = 1;
    DStarLite planner;
    auto r = planner.plan(p);
    check(!r.success, "UnreachableGoal: fails cleanly", r.success ? "returned success" : "");
    check(!r.errorMessage.empty(), "UnreachableGoal: has an error message");
}

static void test_BadInitialState() {
    PlanningProblem p;
    p.states = {State(0, {0, 0}), State(1, {1, 0})};
    p.transitions = {Transition(0, 0, 1, 1.0, 0.9, 0.9, true)};
    p.initialState = 0;
    p.goalState = 1;
    p.badStates = {0}; // initial state itself is bad
    DStarLite planner;
    auto r = planner.plan(p);
    check(!r.success, "BadInitialState: planning fails");
    check(r.errorMessage.find("initial") != std::string::npos,
          "BadInitialState: error mentions initial state", r.errorMessage);
}

static void test_BadGoalState() {
    PlanningProblem p;
    p.states = {State(0, {0, 0}), State(1, {1, 0})};
    p.transitions = {Transition(0, 0, 1, 1.0, 0.9, 0.9, true)};
    p.initialState = 0;
    p.goalState = 1;
    p.badStates = {1}; // goal itself is bad
    DStarLite planner;
    auto r = planner.plan(p);
    check(!r.success, "BadGoalState: planning fails");
    check(r.errorMessage.find("goal") != std::string::npos,
          "BadGoalState: error mentions goal state", r.errorMessage);
}

static void test_BadStateAvoidance() {
    DStarLite planner;
    auto r = planner.plan(scenarios::testCase2_BadStateAvoidance());
    check(r.success, "BadStateAvoidance: success");
    check(r.badStatesVisited == 0, "BadStateAvoidance: zero bad states visited");
    bool visitsX = false;
    for (auto s : r.statePath) if (s == 2) visitsX = true;
    check(!visitsX, "BadStateAvoidance: bad state X never appears in path");
}

static void test_UnavailableTransition() {
    auto problem = scenarios::testCase4_DynamicTransition();
    DStarLite planner;
    auto initial = planner.plan(problem);
    check(initial.success, "UnavailableTransition: initial plan succeeds");
    auto replanned = planner.setTransitionAvailability(1, false);
    check(replanned.success, "UnavailableTransition: replan after disabling edge succeeds");
    bool usesDisabledEdge = false;
    for (auto tid : replanned.transitionPath) if (tid == 1) usesDisabledEdge = true;
    check(!usesDisabledEdge, "UnavailableTransition: new path avoids the disabled transition");
}

static void test_TransitionAddition() {
    auto problem = scenarios::testCase6_TransitionAddition();
    DStarLite planner;
    auto initial = planner.plan(problem);
    check(initial.success && initial.totalCost == 4.0,
          "TransitionAddition: initial detour cost is 4.0");
    Transition shortcut(10, 0, 3, 3.0, 0.9, 0.9, true);
    auto replanned = planner.addTransition(shortcut);
    check(replanned.success, "TransitionAddition: replan succeeds");
    check(replanned.totalCost == 3.0, "TransitionAddition: shortcut is adopted (cost 3.0)");
}

static void test_TransitionRemoval() {
    auto problem = scenarios::testCase4_DynamicTransition();
    DStarLite planner;
    planner.plan(problem);
    auto afterRemoval = planner.removeTransition(1); // A->G removed entirely
    check(afterRemoval.success, "TransitionRemoval: alternate route still found");
    check(afterRemoval.totalCost > 4.0, "TransitionRemoval: alternate route costs more than the original");
}

static void test_GoalUpdate() {
    auto problem = scenarios::testCase5_GoalUpdate();
    DStarLite planner;
    auto initial = planner.plan(problem);
    check(initial.success && initial.statePath.back() == 2, "GoalUpdate: initial plan reaches G1");
    auto replanned = planner.updateGoal(5);
    check(replanned.success && replanned.statePath.back() == 5, "GoalUpdate: replanned path reaches G2");
}

static void test_SafetyDistanceCalculation() {
    std::unordered_map<uint64_t, State> states;
    states[0] = State(0, {0.0, 0.0});
    states[1] = State(1, {3.0, 4.0}); // distance 5 from origin
    states[2] = State(2, {10.0, 0.0});
    std::vector<uint64_t> path = {0, 1, 2};
    std::vector<uint64_t> bad = {2};
    double d = metrics::minimumSafetyDistance(path, bad, states);
    // min( dist(0,2)=10, dist(1,2)=sqrt(49+16)=8.062, dist(2,2)=0 ) = 0
    check(std::abs(d - 0.0) < 1e-9, "SafetyDistanceCalculation: min distance is 0 when path visits the bad state",
          std::to_string(d));

    std::vector<uint64_t> path2 = {0, 1};
    double d2 = metrics::minimumSafetyDistance(path2, bad, states);
    double expected = std::min(metrics::euclideanDistance(states[0].embedding, states[2].embedding),
                                metrics::euclideanDistance(states[1].embedding, states[2].embedding));
    check(std::abs(d2 - expected) < 1e-9, "SafetyDistanceCalculation: correct min distance over a path",
          std::to_string(d2) + " vs " + std::to_string(expected));

    double dNoBad = metrics::minimumSafetyDistance(path2, {}, states);
    check(std::isinf(dNoBad), "SafetyDistanceCalculation: no bad states => +infinity (documented convention)");
}

static void test_PathValidation() {
    auto problem = scenarios::testCase1_BasicReachability();
    std::string err = metrics::validatePath(problem, {0, 1, 2, 3}, {0, 1, 2});
    check(err.empty(), "PathValidation: a correct path validates cleanly", err);

    std::string err2 = metrics::validatePath(problem, {1, 2, 3}, {1, 2});
    check(!err2.empty(), "PathValidation: rejects a path not starting at the initial state");

    std::string err3 = metrics::validatePath(problem, {0, 1, 2}, {0, 1});
    check(!err3.empty(), "PathValidation: rejects a path not ending at the goal");

    auto problem2 = scenarios::testCase2_BadStateAvoidance();
    std::string err4 = metrics::validatePath(problem2, {0, 1, 2, 4}, {0, 1, 2});
    check(!err4.empty(), "PathValidation: rejects a path that visits a bad state", err4);
}

static void test_DynamicReplanningReusesSearch() {
    auto problem = scenarios::testCase4_DynamicTransition();
    DStarLite planner;
    auto initial = planner.plan(problem);
    auto replanned = planner.setTransitionAvailability(1, false);
    // A correct incremental implementation only needs to re-expand the
    // handful of vertices in the affected region, not the whole graph again.
    check(replanned.exploredStates <= static_cast<int>(problem.states.size()),
          "DynamicReplanning: replanning explores a bounded number of vertices",
          std::to_string(replanned.exploredStates));
    check(replanned.success, "DynamicReplanning: still finds a valid alternate path");
}

int main() {
    std::cout << "Running Safe Semantic Planner unit tests...\n\n";
    test_BasicReachability();
    test_UnreachableGoal();
    test_BadInitialState();
    test_BadGoalState();
    test_BadStateAvoidance();
    test_UnavailableTransition();
    test_TransitionAddition();
    test_TransitionRemoval();
    test_GoalUpdate();
    test_SafetyDistanceCalculation();
    test_PathValidation();
    test_DynamicReplanningReusesSearch();

    std::cout << "\n" << g_passed << " passed, " << g_failed << " failed.\n";
    return g_failed == 0 ? 0 : 1;
}
