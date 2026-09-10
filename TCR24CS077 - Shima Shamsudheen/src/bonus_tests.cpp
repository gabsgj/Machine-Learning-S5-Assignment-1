#include "LPAStarPlanner.h"
#include <iostream>
#include <iomanip>
#include <string>

static void printResult(const std::string& label, const PlanningResult& r) {
    std::cout << "--- " << label << " ---\n";
    std::cout << "  success: " << (r.success ? "true" : "false") << "\n";
    if (r.success) {
        std::cout << "  path: ";
        for (size_t i = 0; i < r.statePath.size(); ++i)
            std::cout << r.statePath[i] << (i + 1 < r.statePath.size() ? " -> " : "");
        std::cout << "\n  totalCost: " << r.totalCost << "\n";
    }
    std::cout << std::endl;
}

static State mkState(uint64_t id, std::vector<double> emb) { return State{id, emb}; }
static Transition mkT(uint64_t id, uint64_t from, uint64_t to, double cost) {
    return Transition{id, from, to, cost, 1.0, 1.0, true};
}

// ===================== Bonus 1: Multi-goal planning =====================
// Start at 1. Three candidate goals (10, 11, 12) at different distances.
// Expect the planner to reach whichever is cheapest -- here, 11.
void bonusMultiGoal() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}),
                 mkState(10,{5,0}),   // far goal candidate, cost 5 hops away
                 mkState(11,{2,1}),   // near goal candidate, reachable from 3 in 1 hop
                 mkState(12,{9,9}) }; // unreachable goal candidate (no edges into it)
    p.transitions = {
        mkT(1,1,2,1.0), mkT(2,2,3,1.0), mkT(3,3,11,1.0),           // 1->2->3->11, cost 3
        mkT(4,3,4,1.0) // dummy filler, not used
    };
    // add a long, expensive route to candidate 10 as well, so 11 should still win
    p.states.push_back(mkState(4,{3,0}));
    p.states.push_back(mkState(5,{4,0}));
    p.transitions.push_back(mkT(5,3,4,1.0));
    p.transitions.push_back(mkT(6,4,5,1.0));
    p.transitions.push_back(mkT(7,5,10,1.0)); // 1->2->3->4->5->10, cost 5

    p.initialState = 1; p.goalState = 0 /*unused for multi-goal*/; p.badStates = {};

    LPAStarPlanner planner;
    auto r = planner.planMultiGoal(p, {10, 11, 12});
    printResult("Bonus 1: Multi-goal (candidates 10, 11, 12 -- expect 11, cost 3)", r);
}

// ===================== Bonus 2: Time-dependent availability =====================
// A direct bridge (2->3) is only open in [10, 20). Before/after, must detour.
void bonusTimeDependent() {
    PlanningProblem p;
    p.states = { mkState(1,{0,0}), mkState(2,{1,0}), mkState(3,{2,0}),
                 mkState(4,{1,-1}), mkState(5,{2,-1}) };
    p.transitions = {
        mkT(1,1,2,1.0),
        Transition{2, 2, 3, 1.0, 1.0, 1.0, true, 10.0, 20.0},   // bridge, open only during [10,20)
        mkT(3,1,4,2.0), mkT(4,4,5,2.0), mkT(5,5,3,2.0)          // always-open detour, costlier
    };
    p.initialState = 1; p.goalState = 3; p.badStates = {};

    LPAStarPlanner planner;
    planner.setCurrentTime(5.0); // before the bridge opens
    auto r1 = planner.plan(p);
    printResult("Bonus 2a: t=5 (bridge closed, expect detour via 4,5)", r1);

    planner.setCurrentTime(15.0); // bridge now open
    auto r2 = planner.replan();
    printResult("Bonus 2b: t=15 (bridge open, expect direct 1->2->3)", r2);

    planner.setCurrentTime(25.0); // bridge closed again
    auto r3 = planner.replan();
    printResult("Bonus 2c: t=25 (bridge closed again, expect detour)", r3);
}

// ===================== Bonus 3: Small knowledge graph =====================
// A toy ML-pipeline concept graph. Nodes are pipeline stages; one edge routes
// through a deprecated/unsafe stage that must be avoided (hard constraint),
// demonstrating the planner works on a non-geometric semantic graph, not just
// spatial grids. Embeddings here are arbitrary placeholders (unused for
// safety shaping since SAFETY_RADIUS is left at 0), which is the appropriate
// setting for a graph with no meaningful physical distance.
void bonusKnowledgeGraph() {
    // ids: 1 RawData, 2 Preprocessing, 3 FeatureExtraction, 4 DeprecatedEncoder(bad),
    //      5 ModernEncoder, 6 ModelTraining, 7 Evaluation, 8 Deployment
    PlanningProblem p;
    p.states = {
        mkState(1,{0}), mkState(2,{1}), mkState(3,{2}), mkState(4,{3}),
        mkState(5,{3}), mkState(6,{4}), mkState(7,{5}), mkState(8,{6}),
    };
    p.transitions = {
        mkT(1, 1,2, 1.0),               // RawData -> Preprocessing
        mkT(2, 2,3, 1.0),               // Preprocessing -> FeatureExtraction
        mkT(3, 3,4, 0.5),               // FeatureExtraction -> DeprecatedEncoder (cheap but bad)
        mkT(4, 4,6, 0.5),               // DeprecatedEncoder -> ModelTraining
        mkT(5, 3,5, 1.5),               // FeatureExtraction -> ModernEncoder (costlier, safe)
        mkT(6, 5,6, 1.0),               // ModernEncoder -> ModelTraining
        mkT(7, 6,7, 1.0),               // ModelTraining -> Evaluation
        mkT(8, 7,8, 1.0),               // Evaluation -> Deployment
    };
    p.initialState = 1; p.goalState = 8; p.badStates = {4}; // DeprecatedEncoder is unsafe/off-limits

    LPAStarPlanner planner;
    planner.SAFETY_RADIUS = 0.0; // no geometric shaping needed; hard avoidance still applies
    auto r = planner.plan(p);
    printResult("Bonus 3: Knowledge graph (ML pipeline, avoid DeprecatedEncoder=4)", r);
}

int main() {
    bonusMultiGoal();
    bonusTimeDependent();
    bonusKnowledgeGraph();
    return 0;
}
