#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include "DStarLitePlanner.hpp"

namespace {

void printPath(const PlanningResult& r) {
    if (!r.success) { std::cout << "  path: <none>\n"; return; }
    std::cout << "  path: ";
    for (size_t i = 0; i < r.statePath.size(); ++i) {
        std::cout << r.statePath[i];
        if (i + 1 < r.statePath.size()) std::cout << " -> ";
    }
    std::cout << "\n";
}

void report(const std::string& label, const PlanningResult& r, std::ostringstream& csv) {
    std::cout << "== " << label << " ==\n";
    std::cout << "  success: " << (r.success ? "yes" : "no") << "\n";
    printPath(r);
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "  totalCost: " << r.totalCost
              << "  minDistToBad: " << r.safetyScore
              << "  reliabilitySum: " << r.cumulativeReliability
              << "  exploredStates: " << r.exploredStates
              << "  planningTimeMs: " << r.planningTimeMs << "\n\n";

    csv << label << "," << (r.success ? 1 : 0) << "," << r.totalCost << ","
        << r.safetyScore << "," << r.cumulativeReliability << ","
        << r.exploredStates << "," << r.planningTimeMs << "\n";
}

State s(uint64_t id, std::vector<double> emb) { return State(id, std::move(emb)); }
Transition t(uint64_t id, uint64_t from, uint64_t to, double cost, double safety, double reliability, bool avail = true) {
    return Transition(id, from, to, cost, safety, reliability, avail);
}

} // namespace

int main() {
    std::ostringstream csv;
    csv << "test_case,success,total_cost,min_dist_to_bad,reliability_sum,explored_states,planning_time_ms\n";

    // ---------------- Test Case 1: Basic Reachability ----------------
    // S(0) -> A(1) -> B(2) -> G(3), a single unique path.
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 3;
        p.states = { s(0,{0,0}), s(1,{1,0}), s(2,{2,0}), s(3,{3,0}) };
        p.transitions = { t(0,0,1,1,1,1), t(1,1,2,1,1,1), t(2,2,3,1,1,1) };

        DStarLitePlanner planner;
        auto r = planner.plan(p);
        report("TC1 Basic Reachability", r, csv);
    }

    // ---------------- Test Case 2: Bad State Avoidance ----------------
    // S(0) -> A(1) -> X(2, BAD) -> G(4)   [shorter but forbidden]
    // S(0) -> C(3) -> D(5) -> G(4)        [must be selected]
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 4;
        p.badStates = {2};
        p.states = { s(0,{0,0}), s(1,{1,1}), s(2,{2,1}), s(3,{1,-1}), s(4,{3,0}), s(5,{2,-1}) };
        p.transitions = {
            t(0,0,1,1,1,1), t(1,1,2,1,1,1), t(2,2,4,1,1,1), // via bad state X
            t(3,0,3,1,1,1), t(4,3,5,1,1,1), t(5,5,4,1,1,1)  // safe detour
        };

        DStarLitePlanner planner;
        auto r = planner.plan(p);
        report("TC2 Bad State Avoidance", r, csv);
    }

    // ---------------- Test Case 3: Safety Margin ----------------
    // Path 1: S->A1->A2->G, low cost, hugs a bad state.
    // Path 2: S->B1->B2->G, higher cost, stays far from the bad state.
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 5;
        p.badStates = {6};
        p.states = {
            s(0,{0,0}), s(1,{1,0.2}), s(2,{2,0.2}), s(5,{3,0}),
            s(3,{1,3}), s(4,{2,3}), s(6,{1.5,0.2}) // bad state sits right between A1/A2
        };
        p.transitions = {
            t(0,0,1,1,1,0.9), t(1,1,2,1,1,0.9), t(2,2,5,1,1,0.9),   // cheap, close to bad state
            t(3,0,3,2,1,0.95), t(4,3,4,2,1,0.95), t(5,4,5,2,1,0.95) // costlier, far from bad state
        };

        DStarLitePlanner planner; // default weights favour safety margin
        auto r = planner.plan(p);
        report("TC3 Safety Margin", r, csv);
        std::cout << "  (planner trades cost for safety via weights.proximity;"
                     " see docs/DesignReport.md Sec. 4)\n\n";
    }

    // ---------------- Test Case 4: Dynamic Transition (edge removed) ----------------
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 2;
        p.states = { s(0,{0,0}), s(1,{1,0}), s(2,{2,0}) };
        p.transitions = { t(10,0,1,1,1,1), t(11,1,2,1,1,1), t(12,0,2,5,0.6,0.6) }; // direct edge is a costly fallback

        DStarLitePlanner planner;
        planner.initialize(p);
        auto r1 = planner.replan();
        report("TC4a before edge removal", r1, csv);

        planner.setEdgeAvailability(11, false); // (1,2) becomes unavailable
        auto r2 = planner.replan();
        report("TC4b after (A,G) removed", r2, csv);
    }

    // ---------------- Test Case 5: Goal Update ----------------
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 2;
        p.states = { s(0,{0,0}), s(1,{1,0}), s(2,{2,0}), s(3,{0,2}) };
        p.transitions = { t(20,0,1,1,1,1), t(21,1,2,1,1,1), t(22,0,3,1,1,1) };

        DStarLitePlanner planner;
        planner.initialize(p);
        auto r1 = planner.replan();
        report("TC5a original goal", r1, csv);

        planner.updateGoal(3); // goal changes mid-execution
        auto r2 = planner.replan();
        report("TC5b after goal change", r2, csv);
    }

    // ---------------- Test Case 6: Transition Addition (shortcut) ----------------
    {
        PlanningProblem p;
        p.initialState = 0; p.goalState = 3;
        p.states = { s(0,{0,0}), s(1,{1,0}), s(2,{2,0}), s(3,{3,0}) };
        p.transitions = { t(30,0,1,1,1,1), t(31,1,2,1,1,1), t(32,2,3,1,1,1) };

        DStarLitePlanner planner;
        planner.initialize(p);
        auto r1 = planner.replan();
        report("TC6a before shortcut", r1, csv);

        planner.addTransition(Transition(33, 0, 3, 1.0, 1.0, 1.0)); // new direct shortcut
        auto r2 = planner.replan();
        report("TC6b after shortcut added", r2, csv);
    }

    std::ofstream out("results.csv");
    out << csv.str();
    std::cout << "Wrote results.csv with " << 10 << " rows.\n";
    return 0;
}
