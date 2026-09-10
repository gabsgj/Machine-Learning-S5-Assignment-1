// Experimental evaluation for the design report.
// Builds a random grid-graph, plans across it, then measures the speedup
// LPA*'s incremental replanning gives over cold (from-scratch) replanning
// after single-edge availability changes -- this is the core claim the
// "Dynamic Environment" section of the assignment asks us to substantiate.
#include "LPAStarPlanner.h"
#include <iostream>
#include <iomanip>
#include <random>
#include <chrono>
#include <cmath>

#if defined(__unix__) || defined(__APPLE__)
  #define SP_HAVE_GETRUSAGE 1
  #include <sys/resource.h>
#else
  #define SP_HAVE_GETRUSAGE 0
#endif

static PlanningProblem buildGrid(int side, double badFraction, unsigned seed) {
    PlanningProblem p;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> costDist(0.8, 1.5);
    std::uniform_real_distribution<double> reliDist(0.9, 1.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    auto idOf = [side](int r, int c) { return (uint64_t)(r * side + c + 1); };

    for (int r = 0; r < side; ++r)
        for (int c = 0; c < side; ++c)
            p.states.push_back(State{ idOf(r, c), { (double)r, (double)c } });

    uint64_t tid = 1;
    for (int r = 0; r < side; ++r) {
        for (int c = 0; c < side; ++c) {
            if (c + 1 < side) {
                double cost = costDist(rng);
                p.transitions.push_back(Transition{ tid++, idOf(r,c), idOf(r,c+1), cost, 1.0, reliDist(rng), true });
                p.transitions.push_back(Transition{ tid++, idOf(r,c+1), idOf(r,c), cost, 1.0, reliDist(rng), true });
            }
            if (r + 1 < side) {
                double cost = costDist(rng);
                p.transitions.push_back(Transition{ tid++, idOf(r,c), idOf(r+1,c), cost, 1.0, reliDist(rng), true });
                p.transitions.push_back(Transition{ tid++, idOf(r+1,c), idOf(r,c), cost, 1.0, reliDist(rng), true });
            }
        }
    }

    p.initialState = idOf(0, 0);
    p.goalState = idOf(side - 1, side - 1);

    for (auto& s : p.states) {
        if (s.id == p.initialState || s.id == p.goalState) continue;
        if (unit(rng) < badFraction) p.badStates.push_back(s.id);
    }
    return p;
}

static double nowMs(std::chrono::high_resolution_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - t0).count();
}

int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "side,numStates,numTransitions,coldPlanMs,coldExplored,"
                 "avgIncReplanMs,avgIncExplored,avgColdReplanMs,speedupFactor,peakRssKB\n";

    for (int side : {10, 20, 30, 40, 50}) {
        PlanningProblem p = buildGrid(side, 0.08, /*seed=*/side * 17 + 3);

        LPAStarPlanner planner;
        auto t0 = std::chrono::high_resolution_clock::now();
        auto cold = planner.plan(p);
        double coldMs = nowMs(t0);

        // Incremental replanning: toggle 15 random transitions off/on one at a
        // time, replanning after each, and average the time / explored count.
        std::mt19937 rng(12345);
        std::uniform_int_distribution<size_t> pick(0, p.transitions.size() - 1);
        const int trials = 15;
        double incTotalMs = 0.0; int incTotalExplored = 0;
        double coldReplanTotalMs = 0.0;

        for (int i = 0; i < trials; ++i) {
            uint64_t tid = p.transitions[pick(rng)].id;

            auto ti0 = std::chrono::high_resolution_clock::now();
            planner.setTransitionAvailability(tid, false);
            auto r1 = planner.replan();
            incTotalMs += nowMs(ti0);
            incTotalExplored += r1.statesExplored;

            // For comparison only: what a full cold re-plan would have cost
            // at this point (fresh planner instance, same modified problem).
            PlanningProblem modified = p;
            for (auto& t : modified.transitions) if (t.id == tid) t.available = false;
            LPAStarPlanner coldPlanner;
            auto tc0 = std::chrono::high_resolution_clock::now();
            coldPlanner.plan(modified);
            coldReplanTotalMs += nowMs(tc0);

            planner.setTransitionAvailability(tid, true); // restore for next trial
            planner.replan();
        }

        long peakRssKB = 0;
#if SP_HAVE_GETRUSAGE
        struct rusage ru; getrusage(RUSAGE_SELF, &ru);
        peakRssKB = ru.ru_maxrss; // KB on Linux, bytes on macOS -- fine for a relative comparison
#endif

        double avgIncMs = incTotalMs / trials;
        double avgIncExplored = (double)incTotalExplored / trials;
        double avgColdReplanMs = coldReplanTotalMs / trials;
        double speedup = avgIncMs > 0 ? avgColdReplanMs / avgIncMs : 0.0;

        std::cout << side << "," << p.states.size() << "," << p.transitions.size() << ","
                  << coldMs << "," << cold.statesExplored << ","
                  << avgIncMs << "," << avgIncExplored << ","
                  << avgColdReplanMs << "," << speedup << ","
                  << peakRssKB << "\n";
    }
    return 0;
}
