#include "planner.hpp"

#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

using namespace safeplanner;

static State S(std::uint64_t id, double x, double y) {
    return State{id, {x, y}};
}

static Transition T(std::uint64_t id, std::uint64_t from, std::uint64_t to,
                    double cost, double reliability = 0.95,
                    bool available = true) {
    return Transition{id, from, to, cost, 1.0, reliability, available};
}

static PlanningProblem baseProblem() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 5;
    p.states = {
        S(0, 0, 0), S(1, 1, 0), S(2, 2, 0), S(3, 0, 2),
        S(4, 1, 2), S(5, 3, 2), S(6, 2, 1)
    };
    p.transitions = {
        T(0, 0, 1, 1.0),
        T(1, 1, 2, 1.0),
        T(2, 2, 5, 1.0),
        T(3, 0, 3, 1.2),
        T(4, 3, 4, 1.0),
        T(5, 4, 5, 1.0),
        T(6, 1, 6, 0.8),
        T(7, 6, 5, 1.0),
        T(8, 2, 6, 0.7)
    };
    return p;
}

static void printResult(const std::string& name, const PlanningResult& r) {
    std::cout << "\n[" << name << "]\n";
    std::cout << "success: " << std::boolalpha << r.success << "\n";
    std::cout << "state path: ";
    for (std::size_t i = 0; i < r.statePath.size(); ++i) {
        if (i) std::cout << " -> ";
        std::cout << r.statePath[i];
    }
    std::cout << "\n";
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "total cost: " << r.totalCost << "\n";
    std::cout << "minimum safety distance: " << r.safetyScore << "\n";
    std::cout << "cumulative reliability: " << r.cumulativeReliability << "\n";
    std::cout << "explored states: " << r.exploredStates << "\n";
    std::cout << "planning time (ms): " << r.planningTimeMs << "\n";
    if (r.replanningTimeMs > 0)
        std::cout << "replanning time (ms): " << r.replanningTimeMs << "\n";
}

int main() {
    std::cout << "Safe Semantic Planner - D* Lite demonstration\n";

    // 1. Basic reachability
    {
        auto p = baseProblem();
        p.transitions = {T(0,0,1,1), T(1,1,2,1), T(2,2,5,1)};
        DStarLitePlanner planner;
        printResult("Test 1: Basic Reachability", planner.plan(p));
    }

    // 2. Bad-state avoidance
    {
        auto p = baseProblem();
        p.badStates = {2};
        DStarLitePlanner planner;
        printResult("Test 2: Bad State Avoidance", planner.plan(p));
    }

    // 3. Safety margin: state 6 is closer to the bad state than the detour.
    {
        auto p = baseProblem();
        p.badStates = {6};
        DStarLitePlanner planner(2.5, 0.5);
        printResult("Test 3: Safety Margin", planner.plan(p));
    }

    // 4. Dynamic transition update
    {
        auto p = baseProblem();
        DStarLitePlanner planner;
        planner.plan(p);
        planner.setTransitionAvailable(2, false); // 2 -> 5
        printResult("Test 4: Dynamic Transition Removal",
                    planner.replan(0));
    }

    // 5. Goal update
    {
        auto p = baseProblem();
        DStarLitePlanner planner;
        planner.plan(p);
        planner.setGoal(4);
        printResult("Test 5: Goal Update", planner.replan(0));
    }

    // 6. Transition addition
    {
        auto p = baseProblem();
        p.transitions = {
            T(0,0,1,1), T(1,1,2,1), T(2,2,5,2),
            T(3,0,3,2), T(4,3,4,2), T(5,4,5,2)
        };
        DStarLitePlanner planner;
        planner.plan(p);
        planner.addTransition(T(99,0,5,0.5,0.99));
        printResult("Test 6: Transition Addition", planner.replan(0));
    }

    return 0;
}
