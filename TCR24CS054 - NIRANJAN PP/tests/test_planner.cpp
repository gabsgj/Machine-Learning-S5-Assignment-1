#include "planner.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace safeplanner;

static State s(std::uint64_t id, double x, double y) {
    return State{id, {x, y}};
}

static Transition t(std::uint64_t id, std::uint64_t a, std::uint64_t b,
                    double cost) {
    return Transition{id, a, b, cost, 1.0, 1.0, true};
}

static PlanningProblem simple() {
    PlanningProblem p;
    p.initialState = 0;
    p.goalState = 3;
    p.states = {s(0,0,0), s(1,1,0), s(2,2,0), s(3,3,0),
                s(4,0,1), s(5,1,1), s(6,2,1)};
    p.transitions = {
        t(0,0,1,1), t(1,1,2,1), t(2,2,3,1),
        t(3,0,4,1), t(4,4,5,1), t(5,5,6,1), t(6,6,3,1)
    };
    return p;
}

int main() {
    {
        DStarLitePlanner planner;
        auto r = planner.plan(simple());
        assert(r.success);
        assert(r.statePath.front() == 0);
        assert(r.statePath.back() == 3);
    }

    {
        auto p = simple();
        p.badStates = {1};
        DStarLitePlanner planner;
        auto r = planner.plan(p);
        assert(r.success);
        for (auto id : r.statePath) assert(id != 1);
    }

    {
        auto p = simple();
        DStarLitePlanner planner;
        auto r1 = planner.plan(p);
        assert(r1.success);

        planner.setTransitionAvailable(2, false);
        auto r2 = planner.replan(0);
        assert(r2.success);
        for (auto id : r2.transitionPath) assert(id != 2);
    }

    {
        auto p = simple();
        DStarLitePlanner planner;
        planner.plan(p);
        planner.setGoal(6);
        auto r = planner.replan(0);
        assert(r.success);
        assert(r.statePath.back() == 6);
    }

    {
        auto p = simple();
        p.transitions = {t(0,0,1,10), t(1,1,3,10)};
        DStarLitePlanner planner;
        auto r1 = planner.plan(p);
        assert(r1.success);

        planner.addTransition(t(99,0,3,1));
        auto r2 = planner.replan(0);
        assert(r2.success);
        assert(r2.statePath.size() == 2);
        assert(std::fabs(r2.totalCost - 1.0) < 1e-6);
    }

    std::cout << "All tests passed.\n";
    return 0;
}
