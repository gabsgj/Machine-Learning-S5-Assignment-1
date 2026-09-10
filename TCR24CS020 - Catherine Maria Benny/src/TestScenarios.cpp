#include "TestScenarios.h"

namespace scenarios {

static Transition mk(uint64_t id, uint64_t from, uint64_t to, double cost,
                      double safety = 0.9, double reliability = 0.95, bool available = true) {
    return Transition(id, from, to, cost, safety, reliability, available);
}

PlanningProblem testCase1_BasicReachability() {
    PlanningProblem p;
    p.states = {
        State(0, {0.0, 0.0}),  // S
        State(1, {1.0, 0.0}),  // A
        State(2, {2.0, 0.0}),  // B
        State(3, {3.0, 0.0}),  // G
    };
    p.transitions = {
        mk(0, 0, 1, 1.0),
        mk(1, 1, 2, 1.0),
        mk(2, 2, 3, 1.0),
    };
    p.initialState = 0;
    p.goalState = 3;
    return p;
}

PlanningProblem testCase2_BadStateAvoidance() {
    PlanningProblem p;
    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {1.0, 1.0}),   // A
        State(2, {2.0, 1.0}),   // X (bad)
        State(3, {1.0, -1.0}),  // C
        State(4, {3.0, 0.0}),   // G
        State(5, {2.0, -1.0}),  // D
    };
    p.transitions = {
        // Costs are set >= the Euclidean distance between the two states'
        // Cartesian embeddings so that h(s,start)=beta*EuclideanDistance
        // stays an admissible lower bound (see DStarLite.cpp heuristic()).
        mk(0, 0, 1, 1.5),  // S(0,0)->A(1,1), dist=1.414 -> S -> A
        mk(1, 1, 2, 1.0),  // A(1,1)->X(2,1), dist=1.0   -> A -> X (leads into a bad state)
        mk(2, 2, 4, 1.5),  // X(2,1)->G(3,0), dist=1.414 -> X -> G
        mk(3, 0, 3, 1.5),  // S(0,0)->C(1,-1), dist=1.414-> S -> C
        mk(4, 3, 5, 1.0),  // C(1,-1)->D(2,-1), dist=1.0 -> C -> D
        mk(5, 5, 4, 1.5),  // D(2,-1)->G(3,0), dist=1.414-> D -> G
    };
    p.initialState = 0;
    p.goalState = 4;
    p.badStates = {2};
    return p;
}

PlanningProblem testCase3_SafetyMargin() {
    PlanningProblem p;
    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {2.0, 0.3}),   // Pnear (close to bad state)
        State(2, {2.0, 0.0}),   // Bad
        State(3, {4.0, 0.0}),   // G
        State(4, {2.0, 4.0}),   // Pfar (far from bad state)
    };
    p.transitions = {
        // S(0,0)->Pnear(2,0.3): dist=2.022 -> cost 2.1  (cheap, unsafe: total 4.2)
        // Pnear(2,0.3)->G(4,0): dist=2.022 -> cost 2.1
        // S(0,0)->Pfar(2,4):    dist=4.472 -> cost 4.5  (expensive, safe: total 9.0)
        // Pfar(2,4)->G(4,0):    dist=4.472 -> cost 4.5
        mk(0, 0, 1, 2.1),  // S -> Pnear
        mk(1, 1, 3, 2.1),  // Pnear -> G
        mk(2, 0, 4, 4.5),  // S -> Pfar
        mk(3, 4, 3, 4.5),  // Pfar -> G
    };
    p.initialState = 0;
    p.goalState = 3;
    p.badStates = {2};
    return p;
}

PlanningProblem testCase4_DynamicTransition() {
    PlanningProblem p;
    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {2.0, 0.0}),   // A
        State(2, {4.0, 0.0}),   // G
        State(3, {1.0, -2.0}),  // C
        State(4, {3.0, -2.0}),  // D
    };
    p.transitions = {
        // S(0,0)->A(2,0): dist=2.0     -> cost 2.0
        // A(2,0)->G(4,0): dist=2.0     -> cost 2.0   (initial shortest path, will be disabled)
        // S(0,0)->C(1,-2): dist=2.236  -> cost 2.3
        // C(1,-2)->D(3,-2): dist=2.0   -> cost 2.0
        // D(3,-2)->G(4,0): dist=2.236  -> cost 2.3   (alternate longer route)
        mk(0, 0, 1, 2.0),  // S -> A
        mk(1, 1, 2, 2.0),  // A -> G
        mk(2, 0, 3, 2.3),  // S -> C
        mk(3, 3, 4, 2.0),  // C -> D
        mk(4, 4, 2, 2.3),  // D -> G
    };
    p.initialState = 0;
    p.goalState = 2;
    return p;
}

PlanningProblem testCase5_GoalUpdate() {
    PlanningProblem p;
    p.states = {
        State(0, {0.0, 0.0}),   // S
        State(1, {2.0, 0.0}),   // A
        State(2, {4.0, 0.0}),   // G1
        State(3, {1.0, -3.0}),  // C
        State(4, {3.0, -3.0}),  // D
        State(5, {5.0, -3.0}),  // G2
    };
    p.transitions = {
        // S(0,0)->A(2,0): dist=2.0     -> cost 2.0
        // A(2,0)->G1(4,0): dist=2.0    -> cost 2.0
        // S(0,0)->C(1,-3): dist=3.162  -> cost 3.2
        // C(1,-3)->D(3,-3): dist=2.0   -> cost 2.0
        // D(3,-3)->G2(5,-3): dist=2.0  -> cost 2.0
        mk(0, 0, 1, 2.0),  // S -> A
        mk(1, 1, 2, 2.0),  // A -> G1
        mk(2, 0, 3, 3.2),  // S -> C
        mk(3, 3, 4, 2.0),  // C -> D
        mk(4, 4, 5, 2.0),  // D -> G2
    };
    p.initialState = 0;
    p.goalState = 2; // G1 initially
    return p;
}

PlanningProblem testCase6_TransitionAddition() {
    PlanningProblem p;
    // B is bent off the direct S->G line so that the initial route is a
    // genuine detour (total cost 4.0) longer than the straight-line
    // distance from S to G (3.0). This lets a later direct shortcut with
    // cost == distance (3.0, still admissible: cost >= distance) be a real
    // improvement, rather than requiring the shortcut to violate the
    // cost >= Euclidean-distance invariant the heuristic relies on.
    p.states = {
        State(0, {0.0, 0.0}),  // S
        State(1, {1.0, 0.0}),  // A
        State(2, {2.0, 1.0}),  // B (bent up off the S-G line)
        State(3, {3.0, 0.0}),  // G
    };
    p.transitions = {
        // S(0,0)->A(1,0): dist=1.0    -> cost 1.0
        // A(1,0)->B(2,1): dist=1.414  -> cost 1.5
        // B(2,1)->G(3,0): dist=1.414  -> cost 1.5   (only route initially, total cost 4.0)
        mk(0, 0, 1, 1.0),  // S -> A
        mk(1, 1, 2, 1.5),  // A -> B
        mk(2, 2, 3, 1.5),  // B -> G
    };
    p.initialState = 0;
    p.goalState = 3;
    return p;
}

} // namespace scenarios
