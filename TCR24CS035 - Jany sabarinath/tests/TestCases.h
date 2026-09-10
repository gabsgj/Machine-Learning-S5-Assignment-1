#ifndef TEST_CASES_H
#define TEST_CASES_H

#include "../src/PlanningProblem.h"

#include <iostream>
#include <string>

namespace TestCases {

PlanningProblem basicReachability() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {
        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {3, 0}}
    };

    problem.transitions = {
        {0, 0, 1, 2.0, 0.9, 0.95, true},
        {1, 1, 2, 2.0, 0.9, 0.95, true},
        {2, 2, 3, 2.0, 0.9, 0.95, true}
    };

    return problem;
}

PlanningProblem badStateAvoidance() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 5;

    problem.states = {
        {0, {0, 0}},   // S
        {1, {1, 1}},   // A
        {2, {2, 2}},   // X BAD
        {3, {1, -1}},  // C
        {4, {2, -1}},  // D
        {5, {3, 0}}    // G
    };

    problem.badStates = {
        2
    };

    problem.transitions = {

        // Bad path
        {0, 0, 1, 1.0, 0.8, 0.9, true},
        {1, 1, 2, 1.0, 0.8, 0.9, true},
        {2, 2, 5, 1.0, 0.8, 0.9, true},

        // Safe path
        {3, 0, 3, 2.0, 0.9, 0.95, true},
        {4, 3, 4, 2.0, 0.9, 0.95, true},
        {5, 4, 5, 2.0, 0.9, 0.95, true}
    };

    return problem;
}

PlanningProblem safetyMargin() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 6;

    problem.states = {

        {0, {0, 0}},      // S

        {1, {1, 0}},      // A
        {2, {2, 0}},      // B

        {3, {1, 4}},      // C
        {4, {2, 4}},      // D

        {5, {3, 2}},      // intermediate
        {6, {4, 2}}       // G
    };

    /*
       Bad state near the first route.
    */

    problem.badStates = {
        7
    };

    problem.states.push_back(
        {7, {2, 1}}
    );

    problem.transitions = {

        // Cheap but unsafe route
        {0, 0, 1, 2.0, 0.8, 0.95, true},
        {1, 1, 2, 2.0, 0.8, 0.95, true},
        {2, 2, 5, 2.0, 0.8, 0.95, true},
        {3, 5, 6, 2.0, 0.8, 0.95, true},

        // Expensive but safer route
        {4, 0, 3, 4.0, 0.95, 0.98, true},
        {5, 3, 4, 4.0, 0.95, 0.98, true},
        {6, 4, 6, 4.0, 0.95, 0.98, true}
    };

    return problem;
}

PlanningProblem dynamicTransition() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        {0, {0, 0}}, // S
        {1, {1, 0}}, // A
        {2, {1, 1}}, // B
        {3, {2, 1}}  // G
    };

    problem.transitions = {

        // Initial best route
        {0, 0, 1, 1.0, 0.9, 0.95, true},
        {1, 1, 3, 1.0, 0.9, 0.95, true},

        // Alternative
        {2, 0, 2, 3.0, 0.9, 0.95, true},
        {3, 2, 3, 3.0, 0.9, 0.95, true}
    };

    return problem;
}

PlanningProblem goalUpdate() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        {0, {0, 0}},
        {1, {1, 0}},
        {2, {1, 1}},
        {3, {2, 0}},
        {4, {2, 2}}
    };

    problem.transitions = {

        {0, 0, 1, 1.0, 0.9, 0.95, true},
        {1, 1, 3, 1.0, 0.9, 0.95, true},

        {2, 0, 2, 2.0, 0.9, 0.95, true},
        {3, 2, 4, 2.0, 0.9, 0.95, true}
    };

    return problem;
}

PlanningProblem transitionAddition() {

    PlanningProblem problem;

    problem.initialState = 0;
    problem.goalState = 3;

    problem.states = {

        {0, {0, 0}},
        {1, {1, 0}},
        {2, {2, 0}},
        {3, {3, 0}}
    };

    problem.transitions = {

        {0, 0, 1, 3.0, 0.9, 0.95, true},
        {1, 1, 2, 3.0, 0.9, 0.95, true},
        {2, 2, 3, 3.0, 0.9, 0.95, true}
    };

    return problem;
}

}

#endif