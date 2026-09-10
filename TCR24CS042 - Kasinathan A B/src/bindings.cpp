#include <emscripten/bind.h>
#include "../include/state.h"
#include "../include/transition.h"
#include "../include/planning_problem.h"
#include "../include/planning_result.h"
#include "../include/lpa_star.h"

using namespace emscripten;

EMSCRIPTEN_BINDINGS(planner_module) {

    // Register vector types used by the API
    register_vector<double>("VectorDouble");
    register_vector<uint64_t>("VectorUint64");
    register_vector<State>("VectorState");
    register_vector<Transition>("VectorTransition");

    // State: { id, embedding }
    class_<State>("State")
        .constructor<>()
        .property("id", &State::id)
        .property("embedding", &State::embedding);

    // Transition: { id, from, to, cost, safety, reliability, available }
    class_<Transition>("Transition")
        .constructor<>()
        .property("id", &Transition::id)
        .property("from", &Transition::from)
        .property("to", &Transition::to)
        .property("cost", &Transition::cost)
        .property("safety", &Transition::safety)
        .property("reliability", &Transition::reliability)
        .property("available", &Transition::available);

    // PlanningProblem: { initialState, goalState, badStates, states, transitions }
    class_<PlanningProblem>("PlanningProblem")
        .constructor<>()
        .property("initialState", &PlanningProblem::initialState)
        .property("goalState", &PlanningProblem::goalState)
        .property("badStates", &PlanningProblem::badStates)
        .property("states", &PlanningProblem::states)
        .property("transitions", &PlanningProblem::transitions);

    // PlanningResult: { success, statePath, transitionPath, totalCost, safetyScore }
    class_<PlanningResult>("PlanningResult")
        .constructor<>()
        .property("success", &PlanningResult::success)
        .property("statePath", &PlanningResult::statePath)
        .property("transitionPath", &PlanningResult::transitionPath)
        .property("totalCost", &PlanningResult::totalCost)
        .property("safetyScore", &PlanningResult::safetyScore);

    // LPAStarPlanner: the core planner with plan() and 4 replanning methods
    class_<LPAStarPlanner, base<Planner>>("LPAStarPlanner")
        .constructor<>()
        .function("plan", &LPAStarPlanner::plan)
        .function("onTransitionUnavailable", &LPAStarPlanner::onTransitionUnavailable)
        .function("onTransitionAdded", &LPAStarPlanner::onTransitionAdded)
        .function("onBadStatesChanged", &LPAStarPlanner::onBadStatesChanged)
        .function("onGoalChanged", &LPAStarPlanner::onGoalChanged)
        .property("statesExpanded", &LPAStarPlanner::statesExpanded);

    // Planner base class (needed for the inheritance chain)
    class_<Planner>("Planner")
        .function("plan", &Planner::plan, pure_virtual());
}
