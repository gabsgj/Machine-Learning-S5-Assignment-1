/**
 * @file wasm_api.cpp
 * @brief Handle-based stateful C API for the D* Lite planner, compiled to WASM.
 *        Uses nlohmann::json for boundary marshaling.
 *        JSON strings in/out via EMSCRIPTEN_KEEPALIVE functions.
 */

#ifdef __EMSCRIPTEN__
#include <emscripten/emscripten.h>
#else
// Native stub for non-WASM builds (testing)
#define EMSCRIPTEN_KEEPALIVE
#endif

#include "safe_semantic_planner/core_types.hpp"
#include "safe_semantic_planner/problem_loader.hpp"
#include "safe_semantic_planner/dstar_lite.hpp"
#include "nlohmann/json.hpp"

#include <unordered_map>
#include <string>
#include <cstring>
#include <cstdlib>

using json = nlohmann::json;
using namespace safe_semantic_planner;

// ─── Global planner instance store ──────────────────────────────────────────

static int nextHandle_ = 1;
static std::unordered_map<int, DStarLitePlanner*> planners_;

// ─── Persistent return buffer ───────────────────────────────────────────────
// We maintain a static string so the pointer returned to JS stays valid
// until the next call to any returning function.
static std::string returnBuffer_;

static const char* setReturn(const std::string& s) {
    returnBuffer_ = s;
    return returnBuffer_.c_str();
}

// ─── JSON ↔ Domain Marshaling ───────────────────────────────────────────────

static State stateFromJson(const json& j) {
    State s;
    s.id = j.at("id").get<std::string>();
    s.label = j.value("label", s.id);
    if (j.count("embedding")) {
        s.embedding = j.at("embedding").get<std::vector<double>>();
    }
    return s;
}

static Transition transitionFromJson(const json& j) {
    Transition t;
    t.id = j.at("id").get<std::string>();
    t.from = j.at("from").get<std::string>();
    t.to = j.at("to").get<std::string>();
    t.cost = j.value("cost", 1.0);
    t.reliability = j.value("reliability", 1.0);
    t.safetyScore = j.value("safetyScore", 1.0);
    t.available = j.value("available", true);
    return t;
}

static PlanningProblem problemFromJson(const json& j) {
    PlanningProblem p;
    p.initialState = j.at("initialState").get<std::string>();
    p.goalState = j.at("goalState").get<std::string>();
    if (j.count("badStates")) {
        p.badStates = j.at("badStates").get<std::vector<std::string>>();
    }
    for (const auto& sj : j.at("states")) {
        p.states.push_back(stateFromJson(sj));
    }
    for (const auto& tj : j.at("transitions")) {
        p.transitions.push_back(transitionFromJson(tj));
    }
    return p;
}

static WeightParams weightsFromJson(const json& j) {
    WeightParams w;
    w.alpha = j.value("alpha", 1.0);
    w.beta  = j.value("beta",  1.0);
    w.gamma = j.value("gamma", 1.0);
    w.delta = j.value("delta", 0.0);
    w.r     = j.value("r",     0.0);
    return w;
}

static json resultToJson(const PlanningResult& r) {
    json j;
    j["success"]        = r.success;
    j["statePath"]      = r.statePath;
    j["transitionPath"] = r.transitionPath;
    j["totalCost"]      = r.totalCost;
    j["safetyScore"]    = r.safetyScore;
    j["errorMessage"]   = r.errorMessage;
    return j;
}

static json metricsToJson(const PlannerMetrics& m) {
    json j;
    j["statesExplored"]   = m.statesExplored;
    j["planningTimeMs"]   = m.planningTimeMs;
    j["replanningTimeMs"] = m.replanningTimeMs;
    j["memoryBytes"]      = m.memoryBytes;
    j["totalCost"]        = m.totalCost;
    j["minClearance"]     = m.minClearance;
    j["badStatesVisited"] = m.badStatesVisited;
    j["goalSuccessCount"] = m.goalSuccessCount;
    j["attemptCount"]     = m.attemptCount;
    j["goalSuccessRate"]  = m.getGoalSuccessRate();
    return j;
}

// ─── Public WASM API ────────────────────────────────────────────────────────

extern "C" {

/**
 * Creates a persistent planner instance from JSON problem + weights.
 * @param problemJson  JSON string encoding PlanningProblem.
 * @param weightsJson  JSON string encoding WeightParams.
 * @return handle (>0) on success, -1 on error (call get_last_error for message).
 */
EMSCRIPTEN_KEEPALIVE
int create_planner(const char* problemJson, const char* weightsJson) {
    try {
        json pj = json::parse(problemJson);
        json wj = json::parse(weightsJson);
        PlanningProblem problem = problemFromJson(pj);
        WeightParams weights = weightsFromJson(wj);

        DStarLitePlanner* planner = new DStarLitePlanner();
        planner->initialize(problem, weights);

        PlanningResult initResult = planner->computeShortestPath();

        int handle = nextHandle_++;
        planners_[handle] = planner;
        return handle;
    } catch (const std::exception& ex) {
        setReturn(std::string("{\"error\":\"") + ex.what() + "\"}");
        return -1;
    }
}

/**
 * Runs incremental edge update and returns updated result + metrics as JSON.
 */
EMSCRIPTEN_KEEPALIVE
const char* notify_edge_changed(int handle, const char* edgeChangeJson) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        json ej = json::parse(edgeChangeJson);
        std::string transitionId = ej.at("transitionId").get<std::string>();
        double newCost = ej.value("newCost", 1.0);
        bool newAvailability = ej.value("newAvailability", true);

        DStarLitePlanner* planner = it->second;
        planner->notifyEdgeChanged(transitionId, newCost, newAvailability);
        PlanningResult result = planner->computeShortestPath();
        PlannerMetrics metrics = planner->getMetrics();

        json response;
        response["result"]  = resultToJson(result);
        response["metrics"] = metricsToJson(metrics);
        return setReturn(response.dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Runs incremental goal shift and returns updated result + metrics as JSON.
 */
EMSCRIPTEN_KEEPALIVE
const char* notify_goal_changed(int handle, const char* newGoalJson) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        json gj = json::parse(newGoalJson);
        std::string newGoalStateId = gj.at("newGoalStateId").get<std::string>();

        DStarLitePlanner* planner = it->second;
        planner->notifyGoalChanged(newGoalStateId);
        PlanningResult result = planner->computeShortestPath();
        PlannerMetrics metrics = planner->getMetrics();

        json response;
        response["result"]  = resultToJson(result);
        response["metrics"] = metricsToJson(metrics);
        return setReturn(response.dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Recomputes Stage 1 safety exclusions after the set of bad states changes.
 */
EMSCRIPTEN_KEEPALIVE
const char* notify_bad_states_changed(int handle, const char* badStatesJson) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        json bj = json::parse(badStatesJson);
        std::vector<std::string> badStates = bj.at("badStates").get<std::vector<std::string>>();

        DStarLitePlanner* planner = it->second;
        planner->notifyBadStatesChanged(badStates);
        PlanningResult result = planner->computeShortestPath();
        PlannerMetrics metrics = planner->getMetrics();

        json response;
        response["result"] = resultToJson(result);
        response["metrics"] = metricsToJson(metrics);
        return setReturn(response.dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Re-solves with updated weight and safety parameters (alpha, beta, gamma, delta, r).
 */
EMSCRIPTEN_KEEPALIVE
const char* recompute_with_weights(int handle, const char* weightsJson) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        json wj = json::parse(weightsJson);
        WeightParams weights = weightsFromJson(wj);

        DStarLitePlanner* planner = it->second;
        planner->notifySafetyRadiusChanged(weights.r);
        PlanningResult result = planner->computeShortestPath();
        PlannerMetrics metrics = planner->getMetrics();

        json response;
        response["result"]  = resultToJson(result);
        response["metrics"] = metricsToJson(metrics);
        return setReturn(response.dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Returns current metrics as JSON without running any computation.
 */
EMSCRIPTEN_KEEPALIVE
const char* get_metrics(int handle) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        PlannerMetrics metrics = it->second->getMetrics();
        return setReturn(metricsToJson(metrics).dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Returns the current planning result as JSON.
 */
EMSCRIPTEN_KEEPALIVE
const char* get_result(int handle) {
    auto it = planners_.find(handle);
    if (it == planners_.end()) {
        return setReturn("{\"error\":\"Invalid planner handle\"}");
    }
    try {
        PlanningResult result = it->second->extractPath();
        PlannerMetrics metrics = it->second->getMetrics();
        json response;
        response["result"]  = resultToJson(result);
        response["metrics"] = metricsToJson(metrics);
        return setReturn(response.dump());
    } catch (const std::exception& ex) {
        json err;
        err["error"] = ex.what();
        return setReturn(err.dump());
    }
}

/**
 * Destroys the planner instance and frees all associated memory.
 */
EMSCRIPTEN_KEEPALIVE
void destroy_planner(int handle) {
    auto it = planners_.find(handle);
    if (it != planners_.end()) {
        delete it->second;
        planners_.erase(it);
    }
}

} // extern "C"
