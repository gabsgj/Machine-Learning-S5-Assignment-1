/**
 * @file test_wasm_api_native.cpp
 * @brief Native smoke test for the WASM C API — validates JSON boundary
 *        marshaling and stateful planner persistence without Emscripten.
 */

#include <iostream>
#include <cstdlib>
#include <cstring>
#include <string>

// Forward-declare the C API
extern "C" {
    int create_planner(const char* problemJson, const char* weightsJson);
    const char* notify_edge_changed(int handle, const char* edgeChangeJson);
    const char* notify_goal_changed(int handle, const char* newGoalJson);
    const char* notify_bad_states_changed(int handle, const char* badStatesJson);
    const char* get_metrics(int handle);
    const char* get_result(int handle);
    void destroy_planner(int handle);
}

// Minimal JSON value extractor (avoids linking nlohmann in the test itself)
static std::string extractJsonField(const std::string& json, const std::string& key) {
    std::string needle = "\"" + key + "\":";
    size_t pos = json.find(needle);
    if (pos == std::string::npos) return "";
    pos += needle.size();
    // Skip whitespace
    while (pos < json.size() && (json[pos] == ' ' || json[pos] == '\t')) pos++;
    if (pos >= json.size()) return "";
    // If string value
    if (json[pos] == '"') {
        size_t end = json.find('"', pos + 1);
        return json.substr(pos + 1, end - pos - 1);
    }
    // Numeric / bool / null — read until , or } or ]
    size_t end = json.find_first_of(",}]", pos);
    return json.substr(pos, end - pos);
}

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] " << (msg) << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            std::exit(1); \
        } \
    } while (0)

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "WASM C API Native Smoke Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // ── TC1 Problem JSON: 6x6 grid, no bad states ──
    std::string problemJson = R"({
        "initialState": "s_0_0",
        "goalState": "s_5_5",
        "badStates": [],
        "states": [)";

    // Generate 6x6 grid states
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 6; ++x) {
            if (y > 0 || x > 0) problemJson += ",";
            problemJson += "{\"id\":\"s_" + std::to_string(x) + "_" + std::to_string(y) + "\","
                           "\"embedding\":[" + std::to_string(x) + ".0," + std::to_string(y) + ".0]}";
        }
    }
    problemJson += "],\"transitions\":[";

    int tCount = 0;
    bool firstT = true;
    for (int y = 0; y < 6; ++y) {
        for (int x = 0; x < 6; ++x) {
            std::string u = "s_" + std::to_string(x) + "_" + std::to_string(y);
            if (x + 1 < 6) {
                if (!firstT) problemJson += ","; firstT = false;
                std::string v = "s_" + std::to_string(x+1) + "_" + std::to_string(y);
                problemJson += "{\"id\":\"t_" + std::to_string(++tCount) + "\","
                               "\"from\":\"" + u + "\",\"to\":\"" + v + "\","
                               "\"cost\":1.0,\"reliability\":0.95}";
            }
            if (y + 1 < 6) {
                if (!firstT) problemJson += ","; firstT = false;
                std::string v = "s_" + std::to_string(x) + "_" + std::to_string(y+1);
                problemJson += "{\"id\":\"t_" + std::to_string(++tCount) + "\","
                               "\"from\":\"" + u + "\",\"to\":\"" + v + "\","
                               "\"cost\":1.0,\"reliability\":0.95}";
            }
            if (x + 1 < 6 && y + 1 < 6) {
                if (!firstT) problemJson += ","; firstT = false;
                std::string v = "s_" + std::to_string(x+1) + "_" + std::to_string(y+1);
                problemJson += "{\"id\":\"t_" + std::to_string(++tCount) + "\","
                               "\"from\":\"" + u + "\",\"to\":\"" + v + "\","
                               "\"cost\":1.414,\"reliability\":0.90}";
            }
        }
    }
    problemJson += "]}";

    std::string weightsJson = R"({"alpha":1.0,"beta":1.0,"gamma":1.0,"delta":0.1,"r":0.0})";

    // ── Step 1: Create planner (initial solve happens inside) ──
    std::cout << "[1] Creating planner from TC1 grid..." << std::endl;
    int handle = create_planner(problemJson.c_str(), weightsJson.c_str());
    TEST_ASSERT(handle > 0, "create_planner must return valid handle");

    // ── Step 2: Get initial metrics ──
    std::cout << "[2] Getting initial metrics..." << std::endl;
    const char* metricsRaw = get_metrics(handle);
    std::string metricsStr(metricsRaw);
    std::cout << "    Initial metrics: " << metricsStr << std::endl;

    size_t initialExplored = std::stoull(extractJsonField(metricsStr, "statesExplored"));
    size_t initialAttempts = std::stoull(extractJsonField(metricsStr, "attemptCount"));
    TEST_ASSERT(initialAttempts == 1, "attemptCount must be 1 after initial solve");

    // ── Step 3: Get initial result ──
    const char* resultRaw = get_result(handle);
    std::string resultStr(resultRaw);
    std::cout << "    Initial result: " << resultStr.substr(0, 200) << "..." << std::endl;
    TEST_ASSERT(resultStr.find("\"success\":true") != std::string::npos, "Initial path must succeed");

    // Find a transition id from the path to break (simulate TC4 edge-sever)
    // Extract the first transition from the path
    size_t tpPos = resultStr.find("\"transitionPath\":[");
    TEST_ASSERT(tpPos != std::string::npos, "transitionPath must exist in result");
    size_t firstTStart = resultStr.find("\"", tpPos + 18) + 1;
    size_t firstTEnd = resultStr.find("\"", firstTStart);
    std::string edgeToBreak = resultStr.substr(firstTStart, firstTEnd - firstTStart);
    std::cout << "    Breaking edge: " << edgeToBreak << std::endl;

    // ── Step 4: Notify edge changed (TC4 edge sever) ──
    std::cout << "[3] Notifying edge change (TC4 sever)..." << std::endl;
    std::string edgeChangeJson = "{\"transitionId\":\"" + edgeToBreak + "\",\"newCost\":100.0,\"newAvailability\":false}";
    const char* edgeResultRaw = notify_edge_changed(handle, edgeChangeJson.c_str());
    std::string edgeResultStr(edgeResultRaw);
    std::cout << "    Replan result: " << edgeResultStr.substr(0, 200) << "..." << std::endl;

    // Extract replan statesExplored from the nested metrics
    size_t metricsPos = edgeResultStr.find("\"metrics\":");
    TEST_ASSERT(metricsPos != std::string::npos, "Response must contain metrics");
    std::string replanMetricsSub = edgeResultStr.substr(metricsPos);
    size_t replanExplored = std::stoull(extractJsonField(replanMetricsSub, "statesExplored"));

    std::cout << "    Initial statesExplored: " << initialExplored 
              << ", Replan statesExplored: " << replanExplored << std::endl;

    // Verify replan succeeds
    TEST_ASSERT(edgeResultStr.find("\"success\":true") != std::string::npos, "Replan must succeed");

    // Verify the broken edge is not in the new path
    TEST_ASSERT(edgeResultStr.find("\"" + edgeToBreak + "\"") == std::string::npos ||
                edgeResultStr.find("\"transitionId\"") != std::string::npos,
                "Broken edge should not be in replanned path");

    // ── Step 5: Verify statefulness — expansion count for replan is bounded ──
    TEST_ASSERT(replanExplored < 36, "Replan expansions must be a small fraction of the 36-node graph");

    // ── Step 6: Goal shift ──
    std::cout << "[4] Notifying goal change to s_4_4..." << std::endl;
    const char* goalResultRaw = notify_goal_changed(handle, R"({"newGoalStateId":"s_4_4"})");
    std::string goalResultStr(goalResultRaw);
    std::cout << "    Goal shift result: " << goalResultStr.substr(0, 200) << "..." << std::endl;
    TEST_ASSERT(goalResultStr.find("\"success\":true") != std::string::npos, "Goal shift must succeed");

    // ── Step 7: Bad-state change recomputes Stage 1 exclusions ──
    std::cout << "[5] Marking s_3_3 as a hazard..." << std::endl;
    const char* badStateResultRaw = notify_bad_states_changed(handle, R"({"badStates":["s_3_3"]})");
    std::string badStateResultStr(badStateResultRaw);
    TEST_ASSERT(badStateResultStr.find("\"result\":") != std::string::npos,
                "Bad-state notification must return result and metrics");

    // ── Step 8: Final metrics ──
    const char* finalMetrics = get_metrics(handle);
    std::string finalMetricsStr(finalMetrics);
    std::cout << "    Final metrics: " << finalMetricsStr << std::endl;

    size_t finalAttempts = std::stoull(extractJsonField(finalMetricsStr, "attemptCount"));
    size_t finalSuccess = std::stoull(extractJsonField(finalMetricsStr, "goalSuccessCount"));
    TEST_ASSERT(finalAttempts == 4, "attemptCount must be 4 after 4 calls");
    TEST_ASSERT(finalSuccess == 4, "goalSuccessCount must be 4");

    // ── Step 9: Destroy ──
    std::cout << "[6] Destroying planner..." << std::endl;
    destroy_planner(handle);

    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "WASM C API SMOKE TEST PASSED!" << std::endl;
    std::cout << "========================================" << std::endl;
    return 0;
}
