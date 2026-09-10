#include "VisualisationData.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <unordered_set>

namespace visualisation {

static std::string jsonEscape(const std::string& s) {
    std::string out;
    for (char c : s) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
    }
    return out;
}

// std::ostream prints +/-inf and nan as the bareword "inf"/"nan", which is
// not valid JSON (JSON has no infinity literal). We emit `null` instead,
// which the Python visualization scripts interpret as "no bad states in
// this problem instance" (the documented convention from Metrics.h).
static std::string jsonNumber(double v) {
    if (std::isnan(v) || std::isinf(v)) return "null";
    std::ostringstream oss;
    oss << v;
    return oss.str();
}

void exportToJson(const std::string& path, const PlanningProblem& problem,
                   const PlanningResult& result, const std::string& label) {
    std::ofstream out(path);
    std::unordered_set<uint64_t> badSet(problem.badStates.begin(), problem.badStates.end());
    std::unordered_set<uint64_t> pathStateSet(result.statePath.begin(), result.statePath.end());
    std::unordered_set<uint64_t> pathTransSet(result.transitionPath.begin(), result.transitionPath.end());

    out << "{\n";
    out << "  \"label\": \"" << jsonEscape(label) << "\",\n";
    out << "  \"states\": [\n";
    for (size_t i = 0; i < problem.states.size(); ++i) {
        const State& s = problem.states[i];
        out << "    {\"id\": " << s.id << ", \"embedding\": [";
        for (size_t d = 0; d < s.embedding.size(); ++d) {
            out << s.embedding[d];
            if (d + 1 < s.embedding.size()) out << ", ";
        }
        out << "], \"bad\": " << (badSet.count(s.id) ? "true" : "false")
            << ", \"onPath\": " << (pathStateSet.count(s.id) ? "true" : "false") << "}";
        if (i + 1 < problem.states.size()) out << ",";
        out << "\n";
    }
    out << "  ],\n";

    out << "  \"transitions\": [\n";
    for (size_t i = 0; i < problem.transitions.size(); ++i) {
        const Transition& t = problem.transitions[i];
        out << "    {\"id\": " << t.id << ", \"from\": " << t.from << ", \"to\": " << t.to
            << ", \"cost\": " << t.cost << ", \"safety\": " << t.safety
            << ", \"reliability\": " << t.reliability
            << ", \"available\": " << (t.available ? "true" : "false")
            << ", \"onPath\": " << (pathTransSet.count(t.id) ? "true" : "false") << "}";
        if (i + 1 < problem.transitions.size()) out << ",";
        out << "\n";
    }
    out << "  ],\n";

    out << "  \"initialState\": " << problem.initialState << ",\n";
    out << "  \"goalState\": " << problem.goalState << ",\n";

    out << "  \"badStates\": [";
    for (size_t i = 0; i < problem.badStates.size(); ++i) {
        out << problem.badStates[i];
        if (i + 1 < problem.badStates.size()) out << ", ";
    }
    out << "],\n";

    out << "  \"statePath\": [";
    for (size_t i = 0; i < result.statePath.size(); ++i) {
        out << result.statePath[i];
        if (i + 1 < result.statePath.size()) out << ", ";
    }
    out << "],\n";

    out << "  \"transitionPath\": [";
    for (size_t i = 0; i < result.transitionPath.size(); ++i) {
        out << result.transitionPath[i];
        if (i + 1 < result.transitionPath.size()) out << ", ";
    }
    out << "],\n";

    out << "  \"metrics\": {\n";
    out << "    \"success\": " << (result.success ? "true" : "false") << ",\n";
    out << "    \"totalCost\": " << jsonNumber(result.totalCost) << ",\n";
    out << "    \"minimumSafetyDistance\": " << jsonNumber(result.safetyScore) << ",\n";
    out << "    \"reliability\": " << jsonNumber(result.reliability) << ",\n";
    out << "    \"objectiveScore\": " << jsonNumber(result.objectiveScore) << ",\n";
    out << "    \"badStatesVisited\": " << result.badStatesVisited << ",\n";
    out << "    \"exploredStates\": " << result.exploredStates << ",\n";
    out << "    \"planningTimeMs\": " << result.planningTimeMs << ",\n";
    out << "    \"errorMessage\": \"" << jsonEscape(result.errorMessage) << "\"\n";
    out << "  }\n";
    out << "}\n";
}

} // namespace visualisation
