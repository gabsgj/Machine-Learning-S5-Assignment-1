#ifndef PLANNING_RESULT_HPP
#define PLANNING_RESULT_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>

/**
 * @brief Encapsulates the output of a planning search execution.
 */
class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;

    // Supplementary telemetry and evaluation metrics
    double cumulativeReliability;
    double minSafetyDistance;
    double compositeScore;
    uint64_t exploredStates;
    double planningTimeMs;
    double replanningTimeMs;
    uint64_t badStatesVisited;

    PlanningResult()
        : success(false), totalCost(0.0), safetyScore(0.0),
          cumulativeReliability(1.0), minSafetyDistance(0.0), compositeScore(0.0),
          exploredStates(0), planningTimeMs(0.0), replanningTimeMs(0.0), badStatesVisited(0) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "PlanningResult(success=" << (success ? "TRUE" : "FALSE")
            << ", totalCost=" << std::fixed << std::setprecision(4) << totalCost
            << ", safetyScore=" << safetyScore
            << ", minSafetyDist=" << minSafetyDistance
            << ", cumReliability=" << cumulativeReliability
            << ", compositeScore=" << compositeScore
            << ", badStatesVisited=" << badStatesVisited
            << ", exploredStates=" << exploredStates
            << ", time=" << planningTimeMs << " ms"
            << ", pathLen=" << statePath.size() << ")";
        return oss.str();
    }

    std::string pathString() const {
        if (!success || statePath.empty()) return "No path found";
        std::ostringstream oss;
        for (size_t i = 0; i < statePath.size(); ++i) {
            oss << statePath[i] << (i + 1 < statePath.size() ? " -> " : "");
        }
        return oss.str();
    }
};

#endif // PLANNING_RESULT_HPP
