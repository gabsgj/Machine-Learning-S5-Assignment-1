#ifndef PLANNING_RESULT_HPP
#define PLANNING_RESULT_HPP

#include <cstdint>
#include <vector>
#include <string>
#include <sstream>

class PlanningResult {
public:
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;

    // Extended performance metrics for empirical evaluation
    double cumulativeReliability;
    double minBadStateDistance;
    size_t expandedStates;
    double planningTimeMicroseconds;

    PlanningResult()
        : success(false), totalCost(0.0), safetyScore(0.0),
          cumulativeReliability(1.0), minBadStateDistance(0.0),
          expandedStates(0), planningTimeMicroseconds(0.0) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "PlanningResult(success=" << (success ? "TRUE" : "FALSE")
            << ", totalCost=" << totalCost
            << ", safetyScore=" << safetyScore
            << ", minBadDist=" << minBadStateDistance
            << ", reliability=" << cumulativeReliability
            << ", pathLen=" << statePath.size()
            << ", expanded=" << expandedStates
            << ", timeUs=" << planningTimeMicroseconds
            << ")\nState Path: [";
        for (size_t i = 0; i < statePath.size(); ++i) {
            oss << statePath[i] << (i + 1 < statePath.size() ? " -> " : "");
        }
        oss << "]";
        return oss.str();
    }
};

#endif // PLANNING_RESULT_HPP
