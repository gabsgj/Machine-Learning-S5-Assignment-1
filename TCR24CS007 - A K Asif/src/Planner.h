#ifndef PLANNER_H
#define PLANNER_H

#include "PlanningProblem.h"
#include "PlanningResult.h"

#include <cstdint>
#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>

class Planner {
public:
    explicit Planner(double safetyWeight = 2.0,
                     double reliabilityWeight = 1.0);

    PlanningResult plan(const PlanningProblem& problem);
    PlanningResult replan(const PlanningProblem& problem);

private:
    struct Node {
        uint64_t id{};
        double f{};
        double g{};
        bool operator>(const Node& other) const {
            if (f != other.f) return f > other.f;
            return g > other.g;
        }
    };

    const double INF = std::numeric_limits<double>::infinity();

    double safetyWeight_;
    double reliabilityWeight_;

    PlanningProblem problem_;
    std::unordered_map<uint64_t, State> states_;
    std::unordered_map<uint64_t, Transition> transitions_;
    std::unordered_map<uint64_t, std::vector<uint64_t>> outgoing_;

    std::size_t expanded_{0};

    void buildGraph();
    bool isBad(uint64_t stateId) const;
    double euclideanDistance(uint64_t a, uint64_t b) const;
    double safetyDistance(uint64_t stateId) const;
    double effectiveCost(const Transition& t) const;

    PlanningResult runLPAStar();
    PlanningResult makeResult(
        uint64_t start,
        uint64_t goal,
        const std::unordered_map<uint64_t, double>& g,
        const std::unordered_map<uint64_t, uint64_t>& parentState,
        const std::unordered_map<uint64_t, uint64_t>& parentTransition) const;
};

#endif
