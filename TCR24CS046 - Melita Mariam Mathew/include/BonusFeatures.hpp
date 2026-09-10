#ifndef BONUS_FEATURES_HPP
#define BONUS_FEATURES_HPP

#include "PlanningProblem.hpp"
#include "PlanningResult.hpp"
#include "LPAStarPlanner.hpp"
#include <vector>
#include <string>
#include <map>

// 1. Time-Dependent Transition Availability
struct TimeWindow {
    double startTime;
    double endTime;
};

class TimeDependentEnvironment {
public:
    PlanningProblem problem;
    std::unordered_map<uint64_t, std::vector<TimeWindow>> transitionSchedules;

    TimeDependentEnvironment(const PlanningProblem& prob) : problem(prob) {}

    void addSchedule(uint64_t transitionId, double start, double end) {
        transitionSchedules[transitionId].push_back(TimeWindow{start, end});
    }

    bool isTransitionOpenAt(uint64_t transitionId, double currentTime) const {
        auto it = transitionSchedules.find(transitionId);
        if (it == transitionSchedules.end()) return true; // default always open
        for (const auto& w : it->second) {
            if (currentTime >= w.startTime && currentTime <= w.endTime) return true;
        }
        return false;
    }
};

// 2. Multi-Goal Planner (Sequencing multiple waypoints/goals safely)
class MultiGoalPlanner {
public:
    static PlanningResult planMultiGoalTour(
        const PlanningProblem& baseProblem,
        const std::vector<uint64_t>& goalSequence,
        LPAStarPlanner& planner);
};

// 3. Semantic Knowledge Graph Reasoner
struct KGNode {
    uint64_t id;
    std::string conceptName;
    std::vector<double> semanticEmbedding;
};

struct KGRelation {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    std::string relationType;
    double confidence;
    double safetyScore;
};

class KnowledgeGraphSemanticSearch {
public:
    static PlanningProblem buildSemanticKGProblem(
        const std::vector<KGNode>& nodes,
        const std::vector<KGRelation>& relations,
        uint64_t startConcept,
        uint64_t targetConcept,
        const std::vector<uint64_t>& tabooConcepts);
};

#endif // BONUS_FEATURES_HPP
