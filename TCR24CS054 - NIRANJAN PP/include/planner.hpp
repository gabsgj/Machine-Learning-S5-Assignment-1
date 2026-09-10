#pragma once

#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace safeplanner {

struct State {
    std::uint64_t id{};
    std::vector<double> embedding;
};

struct Transition {
    std::uint64_t id{};
    std::uint64_t from{};
    std::uint64_t to{};
    double cost{1.0};
    double safety{1.0};
    double reliability{1.0};
    bool available{true};
};

struct PlanningProblem {
    std::uint64_t initialState{};
    std::uint64_t goalState{};
    std::vector<std::uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

struct PlanningResult {
    bool success{false};
    std::vector<std::uint64_t> statePath;
    std::vector<std::uint64_t> transitionPath;
    double totalCost{0.0};
    double safetyScore{0.0};
    double cumulativeReliability{1.0};
    std::size_t exploredStates{0};
    double planningTimeMs{0.0};
    double replanningTimeMs{0.0};
};

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

class DStarLitePlanner final : public Planner {
public:
    explicit DStarLitePlanner(double safetyWeight = 2.0,
                              double reliabilityWeight = 1.0);

    PlanningResult plan(const PlanningProblem& problem) override;

    void setGoal(std::uint64_t goal);
    void setBadStates(const std::vector<std::uint64_t>& badStates);
    void setTransitionAvailable(std::uint64_t transitionId, bool available);
    void addTransition(const Transition& transition);
    void removeTransition(std::uint64_t transitionId);

    PlanningResult replan(std::uint64_t start);

private:
    struct Key {
        double k1{std::numeric_limits<double>::infinity()};
        double k2{std::numeric_limits<double>::infinity()};
    };

    struct QueueEntry {
        Key key;
        std::uint64_t state{};
        std::uint64_t serial{};
    };

    struct Compare {
        bool operator()(const QueueEntry& a, const QueueEntry& b) const;
    };

    double safetyWeight_;
    double reliabilityWeight_;
    double km_{0.0};
    std::uint64_t start_{0};
    std::uint64_t goal_{0};
    std::uint64_t serial_{0};

    std::unordered_map<std::uint64_t, State> states_;
    std::unordered_map<std::uint64_t, Transition> transitions_;
    std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> outgoing_;
    std::unordered_map<std::uint64_t, std::vector<std::uint64_t>> incoming_;
    std::unordered_set<std::uint64_t> bad_;

    std::unordered_map<std::uint64_t, double> g_;
    std::unordered_map<std::uint64_t, double> rhs_;

    std::priority_queue<QueueEntry, std::vector<QueueEntry>, Compare> open_;

    double inf() const;
    double heuristic(std::uint64_t a, std::uint64_t b) const;
    double distanceToBad(std::uint64_t state) const;
    bool isBad(std::uint64_t state) const;
    bool validState(std::uint64_t state) const;
    double edgeWeight(const Transition& t) const;

    double g(std::uint64_t s) const;
    double rhs(std::uint64_t s) const;
    void setG(std::uint64_t s, double value);
    void setRhs(std::uint64_t s, double value);

    Key calculateKey(std::uint64_t s) const;
    bool keyLess(const Key& a, const Key& b) const;
    void pushOpen(std::uint64_t s);
    void updateVertex(std::uint64_t u);
    void computeShortestPath(std::size_t& explored);

    void rebuildGraph();
    void resetSearch();
    PlanningResult buildResult(std::size_t explored, double elapsedMs,
                               double replanningMs = 0.0) const;
};

} // namespace safeplanner
