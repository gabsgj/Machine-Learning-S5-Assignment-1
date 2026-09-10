#pragma once

#include "interfaces.hpp"

#include <cstdint>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

struct Key {
    double first;
    double second;
};

class PriorityQueue {
public:
    struct Entry {
        uint64_t state;
        Key key;
    };

    void push(uint64_t state, Key key);
    bool empty() const;
    const Entry& top() const;
    void pop();
    void clear();

private:
    struct LaterKey {
        bool operator()(const Entry& a, const Entry& b) const;
    };
    std::priority_queue<Entry, std::vector<Entry>, LaterKey> heap_;
};

class DStarLitePlanner : public Planner {
public:
    explicit DStarLitePlanner(double alpha = 0.6, double beta = 0.4);
    void setQualityWeights(double alpha, double beta);
    PlanningResult plan(const PlanningProblem& problem) override;

private:
    static constexpr double kInfinity = 1e100;
    static constexpr double kEpsilon = 1e-10;

    PlanningProblem problem_;
    bool initialized_ = false;
    uint64_t start_ = 0;
    uint64_t goal_ = 0;
    double km_ = 0.0;
    double alpha_ = 0.6;
    double beta_ = 0.4;
    double maxClearance_ = 0.0;
    std::size_t exploredStates_ = 0;

    std::unordered_map<uint64_t, const State*> states_;
    std::unordered_map<uint64_t, std::vector<const Transition*>> outgoing_;
    std::unordered_map<uint64_t, std::vector<const Transition*>> incoming_;
    std::unordered_set<uint64_t> bad_;
    std::unordered_map<uint64_t, double> clearance_;
    std::unordered_map<uint64_t, double> safety_;
    std::unordered_map<uint64_t, double> continuationSafety_;
    std::unordered_map<uint64_t, double> reliability_;
    std::unordered_map<uint64_t, double> g_;
    std::unordered_map<uint64_t, double> rhs_;
    PriorityQueue open_;

    bool buildGraph();
    void initialize(const PlanningProblem& problem);
    void applyChanges(const PlanningProblem& problem);
    Key calculateKey(uint64_t u) const;
    void updateVertex(uint64_t u);
    void computeShortestPath();
    std::vector<const Transition*> successors(uint64_t u) const;
    std::vector<const Transition*> predecessors(uint64_t u) const;
    const Transition* getTransition(uint64_t from, uint64_t to) const;
    double heuristic(uint64_t a, uint64_t b) const;
    double calculateQuality(uint64_t u) const;
    bool extractPath(
        std::vector<uint64_t>& states,
        std::vector<uint64_t>& transitions
    ) const;
    double calculatePathReliability(const std::vector<uint64_t>& transitionPath) const;
    double calculatePathSafety(const std::vector<uint64_t>& statePath) const;
    static bool lessKey(const Key& a, const Key& b);
    static bool equal(double a, double b);
};
