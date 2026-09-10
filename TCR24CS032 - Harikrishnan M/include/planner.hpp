#pragma once
#include "planning_types.hpp"
#include "environment.hpp"
#include <unordered_map>
#include <set>

class Planner
{
public:
    virtual PlanningResult plan(const PlanningProblem &problem) = 0;
    virtual ~Planner() = default;
};

class LPAPlanner : public Planner
{
public:
    PlanningResult plan(const PlanningProblem &problem) override;
    PlanningResult replan();

    // Moved to public so test cases can modify the environment directly
    Environment env;

private:
    std::unordered_map<uint64_t, double> g;
    std::unordered_map<uint64_t, double> rhs;
    std::unordered_map<uint64_t, uint64_t> came_from;
    std::unordered_map<uint64_t, uint64_t> transition_used;
    std::set<std::pair<double, uint64_t>> U;

    uint64_t startId;
    double calcKey(uint64_t id);
    PlanningResult extractResult();
};