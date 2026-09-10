#ifndef PLANNER_HPP
#define PLANNER_HPP

#include <cstdint>
#include <vector>

struct State {
    uint64_t id;
    std::vector<double> embedding;
};

struct Transition {
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;
};

struct PlanningProblem {
    uint64_t initialState;
    uint64_t goalState;
    std::vector<uint64_t> badStates;
    std::vector<State> states;
    std::vector<Transition> transitions;
};

struct PlanningResult {
    bool success;
    std::vector<uint64_t> statePath;
    std::vector<uint64_t> transitionPath;
    double totalCost;
    double safetyScore;
};

class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

class SafePlanner : public Planner {
public:
    SafePlanner();
    PlanningResult plan(const PlanningProblem& problem) override;
    void setAlpha(double a);
    void setBeta(double b);
    void setGamma(double g);

private:
    double alpha;
    double beta;
    double gamma;
    
    double getDistance(const std::vector<double>& a, const std::vector<double>& b);
    double getSafety(uint64_t stateId, const PlanningProblem& problem);
};

#endif
