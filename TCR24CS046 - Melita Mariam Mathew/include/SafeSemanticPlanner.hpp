#ifndef SAFE_SEMANTIC_PLANNER_HPP
#define SAFE_SEMANTIC_PLANNER_HPP

#include "Planner.hpp"
#include "LPAStarPlanner.hpp"
#include "DstarlitePlanner.hpp"
#include <memory>
#include <string>

enum class AlgorithmType {
    LPA_STAR,
    D_STAR_LITE,
    SAFE_A_STAR
};

/**
 * @brief High-level Safe Semantic Planner facade supporting multiple algorithms,
 * dynamic updates, and multi-objective optimization.
 */
class SafeSemanticPlanner : public Planner {
public:
    double alpha; // Goal completion weight
    double beta;  // Transition cost weight
    double gamma; // Minimum safety clearance weight
    double delta; // Reliability weight
    double safetyRadius; // Desired safety buffer
    AlgorithmType algoType;

    SafeSemanticPlanner(AlgorithmType type = AlgorithmType::LPA_STAR,
                        double a = 100.0, double b = 1.0, double g = 5.0, double d = 2.0, double sRadius = 3.0);
    virtual ~SafeSemanticPlanner() override = default;

    virtual PlanningResult plan(const PlanningProblem& problem) override;

    // Dynamic environment API
    void init(const PlanningProblem& problem);
    PlanningResult replan();
    void setTransitionAvailability(uint64_t tid, bool avail, double cost = -1.0);
    void insertTransition(const Transition& t);
    void removeTransition(uint64_t tid);
    void updateBadStates(const std::vector<uint64_t>& bads);
    void setGoal(uint64_t newGoal);
    void setStart(uint64_t newStart);

    std::string getAlgorithmName() const;

private:
    std::unique_ptr<LPAStarPlanner> lpaPlanner;
    std::unique_ptr<DstarlitePlanner> dstarPlanner;
    PlanningProblem currentProblem;
    bool initialized;
};

#endif // SAFE_SEMANTIC_PLANNER_HPP
