#pragma once

#include "types.h"

#include <cmath>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

// ---------------------------------------------------------------------------
// Tunable weights  (compile-time; change freely for experiments)
// ---------------------------------------------------------------------------
constexpr double ALPHA            = 100.0; // goal-completion bonus (scoring)
constexpr double BETA             =   1.0; // cost weight
constexpr double GAMMA            =   2.0; // safety-distance weight
constexpr double DELTA            =   0.5; // reliability weight
constexpr double SAFETY_THRESHOLD =   5.0; // distance below which penalty grows
constexpr double INF_COST         =  1e18;

// ---------------------------------------------------------------------------
// D* Lite priority-queue key
// ---------------------------------------------------------------------------
struct Key {
    double k1 = 0, k2 = 0;
    bool operator<(const Key& o)  const { return k1 < o.k1 || (k1 == o.k1 && k2 < o.k2); }
    bool operator>(const Key& o)  const { return o < *this; }
    bool operator<=(const Key& o) const { return !(o < *this); }
};

struct PQEntry {
    Key      key;
    uint64_t state;
    bool operator<(const PQEntry& o) const {
        if (key < o.key) return true;
        if (o.key < key) return false;
        return state < o.state;
    }
};

// ---------------------------------------------------------------------------
// Abstract planner interface
// ---------------------------------------------------------------------------
class Planner {
public:
    virtual ~Planner() = default;
    virtual PlanningResult plan(const PlanningProblem& problem) = 0;
};

// ---------------------------------------------------------------------------
// D* Lite planner with safety awareness
// ---------------------------------------------------------------------------
class DStarLitePlanner : public Planner {
public:
    PlanningResult plan(const PlanningProblem& problem) override;

    /// Incremental replan after environment changes  (Bonus #1)
    PlanningResult replan(const std::vector<EnvironmentChange>& changes);

    /// Find best path among multiple candidate goals  (Bonus #2)
    PlanningResult planMultiGoal(PlanningProblem problem,
                                const std::vector<uint64_t>& goals);

private:
    // ---- stored problem data ------------------------------------------------
    PlanningProblem problem_;
    std::unordered_map<uint64_t, State>                       stateMap_;
    std::unordered_map<uint64_t, std::vector<const Transition*>> succAdj_;
    std::unordered_map<uint64_t, std::vector<const Transition*>> predAdj_;
    std::unordered_set<uint64_t>                              badSet_;
    std::vector<std::vector<double>>                          badPos_;
    std::unordered_map<uint64_t, Transition>                  transMap_;

    // ---- D* Lite search state -----------------------------------------------
    std::unordered_map<uint64_t, double> g_, rhs_;
    std::set<PQEntry>                    open_;
    std::unordered_map<uint64_t, Key>    openKeys_;
    double km_         = 0;
    int exploredCount_ = 0;

    // ---- helpers ------------------------------------------------------------
    void buildGraph();
    void initialize();
    Key  calcKey(uint64_t s);
    void updateVertex(uint64_t u);
    bool computeShortestPath();
    PlanningResult extractResult();

    double effectiveCost(const Transition& t) const;
    double minDistToBad(uint64_t sid) const;
    double heuristic(uint64_t a, uint64_t b) const;
    static double euclidean(const std::vector<double>& a,
                            const std::vector<double>& b);

    double getG  (uint64_t s) const { auto i = g_.find(s);   return i != g_.end()   ? i->second : INF_COST; }
    double getRhs(uint64_t s) const { auto i = rhs_.find(s); return i != rhs_.end() ? i->second : INF_COST; }

    void removeFromOpen(uint64_t s);
    void insertToOpen(uint64_t s, Key k);
};
