#include "GraphGenerator.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <vector>
#include "Metrics.h"

namespace graphgen {

// cost(u,v) = EuclideanDistance(u,v) * factor, factor in [1, costSlack]. This
// keeps cost(u,v) >= EuclideanDistance(u,v) for every generated edge, which
// is exactly the invariant DStarLite::heuristic() relies on for the
// heuristic h(s,start)=beta*EuclideanDistance(s,start) to be admissible (see
// DStarLite.cpp and report.md section 9). A pure uniform-random cost,
// independent of geometry, would routinely be far smaller than the
// straight-line distance across a 100x100 area and silently break
// admissibility for any experiment at scale.
static double costForEdge(const std::vector<double>& a, const std::vector<double>& b,
                           double costSlack, std::mt19937& rng) {
    double dist = metrics::euclideanDistance(a, b);
    std::uniform_real_distribution<double> factor(1.0, std::max(1.0, costSlack));
    return dist * factor(rng);
}

PlanningProblem generate(const GraphGenConfig& cfg) {
    PlanningProblem problem;
    std::mt19937 rng(cfg.seed);
    std::uniform_real_distribution<double> coord(cfg.areaMin, cfg.areaMax);
    std::uniform_real_distribution<double> relDist(cfg.minReliability, cfg.maxReliability);
    std::uniform_real_distribution<double> safeDist(cfg.minSafety, cfg.maxSafety);
    std::uniform_real_distribution<double> unif01(0.0, 1.0);

    int n = cfg.numStates;
    for (int i = 0; i < n; ++i) {
        std::vector<double> emb(cfg.dimensions);
        for (int d = 0; d < cfg.dimensions; ++d) emb[d] = coord(rng);
        problem.states.emplace_back(static_cast<uint64_t>(i), emb);
    }

    problem.initialState = 0;
    problem.goalState = static_cast<uint64_t>(n - 1);

    uint64_t nextTransitionId = 0;

    // Guarantee a reachable path from initial to goal via a random
    // permutation "backbone" chain, so experiments always have a valid
    // baseline path to compare against, independent of the random density.
    std::vector<int> mid;
    mid.reserve(n > 2 ? n - 2 : 0);
    for (int i = 1; i < n - 1; ++i) mid.push_back(i);
    std::shuffle(mid.begin(), mid.end(), rng);
    std::vector<int> order;
    order.reserve(n);
    order.push_back(0);
    for (int v : mid) order.push_back(v);
    if (n > 1) order.push_back(n - 1);
    for (int i = 0; i + 1 < static_cast<int>(order.size()); ++i) {
        Transition t;
        t.id = nextTransitionId++;
        t.from = static_cast<uint64_t>(order[i]);
        t.to = static_cast<uint64_t>(order[i + 1]);
        t.cost = costForEdge(problem.states[order[i]].embedding,
                              problem.states[order[i + 1]].embedding, cfg.costSlack, rng);
        t.reliability = relDist(rng);
        t.safety = safeDist(rng);
        t.available = true; // backbone edges always start available
        problem.transitions.push_back(t);
    }

    // Random extra directed edges for realistic density.
    std::bernoulli_distribution edgeCoin(cfg.edgeProbability);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) continue;
            if (!edgeCoin(rng)) continue;
            Transition t;
            t.id = nextTransitionId++;
            t.from = static_cast<uint64_t>(i);
            t.to = static_cast<uint64_t>(j);
            t.cost = costForEdge(problem.states[i].embedding, problem.states[j].embedding,
                                  cfg.costSlack, rng);
            t.reliability = relDist(rng);
            t.safety = safeDist(rng);
            t.available = unif01(rng) >= cfg.unavailableProbability;
            problem.transitions.push_back(t);
        }
    }

    // Random bad states, excluding initial and goal.
    std::vector<int> candidates;
    for (int i = 1; i < n - 1; ++i) candidates.push_back(i);
    std::shuffle(candidates.begin(), candidates.end(), rng);
    int numBad = std::min<int>(cfg.numBadStates, static_cast<int>(candidates.size()));
    for (int i = 0; i < numBad; ++i) {
        problem.badStates.push_back(static_cast<uint64_t>(candidates[i]));
    }

    return problem;
}

} // namespace graphgen
