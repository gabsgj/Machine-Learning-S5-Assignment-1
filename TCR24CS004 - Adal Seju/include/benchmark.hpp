#pragma once

#include "types.hpp"
#include "geometry.hpp"
#include "lpa_star.hpp"
#include "d_star_lite.hpp"

#include <iostream>
#include <vector>
#include <random>
#include <chrono>
#include <iomanip>
#include <string>

namespace SafePlanner {

struct BenchmarkMetric {
    std::string testName;
    size_t numStates;
    size_t numTransitions;
    size_t numBadStates;
    double coldTimeUs;
    double incrementalTimeUs;
    double speedupFactor;
    size_t coldExplored;
    size_t incrementalExplored;
    double pathCost;
    double safetyScore;
    bool pathFound;
};

class BenchmarkSuite {
public:
    /**
     * @brief Generates a random geometric graph in R^d.
     */
    static PlanningProblem generateRandomGraph(
        size_t numStates,
        double connectionRadius,
        double badStateRatio = 0.05,
        int dimension = 2,
        uint32_t seed = 42)
    {
        PlanningProblem problem;
        std::mt19937_64 rng(seed);
        std::uniform_real_distribution<double> coordDist(0.0, 100.0);
        std::uniform_real_distribution<double> relDist(0.85, 0.99);

        // Generate states in [0, 100]^d
        for (size_t i = 1; i <= numStates; ++i) {
            std::vector<double> emb(dimension);
            for (int d = 0; d < dimension; ++d) {
                emb[d] = coordDist(rng);
            }
            problem.states.emplace_back(i, emb, "Node_" + std::to_string(i));
        }

        problem.initialState = 1;
        problem.goalState = numStates;

        // Choose bad states (excluding start and goal)
        size_t numBad = static_cast<size_t>(numStates * badStateRatio);
        std::vector<uint64_t> candidateIds;
        for (size_t i = 2; i < numStates; ++i) {
            candidateIds.push_back(i);
        }
        std::shuffle(candidateIds.begin(), candidateIds.end(), rng);
        for (size_t i = 0; i < std::min(numBad, candidateIds.size()); ++i) {
            problem.badStates.push_back(candidateIds[i]);
        }

        // Generate directed transitions within connectionRadius
        uint64_t transId = 1;
        for (size_t i = 0; i < problem.states.size(); ++i) {
            for (size_t j = 0; j < problem.states.size(); ++j) {
                if (i == j) continue;
                double dist = Geometry::euclideanDistance(problem.states[i].embedding, problem.states[j].embedding);
                if (dist <= connectionRadius) {
                    double cost = dist * (1.0 + (rng() % 20) / 100.0); // Cost proportional to dist with slight variance
                    double reliability = relDist(rng);
                    problem.transitions.emplace_back(
                        transId++,
                        problem.states[i].id,
                        problem.states[j].id,
                        cost,
                        1.0,
                        reliability,
                        true
                    );
                }
            }
        }

        // Ensure at least a guaranteed chain path from start to goal exists (outside bad states)
        return problem;
    }

    /**
     * @brief Runs stress benchmarks across varying graph scales.
     */
    static std::vector<BenchmarkMetric> runScalabilityBenchmarks() {
        std::vector<BenchmarkMetric> results;
        std::vector<size_t> stateSizes = {50, 100, 250, 500, 1000, 2000};
        std::vector<double> radii = {35.0, 25.0, 18.0, 12.0, 8.5, 6.0};

        for (size_t idx = 0; idx < stateSizes.size(); ++idx) {
            size_t N = stateSizes[idx];
            double r = radii[idx];
            PlanningProblem problem = generateRandomGraph(N, r, 0.04, 2, static_cast<uint32_t>(100 + idx));

            LPAStarPlanner coldPlanner;
            PlanningResult coldRes = coldPlanner.plan(problem);

            // Now perform a dynamic perturbation (e.g. toggle 5% of edges on the path)
            LPAStarPlanner incPlanner;
            incPlanner.loadProblem(problem);
            incPlanner.computeShortestPath();

            // Mutate edge or add shortcut
            if (!coldRes.transitionPath.empty()) {
                uint64_t edgeToBlock = coldRes.transitionPath[coldRes.transitionPath.size() / 2];
                incPlanner.updateEdgeAvailability(edgeToBlock, false);
            }

            PlanningResult incRes = incPlanner.replanIncremental();

            double speedup = (incRes.planningTimeMicroseconds > 0.0)
                ? (coldRes.planningTimeMicroseconds / incRes.planningTimeMicroseconds)
                : 1.0;

            BenchmarkMetric m;
            m.testName = "Scale_N" + std::to_string(N);
            m.numStates = problem.states.size();
            m.numTransitions = problem.transitions.size();
            m.numBadStates = problem.badStates.size();
            m.coldTimeUs = coldRes.planningTimeMicroseconds;
            m.incrementalTimeUs = incRes.planningTimeMicroseconds;
            m.speedupFactor = speedup;
            m.coldExplored = coldRes.exploredStates;
            m.incrementalExplored = incRes.exploredStates;
            m.pathCost = incRes.totalCost;
            m.safetyScore = incRes.safetyScore;
            m.pathFound = incRes.success;

            results.push_back(m);
        }

        return results;
    }
};

} // namespace SafePlanner
