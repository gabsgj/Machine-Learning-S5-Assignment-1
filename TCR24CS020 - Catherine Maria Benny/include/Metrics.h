#pragma once
#include <cstdint>
#include <unordered_map>
#include <vector>
#include "PlanningProblem.h"
#include "PlanningResult.h"

// Configurable weights for Score(P) = alpha*G - beta*C + gamma*D + delta*R
// G, avoidance of bad states, are treated as HARD constraints (see report),
// so alpha mainly matters as a large bonus distinguishing "reached goal" from
// "did not" in comparative logs; beta/gamma/delta trade cost vs safety vs reliability
// among paths that already satisfy the hard constraints.
struct ObjectiveWeights {
    double alpha = 100.0; // weight on goal completion (1.0 if success else 0.0)
    double beta = 1.0;    // weight on cumulative cost (penalized)
    double gamma = 2.0;   // weight on minimum safety distance (rewarded)
    double delta = 10.0;  // weight on cumulative reliability (rewarded)
};

namespace metrics {

// Euclidean distance between two Cartesian embeddings. Both vectors must have
// the same dimensionality (validated at problem-load time).
double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b);

// D(P) = min over s in P ( min over b in B ( EuclideanDistance(s, b) ) )
// If B is empty, this function returns +infinity (documented convention:
// "no bad states" means no safety penalty applies, so the safety term does
// not constrain path selection at all).
double minimumSafetyDistance(const std::vector<uint64_t>& statePath,
                              const std::vector<uint64_t>& badStates,
                              const std::unordered_map<uint64_t, State>& stateById);

// R(P) = product of transition reliabilities along the path.
double cumulativeReliability(const std::vector<uint64_t>& transitionPath,
                              const std::unordered_map<uint64_t, Transition>& transitionById);

// Sum of transition costs along the path.
double totalCost(const std::vector<uint64_t>& transitionPath,
                  const std::unordered_map<uint64_t, Transition>& transitionById);

// Score(P) = alpha*G - beta*C + gamma*D + delta*R
// D is clamped to a finite "noBadStateSentinel" value when infinite (no bad
// states in the problem) so the score stays finite and comparable.
double objectiveScore(bool success, double cost, double minSafetyDistance,
                       double reliability, const ObjectiveWeights& w,
                       double noBadStateSentinel = 1000.0);

// Independently validates a produced path against the raw problem data,
// without using any of the planner's internal search state. Checks:
// 1. first state == initial, 2. last state == goal, 3. all states exist,
// 4. all transitions exist and connect consecutive states correctly,
// 5. all transitions are currently available, 6. no state on the path is bad.
// Returns "" if valid, otherwise a human-readable reason.
std::string validatePath(const PlanningProblem& problem,
                          const std::vector<uint64_t>& statePath,
                          const std::vector<uint64_t>& transitionPath);

} // namespace metrics
