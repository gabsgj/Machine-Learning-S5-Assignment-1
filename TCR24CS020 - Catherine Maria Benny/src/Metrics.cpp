#include "Metrics.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace metrics {

double euclideanDistance(const std::vector<double>& a, const std::vector<double>& b) {
    double sum = 0.0;
    size_t n = std::min(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        double d = a[i] - b[i];
        sum += d * d;
    }
    return std::sqrt(sum);
}

double minimumSafetyDistance(const std::vector<uint64_t>& statePath,
                              const std::vector<uint64_t>& badStates,
                              const std::unordered_map<uint64_t, State>& stateById) {
    if (badStates.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    double best = std::numeric_limits<double>::infinity();
    for (uint64_t sid : statePath) {
        auto itS = stateById.find(sid);
        if (itS == stateById.end()) continue;
        for (uint64_t bid : badStates) {
            auto itB = stateById.find(bid);
            if (itB == stateById.end()) continue;
            double d = euclideanDistance(itS->second.embedding, itB->second.embedding);
            best = std::min(best, d);
        }
    }
    return best;
}

double cumulativeReliability(const std::vector<uint64_t>& transitionPath,
                              const std::unordered_map<uint64_t, Transition>& transitionById) {
    double r = 1.0;
    for (uint64_t tid : transitionPath) {
        auto it = transitionById.find(tid);
        if (it != transitionById.end()) {
            r *= it->second.reliability;
        }
    }
    return r;
}

double totalCost(const std::vector<uint64_t>& transitionPath,
                  const std::unordered_map<uint64_t, Transition>& transitionById) {
    double c = 0.0;
    for (uint64_t tid : transitionPath) {
        auto it = transitionById.find(tid);
        if (it != transitionById.end()) {
            c += it->second.cost;
        }
    }
    return c;
}

double objectiveScore(bool success, double cost, double minSafetyDistance,
                       double reliability, const ObjectiveWeights& w,
                       double noBadStateSentinel) {
    double G = success ? 1.0 : 0.0;
    double D = std::isfinite(minSafetyDistance) ? minSafetyDistance : noBadStateSentinel;
    return w.alpha * G - w.beta * cost + w.gamma * D + w.delta * reliability;
}

std::string validatePath(const PlanningProblem& problem,
                          const std::vector<uint64_t>& statePath,
                          const std::vector<uint64_t>& transitionPath) {
    if (statePath.empty()) {
        return "empty state path";
    }
    if (statePath.front() != problem.initialState) {
        return "path does not start at the initial state";
    }
    if (statePath.back() != problem.goalState) {
        return "path does not end at the goal state";
    }

    std::unordered_map<uint64_t, State> stateById;
    for (const auto& s : problem.states) stateById[s.id] = s;
    std::unordered_map<uint64_t, Transition> transitionById;
    for (const auto& t : problem.transitions) transitionById[t.id] = t;
    std::unordered_set<uint64_t> badSet(problem.badStates.begin(), problem.badStates.end());

    for (uint64_t sid : statePath) {
        if (stateById.find(sid) == stateById.end()) {
            std::ostringstream oss;
            oss << "state " << sid << " does not exist";
            return oss.str();
        }
        if (badSet.count(sid)) {
            std::ostringstream oss;
            oss << "path visits bad state " << sid;
            return oss.str();
        }
    }

    if (transitionPath.size() != statePath.size() - 1) {
        return "transition path length does not match state path length - 1";
    }

    for (size_t i = 0; i < transitionPath.size(); ++i) {
        auto it = transitionById.find(transitionPath[i]);
        if (it == transitionById.end()) {
            std::ostringstream oss;
            oss << "transition " << transitionPath[i] << " does not exist";
            return oss.str();
        }
        const Transition& t = it->second;
        if (!t.available) {
            std::ostringstream oss;
            oss << "transition " << t.id << " is not available";
            return oss.str();
        }
        if (t.from != statePath[i] || t.to != statePath[i + 1]) {
            std::ostringstream oss;
            oss << "transition " << t.id << " does not connect state " << statePath[i]
                << " -> " << statePath[i + 1];
            return oss.str();
        }
    }

    return "";
}

} // namespace metrics
