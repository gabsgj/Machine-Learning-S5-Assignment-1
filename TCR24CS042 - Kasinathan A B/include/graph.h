#ifndef GRAPH_H
#define GRAPH_H

#include <cstdint>
#include <vector>
#include <unordered_map>
#include "transition.h"
#include "planning_problem.h"

class Graph {
public:
    Graph(const PlanningProblem& problem);

    const std::vector<Transition>& getOutgoing(uint64_t stateId) const;
    void addTransition(const Transition& t);
    void removeTransition(uint64_t transitionId, uint64_t fromState);
    void setAvailable(uint64_t transitionId, uint64_t fromState, bool available);

private:
    std::unordered_map<uint64_t, std::vector<Transition>> adjacency;
    std::vector<Transition> empty; // returned when a state has no outgoing edges
};

#endif