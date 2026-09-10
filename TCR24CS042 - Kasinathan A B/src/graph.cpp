#include "../include/graph.h"

Graph::Graph(const PlanningProblem& problem) {
    for (const Transition& t : problem.transitions) {
        adjacency[t.from].push_back(t);
    }
}

const std::vector<Transition>& Graph::getOutgoing(uint64_t stateId) const {
    auto it = adjacency.find(stateId);
    if (it == adjacency.end()) {
        return empty;
    }
    return it->second;
}

void Graph::addTransition(const Transition& t) {
    adjacency[t.from].push_back(t);
}

void Graph::removeTransition(uint64_t transitionId, uint64_t fromState) {
    auto& list = adjacency[fromState];
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].id == transitionId) {
            list.erase(list.begin() + i);
            return;
        }
    }
}

void Graph::setAvailable(uint64_t transitionId, uint64_t fromState, bool available) {
    auto& list = adjacency[fromState];
    for (size_t i = 0; i < list.size(); i++) {
        if (list[i].id == transitionId) {
            list[i].available = available;
            return;
        }
    }
}