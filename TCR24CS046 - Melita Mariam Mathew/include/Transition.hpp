#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include <cstdint>
#include <string>
#include <sstream>

/**
 * @brief Represents a directed transition between two states.
 */
class Transition {
public:
    uint64_t id;
    uint64_t from;
    uint64_t to;
    double cost;
    double safety;
    double reliability;
    bool available;

    Transition()
        : id(0), from(0), to(0), cost(1.0), safety(1.0), reliability(1.0), available(true) {}

    Transition(uint64_t tId, uint64_t fromState, uint64_t toState,
               double tCost = 1.0, double tSafety = 1.0, double tReliability = 1.0, bool isAvailable = true)
        : id(tId), from(fromState), to(toState), cost(tCost), safety(tSafety),
          reliability(tReliability), available(isAvailable) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "Transition(id=" << id << ", " << from << " -> " << to
            << ", cost=" << cost << ", safety=" << safety
            << ", rel=" << reliability << ", avail=" << (available ? "true" : "false") << ")";
        return oss.str();
    }
};

#endif // TRANSITION_HPP
