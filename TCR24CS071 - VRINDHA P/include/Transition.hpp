#ifndef TRANSITION_HPP
#define TRANSITION_HPP

#include <cstdint>
#include <string>
#include <sstream>

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

    Transition(uint64_t id_, uint64_t from_, uint64_t to_, double cost_,
               double safety_ = 1.0, double reliability_ = 1.0, bool available_ = true)
        : id(id_), from(from_), to(to_), cost(cost_), safety(safety_),
          reliability(reliability_), available(available_) {}

    std::string toString() const {
        std::ostringstream oss;
        oss << "Transition(id=" << id << ", " << from << "->" << to
            << ", cost=" << cost << ", safety=" << safety
            << ", rel=" << reliability << ", avail=" << (available ? "true" : "false") << ")";
        return oss.str();
    }
};

#endif // TRANSITION_HPP
