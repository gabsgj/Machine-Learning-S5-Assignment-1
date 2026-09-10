#include "Transition.h"

Transition::Transition()
    : id(0),
      from(0),
      to(0),
      cost(0.0),
      safety(0.0),
      reliability(0.0),
      available(true) {
}

Transition::Transition(
    uint64_t id,
    uint64_t from,
    uint64_t to,
    double cost,
    double safety,
    double reliability,
    bool available
)
    : id(id),
      from(from),
      to(to),
      cost(cost),
      safety(safety),
      reliability(reliability),
      available(available) {
}
