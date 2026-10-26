#include "State.h"

State::State()
    : id(0) {
}

State::State(
    uint64_t id,
    const std::vector<double>& embedding
)
    : id(id),
      embedding(embedding) {
}
