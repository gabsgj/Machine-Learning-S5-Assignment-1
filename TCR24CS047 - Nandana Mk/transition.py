from dataclasses import dataclass


@dataclass
class Transition:
    """
    Represents a directed edge between two states.
    """

    from_state: str
    to_state: str
    cost: float
    reliability: float
    safety: float
    available: bool = True

    def __repr__(self):
        return (
            f"Transition({self.from_state} -> {self.to_state}, "
            f"Cost={self.cost}, "
            f"Reliability={self.reliability}, "
            f"Safety={self.safety}, "
            f"Available={self.available})"
        )