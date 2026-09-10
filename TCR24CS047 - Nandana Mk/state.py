from dataclasses import dataclass


@dataclass
class State:
    """
    Represents a state (node) in the Cartesian space.
    """

    id: str
    x: float
    y: float

    def __repr__(self):
        return f"State({self.id}, {self.x}, {self.y})"