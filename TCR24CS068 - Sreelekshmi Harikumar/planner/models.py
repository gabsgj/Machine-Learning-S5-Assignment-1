from dataclasses import dataclass, field
from typing import List, Set


@dataclass
class State:
    """
    Represents a state in the Cartesian state space.
    """

    id: int
    name: str
    embedding: List[float]


@dataclass
class Transition:
    """
    Represents a directed transition between two states.
    """

    id: int
    from_state: int
    to_state: int

    cost: float = 1.0
    safety: float = 1.0
    reliability: float = 1.0
    available: bool = True


@dataclass
class PlanningProblem:
    """
    Complete planning problem.
    """

    initial_state: int
    goal_state: int

    bad_states: Set[int] = field(default_factory=set)

    states: List[State] = field(default_factory=list)
    transitions: List[Transition] = field(default_factory=list)


@dataclass
class PlanningResult:
    """
    Result returned by the planner.
    """

    success: bool

    state_path: List[int] = field(
        default_factory=list
    )

    transition_path: List[int] = field(
        default_factory=list
    )

    total_cost: float = 0.0

    safety_score: float = 0.0

    reliability_score: float = 0.0

    explored_states: int = 0

    planning_time: float = 0.0

    message: str = ""