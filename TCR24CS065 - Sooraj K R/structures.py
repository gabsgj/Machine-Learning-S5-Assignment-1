# structures.py
# Simple data classes that hold all the info for our planning problem.
# These match the interfaces from the assignment PDF.

from dataclasses import dataclass, field


@dataclass
class State:
    """A state in the Cartesian space. Has an ID and a position (embedding)."""
    id: int
    embedding: list  # list of floats, like [x, y] for 2D


@dataclass
class Transition:
    """A directed edge between two states, with cost, safety, reliability, and availability."""
    id: int
    from_state: int
    to_state: int
    cost: float
    safety: float
    reliability: float
    available: bool


@dataclass
class PlanningProblem:
    """Everything the planner needs to find a path."""
    initialState: int
    goalState: int
    badStates: list       # list of state IDs to avoid
    states: list          # list of State objects
    transitions: list     # list of Transition objects


@dataclass
class PlanningResult:
    """What the planner returns after finding (or not finding) a path."""
    success: bool
    statePath: list       # list of state IDs in order
    transitionPath: list  # list of transition IDs in order
    totalCost: float
    safetyScore: float    # min distance to nearest bad state
    # Extra evaluation metrics
    exploredStates: int = 0
    planningTime: float = 0.0      # in seconds
    reliabilityScore: float = 0.0  # cumulative reliability
    score: float = 0.0             # the objective function score