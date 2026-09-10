"""
Data models and interfaces for Safe Semantic Planning in Finite Cartesian State Spaces.
Matches the problem definition and software interfaces.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Optional, Any, Sequence
import numpy as np


@dataclass
class State:
    """Represents a state embedded in Cartesian space R^d."""
    id: int
    embedding: Sequence[float]
    name: Optional[str] = None

    def __post_init__(self):
        if not isinstance(self.embedding, np.ndarray):
            self.embedding = np.asarray(self.embedding, dtype=np.float64)

    @property
    def vector(self) -> np.ndarray:
        return self.embedding

    @property
    def dimension(self) -> int:
        return len(self.embedding)

    def distance_to(self, other: "State") -> float:
        """Euclidean distance in Cartesian state space R^d."""
        return float(np.linalg.norm(self.embedding - other.embedding))


@dataclass
class Transition:
    """Represents a directed transition between two states."""
    id: int
    from_state: int
    to_state: int
    cost: float = 1.0
    safety: float = 1.0
    reliability: float = 1.0
    available: bool = True
    metadata: Dict[str, Any] = field(default_factory=dict)

    def __post_init__(self):
        if self.reliability < 0.0 or self.reliability > 1.0:
            raise ValueError(f"Reliability must be in [0, 1], got {self.reliability}")
        if self.cost < 0.0:
            raise ValueError(f"Cost cannot be negative, got {self.cost}")


@dataclass
class ObjectiveWeights:
    """Weights for composite score: Score(P) = alpha*G - beta*C + gamma*D + delta*R."""
    alpha: float = 1000.0        # Goal completion reward
    beta: float = 1.0            # Cumulative transition cost weight
    gamma: float = 2.0           # Minimum safety distance weight
    delta: float = 1.0           # Cumulative reliability weight
    safety_margin: float = 1.5   # Threshold distance d_safe for repulsive safety barrier
    safety_penalty_coeff: float = 10.0  # Multiplier lambda for proximity barrier


@dataclass
class PlanningProblem:
    """Specification of a planning problem instance."""
    initial_state: int
    goal_state: int
    bad_states: List[int] = field(default_factory=list)
    states: List[State] = field(default_factory=list)
    transitions: List[Transition] = field(default_factory=list)
    weights: ObjectiveWeights = field(default_factory=ObjectiveWeights)

    def get_state(self, state_id: int) -> Optional[State]:
        for s in self.states:
            if s.id == state_id:
                return s
        return None


@dataclass
class PlanningResult:
    """Result returned by a planning algorithm."""
    success: bool
    state_path: List[int] = field(default_factory=list)
    transition_path: List[int] = field(default_factory=list)
    total_cost: float = 0.0
    safety_score: float = 0.0
    min_safety_distance: float = float("inf")
    cumulative_reliability: float = 1.0
    composite_score: float = 0.0
    explored_states: int = 0
    planning_time_ms: float = 0.0
    memory_kb: float = 0.0
    message: str = ""

    def summary(self) -> str:
        status = "SUCCESS" if self.success else "FAILED"
        path_str = " -> ".join(map(str, self.state_path)) if self.state_path else "None"
        return (
            f"PlanningResult({status}):\n"
            f"  State Path: {path_str}\n"
            f"  Transitions: {self.transition_path}\n"
            f"  Total Cost: {self.total_cost:.4f}\n"
            f"  Min Bad Distance: {self.min_safety_distance:.4f}\n"
            f"  Cumulative Reliability: {self.cumulative_reliability:.4f}\n"
            f"  Safety Score: {self.safety_score:.4f}\n"
            f"  Composite Score: {self.composite_score:.4f}\n"
            f"  Explored States: {self.explored_states}\n"
            f"  Planning Time: {self.planning_time_ms:.3f} ms\n"
            f"  Memory Usage: {self.memory_kb:.2f} KB"
        )
