"""
Base Planner abstract class and interface.
"""

from abc import ABC, abstractmethod
from typing import Optional, List, Set
from .models import PlanningProblem, PlanningResult
from .graph import CartesianGraph
from .cost_functions import CompositeCostEvaluator
from .heuristics import Heuristic


class Planner(ABC):
    """
    Abstract interface for planners.
    Conforms to the assignment interface:
      virtual PlanningResult plan(const PlanningProblem& problem) = 0;
    """

    def __init__(self, heuristic: Optional[Heuristic] = None):
        self.heuristic = heuristic
        self.graph: Optional[CartesianGraph] = None
        self.cost_evaluator: Optional[CompositeCostEvaluator] = None
        self.problem: Optional[PlanningProblem] = None
        self.start_id: int = -1
        self.goal_id: int = -1
        self.explored_nodes_count: int = 0

    @abstractmethod
    def plan(self, problem: PlanningProblem) -> PlanningResult:
        """Computes a safe path from initial_state to goal_state."""
        pass

    @abstractmethod
    def update_edge(self, u: int, v: int, new_cost: Optional[float] = None, available: Optional[bool] = None) -> None:
        """Informs the planner that edge (u, v) cost or availability has changed."""
        pass

    @abstractmethod
    def update_bad_states(self, added: Optional[List[int]] = None, removed: Optional[List[int]] = None) -> None:
        """Informs the planner of updates to the bad states set."""
        pass

    @abstractmethod
    def update_goal(self, new_goal: int) -> None:
        """Updates the goal state."""
        pass

    @abstractmethod
    def replan(self) -> PlanningResult:
        """Efficiently re-computes a path after dynamic updates."""
        pass
