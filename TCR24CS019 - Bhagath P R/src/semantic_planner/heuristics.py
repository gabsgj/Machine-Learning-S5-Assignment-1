"""
Heuristic functions for Cartesian state space planning.
Includes Euclidean distance heuristic with admissibility calibration and ZeroHeuristic.
"""

from abc import ABC, abstractmethod
from typing import Optional
import numpy as np
from .graph import CartesianGraph


class Heuristic(ABC):
    """Abstract base class for heuristic estimators."""

    @abstractmethod
    def estimate(self, current_id: int, goal_id: int) -> float:
        """Estimates cost from current state to goal state."""
        pass


class EuclideanHeuristic(Heuristic):
    """
    Euclidean distance heuristic in Cartesian space R^d.
    h(u, v) = scale * ||x_u - x_v||_2
    Guaranteed admissible and consistent when scale <= min_{e=(u,v)} (cost(e) / ||x_u - x_v||).
    """

    def __init__(self, graph: CartesianGraph, scale: Optional[float] = None):
        self.graph = graph
        if scale is not None:
            self.scale = scale
        else:
            self.scale = self._compute_admissible_scale()

    def _compute_admissible_scale(self) -> float:
        """Calibrates scale factor to ensure triangle inequality and admissibility."""
        min_ratio = 1.0
        for t in self.graph.transitions.values():
            dist = self.graph.euclidean_distance(t.from_state, t.to_state)
            if dist > 1e-9 and not np.isinf(dist):
                ratio = t.cost / dist
                if ratio < min_ratio:
                    min_ratio = ratio
        return max(0.0, min_ratio)

    def estimate(self, current_id: int, goal_id: int) -> float:
        if current_id == goal_id:
            return 0.0
        d = self.graph.euclidean_distance(current_id, goal_id)
        if np.isinf(d):
            return 0.0
        return self.scale * d


class ZeroHeuristic(Heuristic):
    """Zero heuristic (Dijkstra-equivalent). Always admissible and consistent."""

    def __init__(self, graph: CartesianGraph):
        self.graph = graph

    def estimate(self, current_id: int, goal_id: int) -> float:
        return 0.0
