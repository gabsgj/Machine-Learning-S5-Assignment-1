"""
Cost function evaluators, safety clearance barriers, and objective score computation.
"""

import math
from typing import List, Optional
from .models import Transition, ObjectiveWeights, PlanningResult
from .graph import CartesianGraph


class CompositeCostEvaluator:
    """
    Evaluates edge costs taking into account:
    1. Base transition cost
    2. Hard obstacle avoidance (infinite cost for bad states)
    3. Soft safety margin (repulsive barrier penalty when approaching bad states)
    4. Transition reliability (additive logarithmic penalty -ln(r))
    5. Availability flags
    """

    def __init__(self, graph: CartesianGraph, weights: Optional[ObjectiveWeights] = None):
        self.graph = graph
        self.weights = weights or ObjectiveWeights()

    def get_edge_cost(self, u: int, v: int, transition: Optional[Transition] = None) -> float:
        """
        Computes the effective edge cost c_eff(u, v).
        Returns float('inf') if edge is unavailable or traverses a bad state.
        """
        if self.graph.is_bad(v) or self.graph.is_bad(u):
            return float("inf")

        if transition is None:
            transition = self.graph.get_transition(u, v)

        if transition is None or not transition.available:
            return float("inf")

        base_cost = self.weights.beta * transition.cost

        # Soft safety barrier penalty
        safety_penalty = 0.0
        min_dist = self.graph.min_distance_to_bad_states(v)
        if min_dist < self.weights.safety_margin:
            margin_diff = self.weights.safety_margin - min_dist
            safety_penalty = self.weights.safety_penalty_coeff * (margin_diff ** 2)

        # Reliability penalty: -ln(rel)
        rel = max(transition.reliability, 1e-6)
        rel_penalty = -self.weights.delta * math.log(rel)

        return base_cost + safety_penalty + rel_penalty

    def evaluate_path(self, state_path: List[int], transition_path: List[int], success: bool) -> PlanningResult:
        """
        Calculates all performance metrics and the objective function score for a path:
        Score(P) = alpha*G - beta*C + gamma*D + delta*R
        """
        if not success or not state_path:
            return PlanningResult(
                success=False,
                state_path=[],
                transition_path=[],
                total_cost=float("inf"),
                safety_score=0.0,
                min_safety_distance=0.0,
                cumulative_reliability=0.0,
                composite_score=-float("inf"),
                message="No valid safe path found."
            )

        raw_cost = 0.0
        cumulative_rel = 1.0
        for tid in transition_path:
            trans = self.graph.transitions.get(tid)
            if trans:
                raw_cost += trans.cost
                cumulative_rel *= trans.reliability

        min_bad_dist = float("inf")
        has_bad_visited = False
        for sid in state_path:
            if self.graph.is_bad(sid):
                has_bad_visited = True
                min_bad_dist = 0.0
                break
            d = self.graph.min_distance_to_bad_states(sid)
            if d < min_bad_dist:
                min_bad_dist = d

        if math.isinf(min_bad_dist):
            min_bad_dist = 100.0

        if has_bad_visited:
            return PlanningResult(
                success=False,
                state_path=state_path,
                transition_path=transition_path,
                total_cost=raw_cost,
                safety_score=0.0,
                min_safety_distance=0.0,
                cumulative_reliability=cumulative_rel,
                composite_score=-float("inf"),
                message="Violated safety: Visited bad state!"
            )

        goal_completion = 1.0 if success else 0.0
        score = (
            self.weights.alpha * goal_completion
            - self.weights.beta * raw_cost
            + self.weights.gamma * min_bad_dist
            + self.weights.delta * cumulative_rel
        )

        return PlanningResult(
            success=True,
            state_path=state_path,
            transition_path=transition_path,
            total_cost=raw_cost,
            safety_score=min_bad_dist,
            min_safety_distance=min_bad_dist,
            cumulative_reliability=cumulative_rel,
            composite_score=score,
            message="Valid safe path computed successfully."
        )
