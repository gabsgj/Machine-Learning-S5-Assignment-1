"""
Standard A* Planner baseline (searches from scratch upon updates).
"""

from typing import Dict, List, Optional, Tuple, Set
import math
import heapq
from .base_planner import Planner
from .models import PlanningProblem, PlanningResult, Transition
from .graph import CartesianGraph
from .cost_functions import CompositeCostEvaluator
from .heuristics import Heuristic, EuclideanHeuristic
from .utils import PerformanceTracker


class AStarPlanner(Planner):
    """
    Classical A* Search Planner (static baseline).
    Re-executes search from scratch upon any environment update.
    """

    def __init__(self, heuristic: Optional[Heuristic] = None):
        super().__init__(heuristic)

    def plan(self, problem: PlanningProblem) -> PlanningResult:
        tracker = PerformanceTracker()
        tracker.start()

        self.problem = problem
        self.graph = CartesianGraph.from_planning_problem(problem)
        self.cost_evaluator = CompositeCostEvaluator(self.graph, problem.weights)
        if self.heuristic is None:
            self.heuristic = EuclideanHeuristic(self.graph)

        self.start_id = problem.initial_state
        self.goal_id = problem.goal_state
        self.explored_nodes_count = 0

        g_score: Dict[int, float] = {self.start_id: 0.0}
        came_from: Dict[int, Tuple[int, int]] = {}
        
        h_start = self.heuristic.estimate(self.start_id, self.goal_id)
        open_set: List[Tuple[float, int]] = [(h_start, self.start_id)]
        closed_set: Set[int] = set()

        success = False
        while open_set:
            f, current = heapq.heappop(open_set)
            if current in closed_set:
                continue

            closed_set.add(current)
            self.explored_nodes_count += 1

            if current == self.goal_id:
                success = True
                break

            g_curr = g_score[current]
            for succ_id, trans in self.graph.get_successors(current, only_available=True):
                if succ_id in closed_set:
                    continue

                edge_c = self.cost_evaluator.get_edge_cost(current, succ_id, trans)
                if math.isinf(edge_c):
                    continue

                tentative_g = g_curr + edge_c
                if tentative_g < g_score.get(succ_id, float("inf")):
                    g_score[succ_id] = tentative_g
                    came_from[succ_id] = (current, trans.id)
                    h_val = self.heuristic.estimate(succ_id, self.goal_id)
                    heapq.heappush(open_set, (tentative_g + h_val, succ_id))

        state_path = []
        trans_path = []
        if success:
            curr = self.goal_id
            state_path = [curr]
            while curr != self.start_id:
                prev, tid = came_from[curr]
                state_path.append(prev)
                trans_path.append(tid)
                curr = prev
            state_path.reverse()
            trans_path.reverse()

        elapsed_ms, peak_kb = tracker.stop()
        res = self.cost_evaluator.evaluate_path(state_path, trans_path, success)
        res.explored_states = self.explored_nodes_count
        res.planning_time_ms = elapsed_ms
        res.memory_kb = peak_kb
        return res

    def update_edge(self, u: int, v: int, new_cost: Optional[float] = None, available: Optional[bool] = None) -> None:
        if new_cost is not None:
            self.graph.update_transition_cost(u, v, new_cost)
        if available is not None:
            self.graph.set_transition_availability(u, v, available)

    def update_bad_states(self, added: Optional[List[int]] = None, removed: Optional[List[int]] = None) -> None:
        if added:
            for b in added:
                self.graph.add_bad_state(b)
        if removed:
            for b in removed:
                self.graph.remove_bad_state(b)

    def update_goal(self, new_goal: int) -> None:
        self.goal_id = new_goal

    def replan(self) -> PlanningResult:
        prob = PlanningProblem(
            initial_state=self.start_id,
            goal_state=self.goal_id,
            bad_states=list(self.graph.bad_states),
            states=list(self.graph.states.values()),
            transitions=list(self.graph.transitions.values()),
            weights=self.cost_evaluator.weights
        )
        return self.plan(prob)
