"""
Lifelong Planning A* (LPA*) Implementation for Finite Cartesian State Space.
"""

from typing import Dict, List, Optional, Tuple, Set
import math
from .base_planner import Planner
from .models import PlanningProblem, PlanningResult, Transition
from .graph import CartesianGraph
from .cost_functions import CompositeCostEvaluator
from .heuristics import Heuristic, EuclideanHeuristic
from .utils import PriorityQueue, PerformanceTracker


class LPAStarPlanner(Planner):
    """
    Lifelong Planning A* (LPA*) incremental forward search planner.
    Efficiently repairs shortest path trees upon edge/obstacle changes.
    """

    def __init__(self, heuristic: Optional[Heuristic] = None):
        super().__init__(heuristic)
        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {}
        self.queue = PriorityQueue()
        self._initialized = False

    def _calculate_key(self, u: int) -> Tuple[float, float]:
        gu = self.g.get(u, float("inf"))
        rhsu = self.rhs.get(u, float("inf"))
        min_val = min(gu, rhsu)
        if math.isinf(min_val):
            return (float("inf"), float("inf"))
        h_val = self.heuristic.estimate(u, self.goal_id) if self.heuristic else 0.0
        return (min_val + h_val, min_val)

    def _update_vertex(self, u: int) -> None:
        if u != self.start_id:
            min_rhs = float("inf")
            for pred_id, trans in self.graph.get_predecessors(u, only_available=True):
                cost = self.cost_evaluator.get_edge_cost(pred_id, u, trans)
                g_pred = self.g.get(pred_id, float("inf"))
                if not math.isinf(cost) and not math.isinf(g_pred):
                    cand = g_pred + cost
                    if cand < min_rhs:
                        min_rhs = cand
            self.rhs[u] = min_rhs

        if self.queue.contains(u):
            self.queue.remove(u)

        gu = self.g.get(u, float("inf"))
        rhsu = self.rhs.get(u, float("inf"))
        if not math.isclose(gu, rhsu, abs_tol=1e-9):
            self.queue.insert(u, self._calculate_key(u))

    def _compute_shortest_path(self) -> None:
        while True:
            top_k = self.queue.top_key()
            goal_k = self._calculate_key(self.goal_id)
            goal_g = self.g.get(self.goal_id, float("inf"))
            goal_rhs = self.rhs.get(self.goal_id, float("inf"))

            if not (top_k < goal_k or not math.isclose(goal_rhs, goal_g, abs_tol=1e-9)):
                break

            if self.queue.is_empty():
                break

            u, _ = self.queue.pop()
            self.explored_nodes_count += 1
            gu = self.g.get(u, float("inf"))
            rhsu = self.rhs.get(u, float("inf"))

            if gu > rhsu:
                self.g[u] = rhsu
                for succ_id, trans in self.graph.get_successors(u, only_available=False):
                    self._update_vertex(succ_id)
            else:
                self.g[u] = float("inf")
                self._update_vertex(u)
                for succ_id, trans in self.graph.get_successors(u, only_available=False):
                    self._update_vertex(succ_id)

    def _extract_path(self) -> Tuple[List[int], List[int], bool]:
        goal_g = self.g.get(self.goal_id, float("inf"))
        if math.isinf(goal_g):
            return [], [], False

        state_path = [self.goal_id]
        trans_path = []
        curr = self.goal_id
        visited = {curr}

        while curr != self.start_id:
            best_pred = None
            best_trans = None
            min_cost = float("inf")

            for pred_id, trans in self.graph.get_predecessors(curr, only_available=True):
                cost = self.cost_evaluator.get_edge_cost(pred_id, curr, trans)
                g_pred = self.g.get(pred_id, float("inf"))
                if not math.isinf(cost) and not math.isinf(g_pred):
                    cand = g_pred + cost
                    if cand < min_cost:
                        min_cost = cand
                        best_pred = pred_id
                        best_trans = trans.id

            if best_pred is None or best_pred in visited or math.isinf(min_cost):
                return [], [], False

            visited.add(best_pred)
            state_path.append(best_pred)
            trans_path.append(best_trans)
            curr = best_pred

        state_path.reverse()
        trans_path.reverse()
        return state_path, trans_path, True

    def initialize(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.graph = CartesianGraph.from_planning_problem(problem)
        self.cost_evaluator = CompositeCostEvaluator(self.graph, problem.weights)
        if self.heuristic is None:
            self.heuristic = EuclideanHeuristic(self.graph)

        self.start_id = problem.initial_state
        self.goal_id = problem.goal_state

        self.g.clear()
        self.rhs.clear()
        self.queue.clear()
        self.explored_nodes_count = 0

        self.rhs[self.start_id] = 0.0
        self.queue.insert(self.start_id, self._calculate_key(self.start_id))
        self._initialized = True

    def plan(self, problem: PlanningProblem) -> PlanningResult:
        tracker = PerformanceTracker()
        tracker.start()

        self.initialize(problem)
        self._compute_shortest_path()
        state_path, trans_path, success = self._extract_path()

        elapsed_ms, peak_kb = tracker.stop()

        res = self.cost_evaluator.evaluate_path(state_path, trans_path, success)
        res.explored_states = self.explored_nodes_count
        res.planning_time_ms = elapsed_ms
        res.memory_kb = peak_kb
        return res

    def update_edge(self, u: int, v: int, new_cost: Optional[float] = None, available: Optional[bool] = None) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating edges.")

        if new_cost is not None:
            self.graph.update_transition_cost(u, v, new_cost)
        if available is not None:
            self.graph.set_transition_availability(u, v, available)

        self._update_vertex(v)

    def update_bad_states(self, added: Optional[List[int]] = None, removed: Optional[List[int]] = None) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating bad states.")

        affected_states: Set[int] = set()

        if added:
            for b in added:
                self.graph.add_bad_state(b)
                affected_states.add(b)
                for succ_id, _ in self.graph.get_successors(b, only_available=False):
                    affected_states.add(succ_id)
                for pred_id, _ in self.graph.get_predecessors(b, only_available=False):
                    affected_states.add(pred_id)

        if removed:
            for b in removed:
                self.graph.remove_bad_state(b)
                affected_states.add(b)
                for succ_id, _ in self.graph.get_successors(b, only_available=False):
                    affected_states.add(succ_id)
                for pred_id, _ in self.graph.get_predecessors(b, only_available=False):
                    affected_states.add(pred_id)

        for s in affected_states:
            self._update_vertex(s)

    def update_goal(self, new_goal: int) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating goal.")
        old_goal = self.goal_id
        self.goal_id = new_goal

        items = []
        while not self.queue.is_empty():
            u, _ = self.queue.pop()
            items.append(u)
        for u in items:
            self.queue.insert(u, self._calculate_key(u))

        self._update_vertex(old_goal)
        self._update_vertex(new_goal)

    def replan(self) -> PlanningResult:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before replanning.")

        tracker = PerformanceTracker()
        tracker.start()
        nodes_before = self.explored_nodes_count

        self._compute_shortest_path()
        state_path, trans_path, success = self._extract_path()

        elapsed_ms, peak_kb = tracker.stop()
        explored_in_replan = self.explored_nodes_count - nodes_before

        res = self.cost_evaluator.evaluate_path(state_path, trans_path, success)
        res.explored_states = explored_in_replan
        res.planning_time_ms = elapsed_ms
        res.memory_kb = peak_kb
        return res
