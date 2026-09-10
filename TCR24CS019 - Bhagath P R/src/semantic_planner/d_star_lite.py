"""
D* Lite Incremental Planning Algorithm for Finite Cartesian State Space.
"""

from typing import Dict, List, Optional, Tuple, Set
import math
from .base_planner import Planner
from .models import PlanningProblem, PlanningResult, Transition
from .graph import CartesianGraph
from .cost_functions import CompositeCostEvaluator
from .heuristics import Heuristic, EuclideanHeuristic
from .utils import PriorityQueue, PerformanceTracker


class DStarLitePlanner(Planner):
    """
    D* Lite incremental heuristic planner (Koenig & Likhachev, 2002).
    Backward search from goal to start with dynamic key modifier.
    """

    def __init__(self, heuristic: Optional[Heuristic] = None):
        super().__init__(heuristic)
        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {}
        self.queue = PriorityQueue()
        self.km: float = 0.0
        self.last_start_id: int = -1
        self._initialized: bool = False

    def _calculate_key(self, u: int) -> Tuple[float, float]:
        gu = self.g.get(u, float("inf"))
        rhsu = self.rhs.get(u, float("inf"))
        min_val = min(gu, rhsu)
        if math.isinf(min_val):
            return (float("inf"), float("inf"))
        h_val = self.heuristic.estimate(self.start_id, u) if self.heuristic else 0.0
        return (min_val + h_val + self.km, min_val)

    def _update_vertex(self, u: int) -> None:
        if u != self.goal_id:
            min_rhs = float("inf")
            for succ_id, trans in self.graph.get_successors(u, only_available=True):
                cost = self.cost_evaluator.get_edge_cost(u, succ_id, trans)
                g_succ = self.g.get(succ_id, float("inf"))
                if not math.isinf(cost) and not math.isinf(g_succ):
                    cand = cost + g_succ
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
            start_k = self._calculate_key(self.start_id)
            start_g = self.g.get(self.start_id, float("inf"))
            start_rhs = self.rhs.get(self.start_id, float("inf"))

            if not (top_k < start_k or not math.isclose(start_rhs, start_g, abs_tol=1e-9)):
                break

            if self.queue.is_empty():
                break

            k_old = top_k
            u, _ = self.queue.pop()
            self.explored_nodes_count += 1
            k_new = self._calculate_key(u)

            if k_old < k_new:
                self.queue.insert(u, k_new)
            else:
                gu = self.g.get(u, float("inf"))
                rhsu = self.rhs.get(u, float("inf"))
                if gu > rhsu:
                    self.g[u] = rhsu
                    for pred_id, trans in self.graph.get_predecessors(u, only_available=False):
                        self._update_vertex(pred_id)
                else:
                    self.g[u] = float("inf")
                    self._update_vertex(u)
                    for pred_id, trans in self.graph.get_predecessors(u, only_available=False):
                        self._update_vertex(pred_id)

    def _extract_path(self) -> Tuple[List[int], List[int], bool]:
        start_g = self.g.get(self.start_id, float("inf"))
        start_rhs = self.rhs.get(self.start_id, float("inf"))
        if math.isinf(min(start_g, start_rhs)):
            return [], [], False

        state_path = [self.start_id]
        trans_path = []
        curr = self.start_id
        visited = {curr}

        while curr != self.goal_id:
            best_succ = None
            best_trans = None
            min_cost = float("inf")

            for succ_id, trans in self.graph.get_successors(curr, only_available=True):
                c = self.cost_evaluator.get_edge_cost(curr, succ_id, trans)
                g_succ = self.g.get(succ_id, float("inf"))
                if not math.isinf(c) and not math.isinf(g_succ):
                    cand = c + g_succ
                    if cand < min_cost:
                        min_cost = cand
                        best_succ = succ_id
                        best_trans = trans.id

            if best_succ is None or best_succ in visited or math.isinf(min_cost):
                return [], [], False

            visited.add(best_succ)
            state_path.append(best_succ)
            trans_path.append(best_trans)
            curr = best_succ

        return state_path, trans_path, True

    def initialize(self, problem: PlanningProblem) -> None:
        self.problem = problem
        self.graph = CartesianGraph.from_planning_problem(problem)
        self.cost_evaluator = CompositeCostEvaluator(self.graph, problem.weights)
        if self.heuristic is None:
            self.heuristic = EuclideanHeuristic(self.graph)

        self.start_id = problem.initial_state
        self.goal_id = problem.goal_state
        self.last_start_id = self.start_id
        self.km = 0.0

        self.g.clear()
        self.rhs.clear()
        self.queue.clear()
        self.explored_nodes_count = 0

        self.rhs[self.goal_id] = 0.0
        self.queue.insert(self.goal_id, self._calculate_key(self.goal_id))
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

    def update_start(self, new_start: int) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating start.")
        if self.heuristic:
            self.km += self.heuristic.estimate(self.last_start_id, new_start)
        self.last_start_id = new_start
        self.start_id = new_start

    def update_edge(self, u: int, v: int, new_cost: Optional[float] = None, available: Optional[bool] = None) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating edges.")

        if new_cost is not None:
            self.graph.update_transition_cost(u, v, new_cost)
        if available is not None:
            self.graph.set_transition_availability(u, v, available)

        self._update_vertex(u)

    def update_bad_states(self, added: Optional[List[int]] = None, removed: Optional[List[int]] = None) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating bad states.")

        affected_states: Set[int] = set()

        if added:
            for b in added:
                self.graph.add_bad_state(b)
                affected_states.add(b)
                for pred_id, _ in self.graph.get_predecessors(b, only_available=False):
                    affected_states.add(pred_id)
                for succ_id, _ in self.graph.get_successors(b, only_available=False):
                    affected_states.add(succ_id)

        if removed:
            for b in removed:
                self.graph.remove_bad_state(b)
                affected_states.add(b)
                for pred_id, _ in self.graph.get_predecessors(b, only_available=False):
                    affected_states.add(pred_id)
                for succ_id, _ in self.graph.get_successors(b, only_available=False):
                    affected_states.add(succ_id)

        for s in affected_states:
            self._update_vertex(s)

    def update_goal(self, new_goal: int) -> None:
        if not self._initialized:
            raise RuntimeError("Planner must be initialized before updating goal.")
        self.goal_id = new_goal
        self.g.clear()
        self.rhs.clear()
        self.queue.clear()
        self.km = 0.0
        self.last_start_id = self.start_id

        self.rhs[self.goal_id] = 0.0
        self.queue.insert(self.goal_id, self._calculate_key(self.goal_id))

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
