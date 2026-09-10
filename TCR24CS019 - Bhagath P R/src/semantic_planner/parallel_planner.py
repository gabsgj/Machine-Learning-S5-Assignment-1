"""
Parallel Search Planner (Bonus Requirement).
Implements concurrent bidirectional search and multi-threaded parallel frontier expansion.
"""

from typing import Dict, List, Optional, Tuple, Set
import math
import heapq
import threading
from concurrent.futures import ThreadPoolExecutor
from .base_planner import Planner
from .models import PlanningProblem, PlanningResult, Transition
from .graph import CartesianGraph
from .cost_functions import CompositeCostEvaluator
from .heuristics import Heuristic, EuclideanHeuristic
from .utils import PerformanceTracker


class ParallelBidirectionalPlanner(Planner):
    """
    Parallel Bidirectional Heuristic Search Planner.
    Spawns concurrent forward (start -> goal) and backward (goal -> start) search threads
    communicating via shared memory and mutual meeting state detection.
    """

    def __init__(self, heuristic: Optional[Heuristic] = None, num_workers: int = 2):
        super().__init__(heuristic)
        self.num_workers = num_workers

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

        if self.start_id == self.goal_id:
            elapsed_ms, peak_kb = tracker.stop()
            res = self.cost_evaluator.evaluate_path([self.start_id], [], True)
            res.planning_time_ms = elapsed_ms
            res.memory_kb = peak_kb
            return res

        # Forward search structures
        g_f: Dict[int, float] = {self.start_id: 0.0}
        came_from_f: Dict[int, Tuple[int, int]] = {}
        open_f: List[Tuple[float, int]] = [(self.heuristic.estimate(self.start_id, self.goal_id), self.start_id)]
        closed_f: Set[int] = set()

        # Backward search structures
        g_b: Dict[int, float] = {self.goal_id: 0.0}
        came_from_b: Dict[int, Tuple[int, int]] = {}
        open_b: List[Tuple[float, int]] = [(self.heuristic.estimate(self.goal_id, self.start_id), self.goal_id)]
        closed_b: Set[int] = set()

        lock = threading.Lock()
        meeting_node: Optional[int] = None
        best_mu = float("inf")
        stop_event = threading.Event()
        explored_f = [0]
        explored_b = [0]

        def forward_worker():
            while open_f and not stop_event.is_set():
                with lock:
                    if not open_f:
                        break
                    f_val, current = heapq.heappop(open_f)
                    if current in closed_f:
                        continue
                    closed_f.add(current)
                    explored_f[0] += 1
                    g_curr = g_f[current]

                    # Check connection with backward closed set
                    if current in g_b:
                        total_cand = g_curr + g_b[current]
                        nonlocal meeting_node, best_mu
                        if total_cand < best_mu:
                            best_mu = total_cand
                            meeting_node = current
                            stop_event.set()
                            break

                for succ_id, trans in self.graph.get_successors(current, only_available=True):
                    if succ_id in closed_f:
                        continue
                    edge_c = self.cost_evaluator.get_edge_cost(current, succ_id, trans)
                    if math.isinf(edge_c):
                        continue

                    tentative_g = g_curr + edge_c
                    with lock:
                        if tentative_g < g_f.get(succ_id, float("inf")):
                            g_f[succ_id] = tentative_g
                            came_from_f[succ_id] = (current, trans.id)
                            h_val = self.heuristic.estimate(succ_id, self.goal_id)
                            heapq.heappush(open_f, (tentative_g + h_val, succ_id))

        def backward_worker():
            while open_b and not stop_event.is_set():
                with lock:
                    if not open_b:
                        break
                    f_val, current = heapq.heappop(open_b)
                    if current in closed_b:
                        continue
                    closed_b.add(current)
                    explored_b[0] += 1
                    g_curr = g_b[current]

                    if current in g_f:
                        total_cand = g_f[current] + g_curr
                        nonlocal meeting_node, best_mu
                        if total_cand < best_mu:
                            best_mu = total_cand
                            meeting_node = current
                            stop_event.set()
                            break

                # In backward search, traverse predecessors
                for pred_id, trans in self.graph.get_predecessors(current, only_available=True):
                    if pred_id in closed_b:
                        continue
                    edge_c = self.cost_evaluator.get_edge_cost(pred_id, current, trans)
                    if math.isinf(edge_c):
                        continue

                    tentative_g = g_curr + edge_c
                    with lock:
                        if tentative_g < g_b.get(pred_id, float("inf")):
                            g_b[pred_id] = tentative_g
                            came_from_b[pred_id] = (current, trans.id)
                            h_val = self.heuristic.estimate(pred_id, self.start_id)
                            heapq.heappush(open_b, (tentative_g + h_val, pred_id))

        t_f = threading.Thread(target=forward_worker)
        t_b = threading.Thread(target=backward_worker)

        t_f.start()
        t_b.start()
        t_f.join()
        t_b.join()

        success = meeting_node is not None
        state_path = []
        trans_path = []

        if success and meeting_node is not None:
            # Reconstruct forward segment: start -> meeting_node
            curr = meeting_node
            fwd_states = [curr]
            fwd_trans = []
            while curr != self.start_id and curr in came_from_f:
                prev, tid = came_from_f[curr]
                fwd_states.append(prev)
                fwd_trans.append(tid)
                curr = prev
            fwd_states.reverse()
            fwd_trans.reverse()

            # Reconstruct backward segment: meeting_node -> goal
            curr = meeting_node
            bwd_states = []
            bwd_trans = []
            while curr != self.goal_id and curr in came_from_b:
                nxt, tid = came_from_b[curr]
                bwd_states.append(nxt)
                bwd_trans.append(tid)
                curr = nxt

            state_path = fwd_states + bwd_states
            trans_path = fwd_trans + bwd_trans

        elapsed_ms, peak_kb = tracker.stop()
        self.explored_nodes_count = explored_f[0] + explored_b[0]

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
