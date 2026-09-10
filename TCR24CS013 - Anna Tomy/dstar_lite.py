"""
dstar_lite.py
A safety- and reliability-aware D* Lite planner.
"""

import heapq
import time
from typing import Dict, List, Tuple, Optional

from models import State, Transition, PlanningProblem, PlanningResult
from planner import Planner

INF = float("inf")


class DStarLite(Planner):
    def __init__(self, safety_weight: float = 5.0, reliability_weight: float = 2.0):
        self.safety_weight = safety_weight
        self.reliability_weight = reliability_weight

        self.states: Dict[int, State] = {}
        self.successors: Dict[int, List[int]] = {}
        self.predecessors: Dict[int, List[int]] = {}
        self.edge: Dict[Tuple[int, int], Transition] = {}
        self.bad_states: set = set()

        self.goal: Optional[int] = None
        self.start: Optional[int] = None

        # D* Lite bookkeeping
        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {}
        self.U: List[Tuple[Tuple[float, float], int]] = []
        self.km: float = 0.0
        self.explored_count: int = 0
        self.last_start: Optional[int] = None

    # ---------- graph construction ----------

    def _build_graph(self, problem: PlanningProblem):
        self.states = {s.id: s for s in problem.states}
        self.bad_states = set(problem.bad_states)
        self.successors = {sid: [] for sid in self.states}
        self.predecessors = {sid: [] for sid in self.states}
        self.edge = {}

        for t in problem.transitions:
            self.edge[(t.from_id, t.to_id)] = t
            if t.available and t.to_id not in self.bad_states:
                self.successors[t.from_id].append(t.to_id)
                self.predecessors[t.to_id].append(t.from_id)

        self.start = problem.initial_state
        self.goal = problem.goal_state

    # ---------- heuristic and cost ----------

    def _heuristic(self, a: int, b: int) -> float:
        return self.states[a].distance_to(self.states[b])

    def _edge_cost(self, u: int, v: int) -> float:
        t = self.edge[(u, v)]
        safety_penalty = self.safety_weight / (t.safety + 1e-6)
        reliability_penalty = self.reliability_weight * (1.0 - t.reliability)
        return t.cost + safety_penalty + reliability_penalty

    def _min_dist_to_bad(self, sid: int) -> float:
        """Euclidean distance from state sid to the nearest bad state."""
        if not self.bad_states:
            return INF
        s = self.states[sid]
        return min(s.distance_to(self.states[b]) for b in self.bad_states)

    # ---------- D* Lite core ----------

    def _calculate_key(self, s: int) -> Tuple[float, float]:
        m = min(self.g[s], self.rhs[s])
        return (m + self._heuristic(self.start, s) + self.km, m)

    def _update_vertex(self, u: int):
        if u != self.goal:
            best = INF
            for v in self.successors[u]:
                cand = self._edge_cost(u, v) + self.g[v]
                if cand < best:
                    best = cand
            self.rhs[u] = best

        self.U = [(k, s) for (k, s) in self.U if s != u]
        heapq.heapify(self.U)

        if self.g[u] != self.rhs[u]:
            heapq.heappush(self.U, (self._calculate_key(u), u))

    def _compute_shortest_path(self):
        while self.U and (
            self.U[0][0] < self._calculate_key(self.start)
            or self.rhs[self.start] != self.g[self.start]
        ):
            k_old, u = heapq.heappop(self.U)
            self.explored_count += 1
            k_new = self._calculate_key(u)

            if k_old < k_new:
                heapq.heappush(self.U, (k_new, u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.predecessors[u]:
                    self._update_vertex(s)
            else:
                self.g[u] = INF
                self._update_vertex(u)
                for s in self.predecessors[u]:
                    self._update_vertex(s)

    # ---------- path extraction ----------

    def _extract_path(self) -> Optional[Tuple[List[int], List[int]]]:
        """Greedily follow the cheapest successor (by edge_cost + g) from start to goal."""
        path = [self.start]
        transition_path = []
        current = self.start
        visited = {current}

        while current != self.goal:
            best_v, best_val = None, INF
            for v in self.successors[current]:
                val = self._edge_cost(current, v) + self.g[v]
                if val < best_val:
                    best_val, best_v = val, v

            if best_v is None or best_v in visited or self.g[best_v] == INF:
                return None  # stuck in a loop or dead end -> no valid path

            transition_path.append(self.edge[(current, best_v)].id)
            current = best_v
            path.append(current)
            visited.add(current)

        return path, transition_path

    def _package_result(self, elapsed: float) -> PlanningResult:
        """Turn the current g/rhs state into a PlanningResult (path, cost, safety, etc.)."""
        if self.g[self.start] == INF:
            return PlanningResult(
                success=False,
                explored_states=self.explored_count,
                planning_time=elapsed,
            )

        extracted = self._extract_path()
        if extracted is None:
            return PlanningResult(
                success=False,
                explored_states=self.explored_count,
                planning_time=elapsed,
            )

        path, transition_path = extracted
        total_cost = sum(self.edge[(path[i], path[i + 1])].cost for i in range(len(path) - 1))
        safety_score = min((self._min_dist_to_bad(sid) for sid in path), default=INF)

        return PlanningResult(
            success=True,
            state_path=path,
            transition_path=transition_path,
            total_cost=total_cost,
            safety_score=safety_score,
            explored_states=self.explored_count,
            planning_time=elapsed,
        )

    def update_transition(
        self,
        from_id: int,
        to_id: int,
        available: Optional[bool] = None,
        cost: Optional[float] = None,
        safety: Optional[float] = None,
        reliability: Optional[float] = None,
    ) -> PlanningResult:
        """
        Apply a change to an existing edge (from_id -> to_id) WITHOUT resetting
        g/rhs for the whole graph, then re-run only the affected part of the search.
        Call this only after plan() has already been run once.
        """
        start_time = time.perf_counter()

        # 1. Update km using the drift between the last known start and current start
        if self.last_start is not None:
            self.km += self._heuristic(self.last_start, self.start)
        self.last_start = self.start

        # 2. Apply the raw data change to the Transition object
        t = self.edge[(from_id, to_id)]
        if available is not None:
            t.available = available
        if cost is not None:
            t.cost = cost
        if safety is not None:
            t.safety = safety
        if reliability is not None:
            t.reliability = reliability

        # 3. Rebuild successors/predecessors for just this one edge
        is_usable = t.available and to_id not in self.bad_states
        if is_usable and to_id not in self.successors[from_id]:
            self.successors[from_id].append(to_id)
            self.predecessors[to_id].append(from_id)
        elif not is_usable and to_id in self.successors[from_id]:
            self.successors[from_id].remove(to_id)
            self.predecessors[to_id].remove(from_id)

        # 4. Mark the affected vertex (from_id) as needing a recheck.
        #    Its rhs depended on this edge, so it may now be stale.
        self._update_vertex(from_id)

        # 5. Re-run the search loop. Since g/rhs were NOT reset, this only
        #    processes states that actually became inconsistent -- the ripple.
        self._compute_shortest_path()

        elapsed = time.perf_counter() - start_time
        return self._package_result(elapsed)

    def add_transition(self, transition: Transition) -> PlanningResult:
        """Insert a brand new transition into the graph, then replan incrementally."""
        self.edge[(transition.from_id, transition.to_id)] = transition
        if transition.from_id not in self.successors:
            self.successors[transition.from_id] = []
        if transition.to_id not in self.predecessors:
            self.predecessors[transition.to_id] = []
        return self.update_transition(transition.from_id, transition.to_id)

    def set_goal(self, new_goal: int, problem: PlanningProblem) -> PlanningResult:
        """
        Change the goal state. D* Lite assumes a fixed goal, so a goal change
        requires a full reinitialization -- this is a known limitation, discussed
        in the report. We still keep the same graph-building code path.
        """
        problem.goal_state = new_goal
        return self.plan(problem)

    # ---------- public interface ----------

    def plan(self, problem: PlanningProblem) -> PlanningResult:
        start_time = time.perf_counter()

        self._build_graph(problem)
        self.g = {sid: INF for sid in self.states}
        self.rhs = {sid: INF for sid in self.states}
        self.rhs[self.goal] = 0.0
        self.km = 0.0
        self.U = []
        self.explored_count = 0
        self.last_start = self.start  # remember start position, needed for km updates later

        heapq.heappush(self.U, (self._calculate_key(self.goal), self.goal))
        self._compute_shortest_path()

        elapsed = time.perf_counter() - start_time
        return self._package_result(elapsed)