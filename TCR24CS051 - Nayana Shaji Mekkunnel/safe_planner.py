"""Safe dynamic planning in a finite Cartesian state space using D* Lite.

The planner always excludes bad states.  Its path selection minimizes an
adjusted edge cost that can trade raw cost against state clearance, transition
safety and reliability.  With the optional weights set to zero it is ordinary
least-cost D* Lite.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import heapq
import itertools
import math
import time
from typing import Dict, Iterable, List, Optional, Set, Tuple


INF = float("inf")


@dataclass(frozen=True)
class State:
    id: int
    embedding: Tuple[float, ...]


@dataclass
class Transition:
    id: int
    source: int
    target: int
    cost: float
    safety: float = 1.0
    reliability: float = 1.0
    available: bool = True


@dataclass
class PlanningProblem:
    initial_state: int
    goal_state: int
    states: Iterable[State]
    transitions: Iterable[Transition]
    bad_states: Set[int] = field(default_factory=set)


@dataclass
class PlanningResult:
    success: bool
    state_path: List[int] = field(default_factory=list)
    transition_path: List[int] = field(default_factory=list)
    total_cost: float = INF
    minimum_safety_distance: float = INF
    cumulative_reliability: float = 0.0
    explored_states: int = 0
    planning_time_ms: float = 0.0
    message: str = ""


class SafeDStarLitePlanner:
    """Incremental D* Lite planner for directed graphs.

    ``safety_weight`` is the penalty applied to reciprocal clearance from bad
    states. ``transition_safety_weight`` penalizes transitions with a safety
    score below one, while ``reliability_weight`` rewards reliable links.
    Keeping all three at zero produces a pure minimum-cost solution.
    """

    def __init__(
        self,
        problem: PlanningProblem,
        *,
        safety_weight: float = 0.0,
        transition_safety_weight: float = 0.0,
        reliability_weight: float = 0.0,
    ) -> None:
        self.safety_weight = safety_weight
        self.transition_safety_weight = transition_safety_weight
        self.reliability_weight = reliability_weight
        self._load_problem(problem)
        self._initialize()

    def _load_problem(self, problem: PlanningProblem) -> None:
        self.states: Dict[int, State] = {state.id: state for state in problem.states}
        self.edges: Dict[int, Transition] = {edge.id: edge for edge in problem.transitions}
        self.start, self.goal = problem.initial_state, problem.goal_state
        self.bad: Set[int] = set(problem.bad_states)
        self.succ: Dict[int, List[int]] = {state_id: [] for state_id in self.states}
        self.pred: Dict[int, List[int]] = {state_id: [] for state_id in self.states}
        for edge in self.edges.values():
            self._validate_edge(edge)
            self.succ[edge.source].append(edge.id)
            self.pred[edge.target].append(edge.id)
        self._validate_state(self.start)
        self._validate_state(self.goal)

    def _validate_state(self, state_id: int) -> None:
        if state_id not in self.states:
            raise ValueError(f"Unknown state: {state_id}")

    def _validate_edge(self, edge: Transition) -> None:
        self._validate_state(edge.source)
        self._validate_state(edge.target)
        if edge.cost < 0:
            raise ValueError("D* Lite requires non-negative transition costs")

    def _initialize(self) -> None:
        self.g = {state_id: INF for state_id in self.states}
        self.rhs = {state_id: INF for state_id in self.states}
        self.km = 0.0
        self.last_start = self.start
        self.queue: List[Tuple[float, float, int, int]] = []
        self._counter = itertools.count()
        self.explored = 0
        if self.goal not in self.bad:
            self.rhs[self.goal] = 0.0
            self._push(self.goal)

    def _distance(self, left: int, right: int) -> float:
        a, b = self.states[left].embedding, self.states[right].embedding
        if len(a) != len(b):
            raise ValueError("All state embeddings must have the same dimension")
        return math.dist(a, b)

    def _clearance(self, state_id: int) -> float:
        if not self.bad:
            return INF
        return min(self._distance(state_id, bad) for bad in self.bad)

    def _edge_cost(self, edge: Transition) -> float:
        if not edge.available or edge.source in self.bad or edge.target in self.bad:
            return INF
        clearance = self._clearance(edge.target)
        clearance_penalty = 0.0 if math.isinf(clearance) else self.safety_weight / max(clearance, 1e-9)
        transition_penalty = self.transition_safety_weight * max(0.0, 1.0 - edge.safety)
        reliability_bonus = self.reliability_weight * max(0.0, edge.reliability)
        return max(0.0, edge.cost + clearance_penalty + transition_penalty - reliability_bonus)

    def _key(self, state_id: int) -> Tuple[float, float]:
        best = min(self.g[state_id], self.rhs[state_id])
        return (best + self._distance(self.start, state_id) + self.km, best)

    def _push(self, state_id: int) -> None:
        key = self._key(state_id)
        heapq.heappush(self.queue, (*key, next(self._counter), state_id))

    def _pop_valid(self) -> Optional[Tuple[Tuple[float, float], int]]:
        while self.queue:
            k1, k2, _, state_id = heapq.heappop(self.queue)
            current = self._key(state_id)
            if (k1, k2) > current:
                self._push(state_id)
            elif (k1, k2) == current:
                return (k1, k2), state_id
        return None

    def _top_key(self) -> Tuple[float, float]:
        while self.queue:
            k1, k2, _, state_id = self.queue[0]
            current = self._key(state_id)
            if (k1, k2) > current:
                heapq.heappop(self.queue)
                self._push(state_id)
                continue
            return (k1, k2)
        return (INF, INF)

    def _update_vertex(self, state_id: int) -> None:
        if state_id != self.goal:
            self.rhs[state_id] = min(
                (self._edge_cost(self.edges[eid]) + self.g[self.edges[eid].target] for eid in self.succ[state_id]),
                default=INF,
            )
        if self.g[state_id] != self.rhs[state_id]:
            self._push(state_id)

    def _compute_shortest_path(self) -> None:
        while self._top_key() < self._key(self.start) or self.rhs[self.start] != self.g[self.start]:
            item = self._pop_valid()
            if item is None:
                return
            old_key, state_id = item
            self.explored += 1
            if old_key < self._key(state_id):
                self._push(state_id)
            elif self.g[state_id] > self.rhs[state_id]:
                self.g[state_id] = self.rhs[state_id]
                for eid in self.pred[state_id]:
                    self._update_vertex(self.edges[eid].source)
            else:
                self.g[state_id] = INF
                self._update_vertex(state_id)
                for eid in self.pred[state_id]:
                    self._update_vertex(self.edges[eid].source)

    def plan(self, current_state: Optional[int] = None) -> PlanningResult:
        """Plan from ``current_state`` (or the configured initial state)."""
        begun = time.perf_counter()
        if current_state is not None:
            self.move_start(current_state)
        self.explored = 0
        if self.start in self.bad or self.goal in self.bad:
            return PlanningResult(False, planning_time_ms=(time.perf_counter() - begun) * 1000,
                                  message="Initial or goal state is marked bad.")
        self._compute_shortest_path()
        result = self._extract_path()
        result.explored_states = self.explored
        result.planning_time_ms = (time.perf_counter() - begun) * 1000
        return result

    def _extract_path(self) -> PlanningResult:
        if math.isinf(self.g[self.start]):
            return PlanningResult(False, message="No safe path to the goal exists.")
        current, seen = self.start, {self.start}
        states, edge_ids, raw_cost, reliability = [current], [], 0.0, 0.0
        while current != self.goal:
            choices = [(self._edge_cost(self.edges[eid]) + self.g[self.edges[eid].target], eid)
                       for eid in self.succ[current]]
            value, eid = min(choices, default=(INF, -1))
            if math.isinf(value) or eid < 0:
                return PlanningResult(False, message="No safe successor can reach the goal.")
            edge = self.edges[eid]
            if edge.target in seen:
                return PlanningResult(False, message="Cycle encountered while extracting path.")
            edge_ids.append(eid)
            states.append(edge.target)
            raw_cost += edge.cost
            reliability += edge.reliability
            current = edge.target
            seen.add(current)
        clearance = min((self._clearance(state) for state in states), default=INF)
        return PlanningResult(True, states, edge_ids, raw_cost, clearance, reliability,
                              message="Safe path found.")

    def move_start(self, new_start: int) -> None:
        self._validate_state(new_start)
        self.km += self._distance(self.last_start, new_start)
        self.last_start = self.start = new_start

    def update_transition(self, transition_id: int, **changes: object) -> None:
        """Apply an edge change and update only its affected source vertex."""
        if transition_id not in self.edges:
            raise ValueError(f"Unknown transition: {transition_id}")
        edge = self.edges[transition_id]
        for name, value in changes.items():
            if name not in {"cost", "safety", "reliability", "available"}:
                raise ValueError(f"Unsupported transition field: {name}")
            setattr(edge, name, value)
        if edge.cost < 0:
            raise ValueError("D* Lite requires non-negative transition costs")
        self._update_vertex(edge.source)

    def add_transition(self, edge: Transition) -> None:
        """Insert a new transition and make it available to the next replan."""
        if edge.id in self.edges:
            raise ValueError(f"Transition id already exists: {edge.id}")
        self._validate_edge(edge)
        self.edges[edge.id] = edge
        self.succ[edge.source].append(edge.id)
        self.pred[edge.target].append(edge.id)
        self._update_vertex(edge.source)

    def update_bad_states(self, bad_states: Set[int]) -> None:
        """Update forbidden states while retaining the loaded graph.

        Clearance can change globally, so this correctly resets D* Lite's
        value state while avoiding reconstruction of states and adjacency.
        """
        for state_id in bad_states:
            self._validate_state(state_id)
        self.bad = set(bad_states)
        # Clearance is global, so every effective edge cost may have changed.
        self._initialize()

    def update_goal(self, new_goal: int) -> None:
        """Change goal and reinitialize D* Lite's goal-dependent values safely."""
        self._validate_state(new_goal)
        self.goal = new_goal
        self._initialize()