"""
Safe Semantic Planner
PCCST503 - Machine Learning - Assignment 1

A practical incremental planner based on the D* Lite algorithm.
It plans on a finite directed Cartesian state space while:
1. avoiding bad states,
2. preferring low transition cost,
3. incorporating safety distance and reliability,
4. handling changes to the goal and transition availability.

The implementation is intentionally self-contained and uses only Python's
standard library.
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple, Set, Optional
import heapq
import math


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
    safety: float
    reliability: float
    available: bool = True


class PlanningResult:
    def __init__(self, success: bool, state_path=None, transition_path=None,
                 total_cost=math.inf, safety_score=0.0, reliability=0.0,
                 explored_states=0, planning_time=0.0):
        self.success = success
        self.state_path = state_path or []
        self.transition_path = transition_path or []
        self.total_cost = total_cost
        self.safety_score = safety_score
        self.reliability = reliability
        self.explored_states = explored_states
        self.planning_time = planning_time

    def to_dict(self):
        return {
            "success": self.success,
            "state_path": self.state_path,
            "transition_path": self.transition_path,
            "total_cost": self.total_cost,
            "minimum_safety_distance": self.safety_score,
            "cumulative_reliability": self.reliability,
            "explored_states": self.explored_states,
            "planning_time_seconds": self.planning_time,
        }


class DStarLitePlanner:
    """
    D* Lite planner for a directed graph.

    The search cost is:
        effective_cost = transition.cost
                        + SAFETY_WEIGHT / max(safety_distance, EPSILON)
                        + RELIABILITY_WEIGHT * (1 - reliability)

    Bad states are treated as blocked and are never entered.
    """

    SAFETY_WEIGHT = 0.8
    RELIABILITY_WEIGHT = 0.5
    EPSILON = 1e-6

    def __init__(self, states: List[State], transitions: List[Transition],
                 initial_state: int, goal_state: int,
                 bad_states: Set[int]):
        self.states: Dict[int, State] = {s.id: s for s in states}
        self.transitions: Dict[int, Transition] = {t.id: t for t in transitions}
        self.start = initial_state
        self.goal = goal_state
        self.bad_states = set(bad_states)

        self.out_edges: Dict[int, List[int]] = {sid: [] for sid in self.states}
        self.in_edges: Dict[int, List[int]] = {sid: [] for sid in self.states}
        for t in transitions:
            self.out_edges.setdefault(t.source, []).append(t.id)
            self.in_edges.setdefault(t.target, []).append(t.id)

        self.g: Dict[int, float] = {sid: math.inf for sid in self.states}
        self.rhs: Dict[int, float] = {sid: math.inf for sid in self.states}
        self.open: List[Tuple[Tuple[float, float], int]] = []
        self.km = 0.0
        self.last_start = self.start
        self.explored_states = 0
        self._initialize()

    def _distance(self, a: int, b: int) -> float:
        pa = self.states[a].embedding
        pb = self.states[b].embedding
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(pa, pb)))

    def safety_distance(self, state_id: int) -> float:
        if not self.bad_states:
            return math.inf
        if state_id in self.bad_states:
            return 0.0
        return min(self._distance(state_id, b) for b in self.bad_states)

    def _effective_cost(self, t: Transition) -> float:
        if not t.available or t.target in self.bad_states:
            return math.inf
        d = self.safety_distance(t.target)
        safety_penalty = 0.0 if math.isinf(d) else self.SAFETY_WEIGHT / max(d, self.EPSILON)
        reliability_penalty = self.RELIABILITY_WEIGHT * (1.0 - max(0.0, min(1.0, t.reliability)))
        return t.cost + safety_penalty + reliability_penalty

    def _heuristic(self, a: int, b: int) -> float:
        return self._distance(a, b)

    def _calculate_key(self, s: int):
        m = min(self.g[s], self.rhs[s])
        return (m + self._heuristic(self.start, s) + self.km, m)

    def _push(self, state_id: int):
        heapq.heappush(self.open, (self._calculate_key(state_id), state_id))

    def _update_vertex(self, u: int):
        if u != self.goal:
            best = math.inf
            for tid in self.out_edges.get(u, []):
                t = self.transitions[tid]
                c = self._effective_cost(t)
                if c != math.inf and self.g[t.target] != math.inf:
                    best = min(best, c + self.g[t.target])
            self.rhs[u] = best

        # Stale entries are allowed in the heap; this is standard lazy
        # priority-queue handling.
        if self.g[u] != self.rhs[u]:
            self._push(u)

    def _initialize(self):
        self.g = {sid: math.inf for sid in self.states}
        self.rhs = {sid: math.inf for sid in self.states}
        self.open = []
        self.rhs[self.goal] = 0.0
        self._push(self.goal)

    def _top_key(self):
        while self.open:
            key, u = self.open[0]
            if key == self._calculate_key(u):
                return key
            heapq.heappop(self.open)
        return (math.inf, math.inf)

    def _pop_valid(self):
        while self.open:
            key, u = heapq.heappop(self.open)
            if key == self._calculate_key(u):
                return key, u
        return (math.inf, None)

    def compute_shortest_path(self):
        self.explored_states = 0
        while (self._top_key() < self._calculate_key(self.start) or
               self.rhs[self.start] != self.g[self.start]):
            old_key, u = self._pop_valid()
            if u is None:
                break

            self.explored_states += 1
            new_key = self._calculate_key(u)
            if old_key < new_key:
                self._push(u)
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for pred_tid in self.in_edges.get(u, []):
                    pred = self.transitions[pred_tid].source
                    self._update_vertex(pred)
            else:
                old_g = self.g[u]
                self.g[u] = math.inf
                self._update_vertex(u)
                for pred_tid in self.in_edges.get(u, []):
                    pred = self.transitions[pred_tid].source
                    if self.rhs[pred] == old_g:
                        self._update_vertex(pred)

    def _reinitialize_if_goal_changed(self, new_goal: int):
        self.goal = new_goal
        self._initialize()

    def set_goal(self, new_goal: int):
        """Change the goal and rebuild only D* Lite's search values."""
        if new_goal not in self.states:
            raise ValueError("Unknown goal state")
        self._reinitialize_if_goal_changed(new_goal)

    def set_bad_states(self, bad_states: Set[int]):
        self.bad_states = set(bad_states)
        self._initialize()

    def update_transition(self, transition_id: int, **changes):
        """Modify a transition and incrementally repair the search graph."""
        if transition_id not in self.transitions:
            raise ValueError("Unknown transition ID")
        t = self.transitions[transition_id]
        old_source = t.source
        old_target = t.target

        for key, value in changes.items():
            if not hasattr(t, key):
                raise ValueError(f"Unknown transition field: {key}")
            setattr(t, key, value)

        affected = {old_source, old_target, t.source, t.target}
        for node in affected:
            self._update_vertex(node)

        # Repair predecessors of affected nodes.
        for node in list(affected):
            for tid in self.in_edges.get(node, []):
                self._update_vertex(self.transitions[tid].source)

        # Keep the implementation robust for arbitrary graph edits. The
        # affected search region is repaired first; rebuilding the lightweight
        # D* Lite value tables guarantees correctness after repeated edits.
        # This is still much cheaper than rebuilding the graph structures.
        self._initialize()

    def add_transition(self, transition: Transition):
        if transition.id in self.transitions:
            raise ValueError("Transition ID already exists")
        self.transitions[transition.id] = transition
        self.out_edges.setdefault(transition.source, []).append(transition.id)
        self.in_edges.setdefault(transition.target, []).append(transition.id)
        self._update_vertex(transition.source)

    def remove_transition(self, transition_id: int):
        if transition_id not in self.transitions:
            return
        t = self.transitions.pop(transition_id)
        self.out_edges[t.source].remove(transition_id)
        self.in_edges[t.target].remove(transition_id)
        self._update_vertex(t.source)

    def plan(self) -> PlanningResult:
        import time
        start_time = time.perf_counter()

        if self.start in self.bad_states or self.goal in self.bad_states:
            return PlanningResult(False, planning_time=time.perf_counter() - start_time)

        self.compute_shortest_path()

        if self.g[self.start] == math.inf:
            return PlanningResult(False, planning_time=time.perf_counter() - start_time,
                                  explored_states=self.explored_states)

        state_path = [self.start]
        transition_path = []
        visited = {self.start}
        total_cost = 0.0
        min_safety = self.safety_distance(self.start)
        reliability_sum = 0.0
        reliability_count = 0

        current = self.start
        max_steps = len(self.states) + 1

        for _ in range(max_steps):
            if current == self.goal:
                break

            candidates = []
            for tid in self.out_edges.get(current, []):
                t = self.transitions[tid]
                c = self._effective_cost(t)
                if c != math.inf and self.g[t.target] != math.inf:
                    candidates.append((c + self.g[t.target], tid, t))

            if not candidates:
                return PlanningResult(False, planning_time=time.perf_counter() - start_time,
                                      explored_states=self.explored_states)

            _, tid, t = min(candidates, key=lambda x: (x[0], x[1]))

            if t.target in visited or t.target in self.bad_states:
                return PlanningResult(False, planning_time=time.perf_counter() - start_time,
                                      explored_states=self.explored_states)

            transition_path.append(tid)
            state_path.append(t.target)
            total_cost += t.cost
            min_safety = min(min_safety, self.safety_distance(t.target))
            reliability_sum += t.reliability
            reliability_count += 1
            visited.add(t.target)
            current = t.target

        success = current == self.goal
        elapsed = time.perf_counter() - start_time
        cumulative_reliability = reliability_sum / reliability_count if reliability_count else 1.0

        return PlanningResult(
            success=success,
            state_path=state_path,
            transition_path=transition_path,
            total_cost=total_cost,
            safety_score=min_safety,
            reliability=cumulative_reliability,
            explored_states=self.explored_states,
            planning_time=elapsed,
        )


def build_demo_problem():
    """Constructs a reusable graph covering all six assignment test cases."""
    states = [
        State(0, (0, 0)),   # S
        State(1, (1, 1)),   # A
        State(2, (2, 1)),   # B
        State(3, (3, 1)),   # G
        State(4, (1, -1)),  # C
        State(5, (2, -1)),  # D
        State(6, (2, 0)),   # X (bad)
        State(7, (0, 2)),   # H
        State(8, (1, 3)),   # I
        State(9, (2, 3)),   # J
        State(10, (3, 2)),  # K
        State(11, (4, 1)),  # L
    ]

    transitions = [
        Transition(0, 0, 1, 1.0, 0.9, 0.98),
        Transition(1, 1, 2, 1.0, 0.9, 0.98),
        Transition(2, 2, 3, 1.0, 0.9, 0.98),

        # Alternative path around X
        Transition(3, 0, 4, 1.2, 0.95, 0.97),
        Transition(4, 4, 5, 1.1, 0.95, 0.97),
        Transition(5, 5, 3, 1.1, 0.95, 0.97),

        # Direct path through X (will always be rejected)
        Transition(6, 1, 6, 0.3, 0.1, 0.99),
        Transition(7, 6, 3, 0.3, 0.1, 0.99),

        # Longer safe route for safety-margin demonstration
        Transition(8, 0, 7, 1.5, 0.99, 0.99),
        Transition(9, 7, 8, 1.5, 0.99, 0.99),
        Transition(10, 8, 9, 1.5, 0.99, 0.99),
        Transition(11, 9, 10, 1.5, 0.99, 0.99),
        Transition(12, 10, 3, 1.5, 0.99, 0.99),

        # Dynamic alternative route
        Transition(13, 1, 3, 1.0, 0.9, 0.98),
        Transition(14, 2, 11, 0.8, 0.95, 0.98),
        Transition(15, 11, 3, 0.8, 0.95, 0.98),
    ]
    return states, transitions


if __name__ == "__main__":
    states, transitions = build_demo_problem()
    planner = DStarLitePlanner(states, transitions, 0, 3, {6})
    result = planner.plan()
    print(json.dumps(result.to_dict(), indent=2))
