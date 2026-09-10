from dataclasses import dataclass, field
from typing import List, Dict, Set, Tuple, Optional
import math
import heapq
import time

@dataclass
class State:
    id: int
    embedding: List[float]  # Cartesian space coordinates in R^d

@dataclass
class Transition:
    id: int
    from_state: int
    to_state: int
    cost: float
    safety: float
    reliability: float
    available: bool

@dataclass
class PlanningProblem:
    initial_state: int
    goal_state: int
    bad_states: List[int]
    states: List[State]
    transitions: List[Transition]

@dataclass
class PlanningResult:
    success: bool = False
    state_path: List[int] = field(default_factory=list)
    transition_path: List[int] = field(default_factory=list)
    total_cost: float = 0.0
    safety_score: float = float('inf')
    execution_time_ms: float = 0.0
    explored_nodes_count: int = 0
class SafeSemanticPlanner:
    def __init__(self, alpha: float = 1.0, beta: float = 5.0, gamma: float = 1.0):
        self.alpha = alpha  # Transition cost weight
        self.beta = beta    # Safety margin weight
        self.gamma = gamma  # Reliability weight

    @staticmethod
    def euclidean_distance(a: State, b: State) -> float:
        return math.sqrt(sum((x - y) ** 2 for x, y in zip(a.embedding, b.embedding)))

    def min_distance_to_bad(self, state: State, state_map: Dict[int, State], bad_set: Set[int]) -> float:
        if not bad_set:
            return float('inf')
        return min(self.euclidean_distance(state, state_map[bad_id]) for bad_id in bad_set)

    def plan(self, problem: PlanningProblem) -> PlanningResult:
        start_time = time.perf_counter()
        result = PlanningResult()
        
        state_map: Dict[int, State] = {s.id: s for s in problem.states}
        bad_set: Set[int] = set(problem.bad_states)

        if problem.initial_state in bad_set or problem.goal_state in bad_set:
            result.execution_time_ms = (time.perf_counter() - start_time) * 1000
            return result

        adj: Dict[int, List[Transition]] = {s.id: [] for s in problem.states}
        for t in problem.transitions:
            if t.available and t.to_state not in bad_set:
                adj[t.from_state].append(t)

        pq: List[Tuple[float, int]] = []
        g_score: Dict[int, float] = {s.id: float('inf') for s in problem.states}
        parent_state: Dict[int, int] = {}
        parent_transition: Dict[int, int] = {}

        g_score[problem.initial_state] = 0.0
        h_start = self.euclidean_distance(state_map[problem.initial_state], state_map[problem.goal_state])
        heapq.heappush(pq, (h_start, problem.initial_state))

        nodes_explored = 0

        while pq:
            current_f, curr_id = heapq.heappop(pq)
            nodes_explored += 1

            if curr_id == problem.goal_state:
                result.success = True
                break

            for edge in adj.get(curr_id, []):
                nxt_id = edge.to_state
                nxt_state = state_map[nxt_id]

                dist_bad = self.min_distance_to_bad(nxt_state, state_map, bad_set)
                safety_penalty = 1.0 / (dist_bad + 1e-5) if dist_bad > 0 else 1e6
                
                edge_cost = (self.alpha * edge.cost) + (self.beta * safety_penalty) + (self.gamma * (1.0 - edge.reliability))
                tentative_g = g_score[curr_id] + edge_cost

                if tentative_g < g_score[nxt_id]:
                    g_score[nxt_id] = tentative_g
                    parent_state[nxt_id] = curr_id
                    parent_transition[nxt_id] = edge.id
                    h = self.euclidean_distance(nxt_state, state_map[problem.goal_state])
                    heapq.heappush(pq, (tentative_g + h, nxt_id))

        if result.success:
            curr = problem.goal_state
            min_safety_dist = float('inf')

            while curr != problem.initial_state:
                result.state_path.append(curr)
                result.transition_path.append(parent_transition[curr])
                dist_bad = self.min_distance_to_bad(state_map[curr], state_map, bad_set)
                min_safety_dist = min(min_safety_dist, dist_bad)
                curr = parent_state[curr]

            result.state_path.append(problem.initial_state)
            result.state_path.reverse()
            result.transition_path.reverse()
            result.total_cost = g_score[problem.goal_state]
            result.safety_score = min_safety_dist

        result.explored_nodes_count = nodes_explored
        result.execution_time_ms = (time.perf_counter() - start_time) * 1000
        return result