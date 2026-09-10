"""
Cartesian Graph representation and spatial indexing for semantic planning.
"""

from typing import Dict, List, Set, Tuple, Optional
import numpy as np
from .models import State, Transition


class CartesianGraph:
    """
    Graph where vertices have Cartesian coordinate embeddings in R^d.
    Maintains forward and backward adjacency lists for LPA* and D* Lite.
    """

    def __init__(self):
        self.states: Dict[int, State] = {}
        self.transitions: Dict[int, Transition] = {}
        self.succ: Dict[int, Dict[int, Transition]] = {}
        self.pred: Dict[int, Dict[int, Transition]] = {}
        self.bad_states: Set[int] = set()

        self._bad_distance_cache: Dict[int, float] = {}
        self._bad_matrix: Optional[np.ndarray] = None
        self._dirty_spatial: bool = True

    def add_state(self, state: State) -> None:
        self.states[state.id] = state
        if state.id not in self.succ:
            self.succ[state.id] = {}
        if state.id not in self.pred:
            self.pred[state.id] = {}
        self._dirty_spatial = True

    def get_state(self, state_id: int) -> Optional[State]:
        return self.states.get(state_id)

    def add_transition(self, transition: Transition) -> None:
        self.transitions[transition.id] = transition
        if transition.from_state not in self.succ:
            self.succ[transition.from_state] = {}
        if transition.to_state not in self.pred:
            self.pred[transition.to_state] = {}

        self.succ[transition.from_state][transition.to_state] = transition
        self.pred[transition.to_state][transition.from_state] = transition

    def get_transition(self, u: int, v: int) -> Optional[Transition]:
        return self.succ.get(u, {}).get(v)

    def remove_transition(self, u: int, v: int) -> Optional[Transition]:
        trans = self.succ.get(u, {}).pop(v, None)
        if v in self.pred and u in self.pred[v]:
            del self.pred[v][u]
        if trans and trans.id in self.transitions:
            del self.transitions[trans.id]
        return trans

    def set_transition_availability(self, u: int, v: int, available: bool) -> bool:
        trans = self.get_transition(u, v)
        if trans is not None:
            trans.available = available
            return True
        return False

    def update_transition_cost(self, u: int, v: int, new_cost: float) -> bool:
        trans = self.get_transition(u, v)
        if trans is not None:
            trans.cost = new_cost
            return True
        return False

    def _rebuild_bad_matrix(self) -> None:
        if not self.bad_states:
            self._bad_matrix = np.empty((0, 0))
        else:
            bad_vectors = [self.states[bid].vector for bid in self.bad_states if bid in self.states]
            if bad_vectors:
                self._bad_matrix = np.array(bad_vectors, dtype=np.float64)
            else:
                self._bad_matrix = np.empty((0, 0))
        self._dirty_spatial = False
        self._bad_distance_cache.clear()

    def set_bad_states(self, bad_states: Set[int]) -> None:
        self.bad_states = set(bad_states)
        self._dirty_spatial = True
        self._bad_distance_cache.clear()

    def add_bad_state(self, state_id: int) -> None:
        self.bad_states.add(state_id)
        self._dirty_spatial = True
        self._bad_distance_cache.clear()

    def remove_bad_state(self, state_id: int) -> None:
        self.bad_states.discard(state_id)
        self._dirty_spatial = True
        self._bad_distance_cache.clear()

    def is_bad(self, state_id: int) -> bool:
        return state_id in self.bad_states

    def get_successors(self, u: int, only_available: bool = True) -> List[Tuple[int, Transition]]:
        if u not in self.succ:
            return []
        res = []
        for v, t in self.succ[u].items():
            if not only_available or t.available:
                res.append((v, t))
        return res

    def get_predecessors(self, v: int, only_available: bool = True) -> List[Tuple[int, Transition]]:
        if v not in self.pred:
            return []
        res = []
        for u, t in self.pred[v].items():
            if not only_available or t.available:
                res.append((u, t))
        return res

    def euclidean_distance(self, u_id: int, v_id: int) -> float:
        su = self.states.get(u_id)
        sv = self.states.get(v_id)
        if su is None or sv is None:
            return float("inf")
        return su.distance_to(sv)

    def min_distance_to_bad_states(self, state_id: int) -> float:
        """Computes minimum Euclidean distance from state_id to any bad state."""
        if not self.bad_states:
            return float("inf")
        if state_id in self._bad_distance_cache:
            return self._bad_distance_cache[state_id]

        target_state = self.states.get(state_id)
        if target_state is None:
            return float("inf")

        if self._dirty_spatial or self._bad_matrix is None:
            self._rebuild_bad_matrix()

        if self._bad_matrix.shape[0] == 0:
            return float("inf")

        diff = self._bad_matrix - target_state.vector
        dists = np.linalg.norm(diff, axis=1)
        min_d = float(np.min(dists))
        self._bad_distance_cache[state_id] = min_d
        return min_d

    @classmethod
    def from_planning_problem(cls, problem) -> "CartesianGraph":
        graph = cls()
        for s in problem.states:
            graph.add_state(State(id=s.id, embedding=np.copy(s.embedding), name=s.name))
        for t in problem.transitions:
            graph.add_transition(
                Transition(
                    id=t.id,
                    from_state=t.from_state,
                    to_state=t.to_state,
                    cost=t.cost,
                    safety=t.safety,
                    reliability=t.reliability,
                    available=t.available,
                    metadata=dict(t.metadata)
                )
            )
        graph.set_bad_states(set(problem.bad_states))
        return graph
