import math
import heapq
from dataclasses import dataclass, field
from typing import List, Dict, Set, Tuple, Optional



@dataclass
class State:
    id: int
    embedding: List[float]  

@dataclass
class Transition:
    id: int
    from_id: int
    to_id: int
    cost: float
    safety: float
    reliability: float
    available: bool = True

@dataclass
class PlanningProblem:
    initial_state: int
    goal_state: int
    bad_states: List[int]
    states: List[State]
    transitions: List[Transition]

@dataclass
class PlanningResult:
    success: bool
    state_path: List[int]
    transition_path: List[int]
    total_cost: float
    safety_score: float  



class LPAPlanner:
    def __init__(self, safety_weight: float = 2.0):
        """
        :param safety_weight: Weight applied to safety distance penalties in cost calculation.
        """
        self.safety_weight = safety_weight
        
        # Problem Data
        self.states: Dict[int, State] = {}
        self.transitions: Dict[int, Transition] = {}
        self.succs: Dict[int, List[int]] = {} 
        self.preds: Dict[int, List[int]] = {} 
        self.edge_to_trans: Dict[Tuple[int, int], Transition] = {}
        
        self.start_id: int = -1
        self.goal_id: int = -1
        self.bad_states: Set[int] = set()
        
        
        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {}
        
        
        self.pq: List[Tuple[float, float, int]] = []
        self.in_pq: Dict[int, Tuple[float, float]] = {}  

    def _euclidean_dist(self, id1: int, id2: int) -> float:
        """Calculates Euclidean distance between two states in R^d."""
        p1 = self.states[id1].embedding
        p2 = self.states[id2].embedding
        return math.sqrt(sum((a - b) ** 2 for a, b in zip(p1, p2)))

    def _min_dist_to_bad(self, state_id: int) -> float:
        """Calculates minimum Euclidean distance from state_id to any bad state."""
        if not self.bad_states:
            return float('inf')
        return min(self._euclidean_dist(state_id, b_id) for b_id in self.bad_states)

    def _heuristic(self, state_id: int) -> float:
        """Admissible Euclidean heuristic to goal state."""
        return self._euclidean_dist(state_id, self.goal_id)

    def _get_edge_cost(self, u: int, v: int) -> float:
        """
        Calculates composite edge cost incorporating transition cost, safety penalty,
        and bad state avoidance.
        """
        if v in self.bad_states or u in self.bad_states:
            return float('inf')
            
        trans = self.edge_to_trans.get((u, v))
        if trans is None or not trans.available:
            return float('inf')
            
        base_cost = trans.cost
        
        min_bad_dist = self._min_dist_to_bad(v)
        safety_penalty = 0.0
        if min_bad_dist < float('inf') and min_bad_dist > 0:
            safety_penalty = self.safety_weight / min_bad_dist
        elif min_bad_dist == 0:
            return float('inf')

        return base_cost + safety_penalty

    def _calculate_key(self, u: int) -> Tuple[float, float]:
        """LPA* priority queue key: [min(g(u), rhs(u)) + h(u), min(g(u), rhs(u))]."""
        m = min(self.g.get(u, float('inf')), self.rhs.get(u, float('inf')))
        return (m + self._heuristic(u), m)

    def _pq_insert_or_update(self, u: int):
        """Inserts or updates a node in the priority queue."""
        key = self._calculate_key(u)
        self.in_pq[u] = key
        heapq.heappush(self.pq, (key[0], key[1], u))

    def _pq_remove(self, u: int):
        """Lazy removal of a node from priority queue tracking."""
        if u in self.in_pq:
            del self.in_pq[u]

    def _update_vertex(self, u: int):
        """Core LPA* vertex update step."""
        if u != self.start_id:
            min_rhs = float('inf')
            for p in self.preds.get(u, []):
                cost = self._get_edge_cost(p, u)
                min_rhs = min(min_rhs, self.g.get(p, float('inf')) + cost)
            self.rhs[u] = min_rhs

        self._pq_remove(u)

        if self.g.get(u, float('inf')) != self.rhs.get(u, float('inf')):
            self._pq_insert_or_update(u)

    def _compute_shortest_path(self):
        """Main search loop of LPA* algorithm."""
        while self.pq:
      
            k1, k2, u = self.pq[0]
            if u not in self.in_pq or self.in_pq[u] != (k1, k2):
                heapq.heappop(self.pq)
                continue

            goal_key = self._calculate_key(self.goal_id)
            if (k1, k2) >= goal_key and self.rhs.get(self.goal_id, float('inf')) == self.g.get(self.goal_id, float('inf')):
                break

            heapq.heappop(self.pq)
            del self.in_pq[u]

            g_u = self.g.get(u, float('inf'))
            rhs_u = self.rhs.get(u, float('inf'))

            if g_u > rhs_u:
                self.g[u] = rhs_u
                for s in self.succs.get(u, []):
                    self._update_vertex(s)
            else:
                self.g[u] = float('inf')
                self._update_vertex(u)
                for s in self.succs.get(u, []):
                    self._update_vertex(s)

    def plan(self, problem: PlanningProblem) -> PlanningResult:
        """Initializes and executes initial search."""
        self.states = {s.id: s for s in problem.states}
        self.transitions = {t.id: t for t in problem.transitions}
        self.start_id = problem.initial_state
        self.goal_id = problem.goal_state
        self.bad_states = set(problem.bad_states)

        self.succs.clear()
        self.preds.clear()
        self.edge_to_trans.clear()

        for t in problem.transitions:
            self.succs.setdefault(t.from_id, []).append(t.to_id)
            self.preds.setdefault(t.to_id, []).append(t.from_id)
            self.edge_to_trans[(t.from_id, t.to_id)] = t

        self.g = {s_id: float('inf') for s_id in self.states}
        self.rhs = {s_id: float('inf') for s_id in self.states}
        self.pq.clear()
        self.in_pq.clear()

        self.rhs[self.start_id] = 0.0
        self._pq_insert_or_update(self.start_id)

        self._compute_shortest_path()
        return self._extract_result()

    def update_edge(self, from_id: int, to_id: int, available: Optional[bool] = None, cost: Optional[float] = None) -> PlanningResult:
        """Dynamically updates a transition and incrementally replans."""
        key = (from_id, to_id)
        if key in self.edge_to_trans:
            t = self.edge_to_trans[key]
            if available is not None:
                t.available = available
            if cost is not None:
                t.cost = cost
            self._update_vertex(to_id)
            self._compute_shortest_path()
        return self._extract_result()

    def update_goal(self, new_goal_id: int) -> PlanningResult:
        """Dynamically changes goal state and replans."""
        self.goal_id = new_goal_id
       
        curr_nodes = list(self.in_pq.keys())
        self.pq.clear()
        self.in_pq.clear()
        for u in curr_nodes:
            self._pq_insert_or_update(u)
        
        self._compute_shortest_path()
        return self._extract_result()

    def _extract_result(self) -> PlanningResult:
        """Reconstructs optimal path and computes path metrics."""
        if self.g.get(self.goal_id, float('inf')) == float('inf'):
            return PlanningResult(False, [], [], float('inf'), 0.0)

       
        curr = self.goal_id
        state_path = [curr]
        trans_path = []

        while curr != self.start_id:
            best_pred = None
            best_cost = float('inf')
            best_trans = None

            for p in self.preds.get(curr, []):
                edge_cost = self._get_edge_cost(p, curr)
                candidate_val = self.g.get(p, float('inf')) + edge_cost
                if candidate_val < best_cost:
                    best_cost = candidate_val
                    best_pred = p
                    best_trans = self.edge_to_trans.get((p, curr))

            if best_pred is None:
                return PlanningResult(False, [], [], float('inf'), 0.0)

            trans_path.append(best_trans.id if best_trans else -1)
            state_path.append(best_pred)
            curr = best_pred

        state_path.reverse()
        trans_path.reverse()

      
        total_cost = sum(self.edge_to_trans[(state_path[i], state_path[i+1])].cost for i in range(len(state_path)-1))
        min_safety_dist = min((self._min_dist_to_bad(s) for s in state_path), default=float('inf'))

        return PlanningResult(
            success=True,
            state_path=state_path,
            transition_path=trans_path,
            total_cost=total_cost,
            safety_score=min_safety_dist
        )




if __name__ == "__main__":
    print("--- Running Test Cases ---")

    def make_state(s_id, x, y):
        return State(id=s_id, embedding=[float(x), float(y)])

    def make_trans(t_id, u, v, cost=1.0):
        return Transition(id=t_id, from_id=u, to_id=v, cost=cost, safety=1.0, reliability=1.0, available=True)

   
    states_tc1 = [make_state(1, 0, 0), make_state(2, 1, 0), make_state(3, 2, 0), make_state(4, 3, 0)]
    trans_tc1 = [make_trans(101, 1, 2), make_trans(102, 2, 3), make_trans(103, 3, 4)]
    prob_tc1 = PlanningProblem(initial_state=1, goal_state=4, bad_states=[], states=states_tc1, transitions=trans_tc1)
    
    planner = LPAPlanner(safety_weight=1.0)
    res1 = planner.plan(prob_tc1)
    print(f"Test Case 1 (Basic Path): Success={res1.success}, Path={res1.state_path}, Cost={res1.total_cost}")

   
    states_tc2 = [make_state(i, i, 0) for i in range(1, 7)]
    trans_tc2 = [
        make_trans(201, 1, 2), make_trans(202, 2, 3), make_trans(203, 3, 5), 
        make_trans(204, 1, 4), make_trans(205, 4, 6), make_trans(206, 6, 5)  
    ]
    prob_tc2 = PlanningProblem(initial_state=1, goal_state=5, bad_states=[3], states=states_tc2, transitions=trans_tc2)
    res2 = planner.plan(prob_tc2)
    print(f"Test Case 2 (Bad State Avoidance): Success={res2.success}, Path={res2.state_path}")

  
    states_tc3 = [
        make_state(1, 0, 0), make_state(2, 1, 0.1), make_state(3, 2, 0),
        make_state(4, 1, -5.0), make_state(99, 1, 1.0)
    ]
    trans_tc3 = [
        make_trans(301, 1, 2, cost=1.0), make_trans(302, 2, 3, cost=1.0),
        make_trans(303, 1, 4, cost=2.0), make_trans(304, 4, 3, cost=2.0)
    ]
    prob_tc3 = PlanningProblem(initial_state=1, goal_state=3, bad_states=[99], states=states_tc3, transitions=trans_tc3)
    res3 = planner.plan(prob_tc3)
    print(f"Test Case 3 (Safety Margin): Selected Path={res3.state_path}, Min Safety Dist={res3.safety_score:.2f}")


    res4_before = planner.plan(prob_tc1) 
    res4_after = planner.update_edge(from_id=2, to_id=3, available=False)
    print(f"Test Case 4 (Edge Dynamic Removal): Path after disable={res4_after.state_path} (Success={res4_after.success})")

    planner.plan(prob_tc1)
    planner.update_edge(from_id=2, to_id=3, available=True)
    res5 = planner.update_goal(new_goal_id=3)
    print(f"Test Case 5 (Goal Update): Path to new goal 3={res5.state_path}")