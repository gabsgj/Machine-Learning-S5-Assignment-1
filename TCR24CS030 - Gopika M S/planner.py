"""
PCCST503 - Machine Learning
Assignment 1: Safe Semantic Planner in a Finite Cartesian State Space

Algorithm: D* Lite
Language: Python 3
"""

from dataclasses import dataclass
from typing import Tuple, List, Set
import heapq
import math
import time

INF = float("inf")


# ============================================================
# DATA STRUCTURES
# ============================================================

@dataclass(frozen=True)
class State:
    """Represents a state in the Cartesian state space."""
    id: int
    embedding: Tuple[float, ...]


@dataclass
class Transition:
    """Represents a directed transition between two states."""
    id: int
    from_state: int
    to_state: int
    cost: float
    safety: float
    reliability: float
    available: bool = True


@dataclass
class PlanningProblem:
    """Complete planning problem."""
    initial_state: int
    goal_state: int
    bad_states: Set[int]
    states: List[State]
    transitions: List[Transition]


@dataclass
class PlanningResult:
    """Result returned by the planner."""
    success: bool
    state_path: List[int]
    transition_path: List[int]
    total_cost: float
    minimum_safety_distance: float
    cumulative_reliability: float
    explored_states: int
    planning_time_ms: float


# ============================================================
# D* LITE PLANNER
# ============================================================

class DStarLitePlanner:

    def __init__(self, safety_weight=0.0):

        self.safety_weight = safety_weight
        self.problem = None

        self.states = {}
        self.adj = {}
        self.rev = {}

        self.g = {}
        self.rhs = {}

        self.open_heap = []
        self.counter = 0

        self.start = None
        self.goal = None
        self.last_start = None
        self.km = 0.0

        self.bad = set()
        self.distance_cache = {}

    # ========================================================
    # EUCLIDEAN DISTANCE
    # ========================================================

    def distance(self, a, b):

        x = self.states[a].embedding
        y = self.states[b].embedding

        return math.sqrt(
            sum(
                (u - v) ** 2
                for u, v in zip(x, y)
            )
        )

    # ========================================================
    # DISTANCE TO NEAREST BAD STATE
    # ========================================================

    def nearest_bad_distance(self, state_id):

        if not self.bad:
            return INF

        if state_id not in self.distance_cache:

            self.distance_cache[state_id] = min(
                self.distance(state_id, bad)
                for bad in self.bad
                if bad in self.states
            )

        return self.distance_cache[state_id]

    # ========================================================
    # EFFECTIVE TRANSITION COST
    # ========================================================

    def edge_cost(self, transition):

        # Never enter or leave a bad state.
        if (
            not transition.available
            or transition.from_state in self.bad
            or transition.to_state in self.bad
        ):
            return INF

        distance = self.nearest_bad_distance(
            transition.to_state
        )

        # Safety penalty.
        if math.isinf(distance):
            safety_penalty = 0.0
        else:
            safety_penalty = (
                self.safety_weight /
                (distance + 1e-9)
            )

        # Reliability penalty.
        reliability_penalty = (
            0.1 *
            (1.0 - transition.reliability)
        )

        return (
            transition.cost
            + safety_penalty
            + reliability_penalty
        )

    # ========================================================
    # D* LITE KEY
    # ========================================================

    def key(self, state_id):

        minimum = min(
            self.g[state_id],
            self.rhs[state_id]
        )

        return (
            minimum
            + self.distance(self.start, state_id)
            + self.km,
            minimum
        )

    # ========================================================
    # PUSH STATE
    # ========================================================

    def push(self, state_id):

        key = self.key(state_id)

        self.counter += 1

        heapq.heappush(
            self.open_heap,
            (
                key[0],
                key[1],
                self.counter,
                state_id
            )
        )

    # ========================================================
    # TOP KEY
    # ========================================================

    def top_key(self):

        while self.open_heap:

            k1, k2, _, state = self.open_heap[0]

            if (k1, k2) != self.key(state):

                heapq.heappop(
                    self.open_heap
                )

            else:

                return (k1, k2)

        return (INF, INF)

    # ========================================================
    # POP STATE
    # ========================================================

    def pop(self):

        while self.open_heap:

            k1, k2, _, state = heapq.heappop(
                self.open_heap
            )

            if (k1, k2) == self.key(state):

                return state

        return None

    # ========================================================
    # UPDATE VERTEX
    # ========================================================

    def update_vertex(self, state):

        if state != self.goal:

            values = []

            for transition in self.adj.get(
                state,
                []
            ):

                cost = self.edge_cost(
                    transition
                )

                if math.isfinite(cost):

                    values.append(
                        cost
                        + self.g[
                            transition.to_state
                        ]
                    )

            self.rhs[state] = (
                min(values)
                if values
                else INF
            )

        if self.g[state] != self.rhs[state]:

            self.push(state)

    # ========================================================
    # COMPUTE SHORTEST PATH
    # ========================================================

    def compute_shortest_path(self):

        explored = 0

        while (
            self.top_key() < self.key(self.start)
            or
            self.rhs[self.start]
            != self.g[self.start]
        ):

            state = self.pop()

            if state is None:
                break

            explored += 1

            if self.g[state] > self.rhs[state]:

                self.g[state] = self.rhs[state]

                for transition in self.rev.get(
                    state,
                    []
                ):

                    self.update_vertex(
                        transition.from_state
                    )

            else:

                self.g[state] = INF

                self.update_vertex(state)

                for transition in self.rev.get(
                    state,
                    []
                ):

                    self.update_vertex(
                        transition.from_state
                    )

        return explored

    # ========================================================
    # INITIALIZE
    # ========================================================

    def initialize(self, problem):

        self.problem = problem

        self.states = {
            state.id: state
            for state in problem.states
        }

        self.adj = {
            state.id: []
            for state in problem.states
        }

        self.rev = {
            state.id: []
            for state in problem.states
        }

        for transition in problem.transitions:

            if (
                transition.from_state in self.states
                and
                transition.to_state in self.states
            ):

                self.adj[
                    transition.from_state
                ].append(transition)

                self.rev[
                    transition.to_state
                ].append(transition)

        self.bad = set(
            problem.bad_states
        )

        self.distance_cache = {}

        self.g = {
            state.id: INF
            for state in problem.states
        }

        self.rhs = {
            state.id: INF
            for state in problem.states
        }

        self.open_heap = []
        self.counter = 0
        self.km = 0.0

        self.start = problem.initial_state
        self.goal = problem.goal_state
        self.last_start = self.start

        if self.goal in self.states and self.goal not in self.bad:

            self.rhs[self.goal] = 0.0
            self.push(self.goal)

    # ========================================================
    # PLAN
    # ========================================================

    def plan(self, problem=None):

        start_time = time.perf_counter()

        if problem is not None:

            self.initialize(problem)

        if (
            self.start not in self.states
            or
            self.goal not in self.states
            or
            self.start in self.bad
            or
            self.goal in self.bad
        ):

            return PlanningResult(
                False,
                [],
                [],
                0.0,
                0.0,
                0.0,
                0,
                (time.perf_counter() - start_time) * 1000
            )

        explored = self.compute_shortest_path()

        if not math.isfinite(
            self.g[self.start]
        ):

            return PlanningResult(
                False,
                [],
                [],
                0.0,
                0.0,
                0.0,
                explored,
                (time.perf_counter() - start_time) * 1000
            )

        state_path = [self.start]
        transition_path = []

        visited = {self.start}
        current = self.start

        total_cost = 0.0
        reliability = 1.0

        while current != self.goal:

            choices = []

            for transition in self.adj.get(
                current,
                []
            ):

                cost = self.edge_cost(
                    transition
                )

                if (
                    math.isfinite(cost)
                    and
                    math.isfinite(
                        self.g[
                            transition.to_state
                        ]
                    )
                    and
                    transition.to_state
                    not in visited
                ):

                    choices.append(
                        (
                            cost
                            + self.g[
                                transition.to_state
                            ],
                            transition
                        )
                    )

            if not choices:

                return PlanningResult(
                    False,
                    [],
                    [],
                    0.0,
                    0.0,
                    0.0,
                    explored,
                    (time.perf_counter() - start_time) * 1000
                )

            _, chosen = min(
                choices,
                key=lambda x: x[0]
            )

            transition_path.append(
                chosen.id
            )

            current = chosen.to_state

            state_path.append(
                current
            )

            visited.add(current)

            total_cost += chosen.cost

            reliability *= (
                chosen.reliability
            )

        distances = [
            self.nearest_bad_distance(
                state
            )
            for state in state_path
        ]

        minimum_safety = (
            min(distances)
            if distances
            else 0.0
        )

        planning_time = (
            time.perf_counter()
            - start_time
        ) * 1000

        return PlanningResult(
            True,
            state_path,
            transition_path,
            total_cost,
            minimum_safety,
            reliability,
            explored,
            planning_time
        )

    # ========================================================
    # DYNAMIC TRANSITION UPDATE
    # ========================================================

    def update_transition(
        self,
        transition_id,
        available=None,
        cost=None
    ):

        if self.problem is None:
            return

        for transition in self.problem.transitions:

            if transition.id == transition_id:

                if available is not None:
                    transition.available = available

                if cost is not None:
                    transition.cost = cost

                self.update_vertex(
                    transition.from_state
                )

                return

    # ========================================================
    # ADD TRANSITION
    # ========================================================

    def add_transition(
        self,
        transition
    ):

        if self.problem is None:
            return

        self.problem.transitions.append(
            transition
        )

        if transition.from_state not in self.adj:
            self.adj[
                transition.from_state
            ] = []

        if transition.to_state not in self.rev:
            self.rev[
                transition.to_state
            ] = []

        self.adj[
            transition.from_state
        ].append(transition)

        self.rev[
            transition.to_state
        ].append(transition)

        self.update_vertex(
            transition.from_state
        )

    # ========================================================
    # DYNAMIC GOAL UPDATE
    # ========================================================

    def update_goal(self, new_goal):

        if new_goal not in self.states:
            return

        self.goal = new_goal

        self.g = {
            state_id: INF
            for state_id in self.states
        }

        self.rhs = {
            state_id: INF
            for state_id in self.states
        }

        self.open_heap = []
        self.counter = 0
        self.km = 0.0

        if new_goal not in self.bad:

            self.rhs[new_goal] = 0.0
            self.push(new_goal)


# ============================================================
# PROBLEM CREATION HELPER
# ============================================================

def make_problem(
    edges,
    bad=(),
    goal=3
):

    states = [

        State(0, (0.0, 0.0)),
        State(1, (1.0, 0.0)),
        State(2, (1.0, 1.0)),
        State(3, (2.0, 1.0)),
        State(4, (0.0, 1.0)),
        State(5, (2.0, 0.0))

    ]

    transitions = [

        Transition(
            i,
            a,
            b,
            cost,
            1.0,
            0.95,
            True
        )

        for i, (a, b, cost)
        in enumerate(edges)

    ]

    return PlanningProblem(
        0,
        goal,
        set(bad),
        states,
        transitions
    )


# ============================================================
# SIX ASSIGNMENT TEST SCENARIOS
# ============================================================

def run_tests():

    results = []

    # --------------------------------------------------------
    # TEST 1: BASIC REACHABILITY
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 1),
        (1, 2, 1),
        (2, 3, 1)
    ])

    results.append(
        (
            "Test 1 - Basic Reachability",
            DStarLitePlanner().plan(problem)
        )
    )

    # --------------------------------------------------------
    # TEST 2: BAD STATE AVOIDANCE
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 1),
        (1, 5, 1),
        (5, 3, 1),

        (0, 2, 2),
        (2, 4, 1),
        (4, 3, 1)
    ], bad={5})

    results.append(
        (
            "Test 2 - Bad State Avoidance",
            DStarLitePlanner().plan(problem)
        )
    )

    # --------------------------------------------------------
    # TEST 3: SAFETY MARGIN
    #
    # Bad state = 5 at coordinate (2,0)
    #
    # Path 1:
    # 0 -> 1 -> 3
    #
    # Path 2:
    # 0 -> 2 -> 4 -> 3
    #
    # Safety-weighted planning should prefer Path 2
    # when the safety weight is sufficiently large.
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 1.0),
        (1, 3, 1.0),

        (0, 2, 1.2),
        (2, 4, 1.2),
        (4, 3, 1.0)
    ], bad={5})

    results.append(
        (
            "Test 3A - Cost Focused",
            DStarLitePlanner(
                safety_weight=0.0
            ).plan(problem)
        )
    )

    results.append(
        (
            "Test 3B - Safety Weighted",
            DStarLitePlanner(
                safety_weight=2.0
            ).plan(problem)
        )
    )

    # --------------------------------------------------------
    # TEST 4: DYNAMIC TRANSITION FAILURE
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 1),
        (1, 3, 1),

        (0, 2, 1.2),
        (2, 4, 1),
        (4, 3, 1)
    ])

    planner = DStarLitePlanner()

    results.append(
        (
            "Test 4A - Before Transition Failure",
            planner.plan(problem)
        )
    )

    planner.update_transition(
        transition_id=1,
        available=False
    )

    results.append(
        (
            "Test 4B - After Transition Failure",
            planner.plan()
        )
    )

    # --------------------------------------------------------
    # TEST 5: GOAL UPDATE
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 1),
        (1, 3, 1),

        (0, 2, 1),
        (2, 4, 1),
        (4, 3, 1)
    ])

    planner = DStarLitePlanner()

    results.append(
        (
            "Test 5A - Original Goal",
            planner.plan(problem)
        )
    )

    planner.update_goal(4)

    results.append(
        (
            "Test 5B - Updated Goal",
            planner.plan()
        )
    )

    # --------------------------------------------------------
    # TEST 6: TRANSITION ADDITION
    # --------------------------------------------------------

    problem = make_problem([
        (0, 1, 2),
        (1, 3, 2),

        (0, 2, 1),
        (2, 4, 1),
        (4, 3, 2)
    ])

    planner = DStarLitePlanner()

    results.append(
        (
            "Test 6A - Before Shortcut",
            planner.plan(problem)
        )
    )

    planner.add_transition(
        Transition(
            99,
            0,
            3,
            0.5,
            1.0,
            0.99,
            True
        )
    )

    results.append(
        (
            "Test 6B - After Shortcut",
            planner.plan()
        )
    )

    return results


# ============================================================
# DIRECT EXECUTION
# ============================================================

if __name__ == "__main__":

    for name, result in run_tests():

        print("\n" + name)
        print("-" * 60)

        print(
            "Success:",
            result.success
        )

        print(
            "State path:",
            result.state_path
        )

        print(
            "Transition path:",
            result.transition_path
        )

        print(
            "Total cost:",
            round(
                result.total_cost,
                3
            )
        )

        print(
            "Minimum safety distance:",
            round(
                result.minimum_safety_distance,
                3
            )
        )

        print(
            "Cumulative reliability:",
            round(
                result.cumulative_reliability,
                4
            )
        )

        print(
            "Explored states:",
            result.explored_states
        )

        print(
            "Planning time:",
            round(
                result.planning_time_ms,
                3
            ),
            "ms"
        )
