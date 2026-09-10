import heapq
import math

from state import State
from transition import Transition


class Planner:

    def __init__(self):

        # Graph
        self.states = {}
        self.graph = {}

        # Problem definition
        self.start = None
        self.goal = None
        self.bad_states = set()

        # LPA* values
        self.g = {}
        self.rhs = {}

        # Priority Queue
        self.open_list = []

    # ---------------------------------------------------
    # Graph Functions
    # ---------------------------------------------------

    def add_state(self, state: State):

        self.states[state.id] = state

        if state.id not in self.graph:
            self.graph[state.id] = []

    def add_transition(self, transition: Transition):

        if transition.from_state not in self.graph:
            self.graph[transition.from_state] = []

        self.graph[transition.from_state].append(transition)

    def set_initial_state(self, state_id):
        self.start = state_id

    def set_goal_state(self, state_id):
        self.goal = state_id

    def add_bad_state(self, state_id):
        self.bad_states.add(state_id)

    # ---------------------------------------------------
    # Heuristic
    # ---------------------------------------------------

    def heuristic(self, s1, s2):

        state1 = self.states[s1]
        state2 = self.states[s2]

        return math.dist(
            (state1.x, state1.y),
            (state2.x, state2.y)
        )

    # ---------------------------------------------------
    # Key Calculation
    # ---------------------------------------------------

    def calculate_key(self, state):

        value = min(
            self.g.get(state, float("inf")),
            self.rhs.get(state, float("inf"))
        )

        return (
            value + self.heuristic(state, self.goal),
            value
        )

    # ---------------------------------------------------
    # Initialize
    # ---------------------------------------------------

    def initialize(self):

        self.g.clear()
        self.rhs.clear()
        self.open_list.clear()

        for state in self.states:

            self.g[state] = float("inf")
            self.rhs[state] = float("inf")

        self.rhs[self.start] = 0

        heapq.heappush(
            self.open_list,
            (
                self.calculate_key(self.start),
                self.start
            )
        )

    # ---------------------------------------------------
    # Successors
    # ---------------------------------------------------

    def get_successors(self, state):

        result = []

        for edge in self.graph.get(state, []):

            if edge.available and edge.to_state not in self.bad_states:

                result.append(edge)

        return result

    # ---------------------------------------------------
    # Predecessors
    # ---------------------------------------------------

    def get_predecessors(self, state):

        result = []

        for node in self.graph:

            for edge in self.graph[node]:

                if edge.available and edge.to_state == state:

                    result.append(edge)

        return result

    def edge_weight(self, edge: Transition):
        """
        Compute the effective edge weight using
        cost, safety and reliability.
        Lower weight is better.
        """

        safety_bonus = edge.safety
        reliability_bonus = edge.reliability

        return edge.cost - (0.5 * safety_bonus) - (0.5 * reliability_bonus)

    def reconstruct_path(self, came_from, current):

        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path

    def plan(self):

        open_set = []

        heapq.heappush(
            open_set,
            (0, self.start)
        )

        came_from = {}

        g_score = {
            state: float("inf")
            for state in self.states
        }

        g_score[self.start] = 0

        f_score = {
            state: float("inf")
            for state in self.states
        }

        f_score[self.start] = self.heuristic(
            self.start,
            self.goal
        )

        while open_set:

            _, current = heapq.heappop(open_set)

            if current == self.goal:
                return self.reconstruct_path(
                    came_from,
                    current
                )

            for edge in self.get_successors(current):

                tentative = (
                        g_score[current]
                        + self.edge_weight(edge)
                )

                if tentative < g_score[edge.to_state]:
                    came_from[edge.to_state] = current

                    g_score[edge.to_state] = tentative

                    f_score[edge.to_state] = (
                            tentative
                            + self.heuristic(
                        edge.to_state,
                        self.goal
                    )
                    )

                    heapq.heappush(
                        open_set,
                        (
                            f_score[edge.to_state],
                            edge.to_state
                        )
                    )

        return None
