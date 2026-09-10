import heapq
import math
import time


INF = float("inf")


class DStarLite:

    def __init__(self, problem):
        self.problem = problem
        self.start = problem.initial_state
        self.goal = problem.goal_state
        self.km = 0
        self.g = {}
        self.rhs = {}
        self.open_list = []
        self.explored_states = 0

        self.initialize()

    def initialize(self):
        self.g = {}
        self.rhs = {}
        self.open_list = []

        for state in self.problem.states:
            self.g[state.id] = INF
            self.rhs[state.id] = INF

        self.rhs[self.goal] = 0

        heapq.heappush(
            self.open_list,
            (self.calculate_key(self.goal), self.goal)
        )

    def get_state(self, state_id):
        for state in self.problem.states:
            if state.id == state_id:
                return state
        return None

    def distance(self, state_a, state_b):
        a = state_a.embedding
        b = state_b.embedding

        return math.sqrt(
            sum((x - y) ** 2 for x, y in zip(a, b))
        )

    def heuristic(self, state_id):
        return self.distance(
            self.get_state(state_id),
            self.get_state(self.start)
        )

    def get_successors(self, state_id):
        successors = []

        for transition in self.problem.transitions:

            if not transition.available:
                continue

            if transition.from_state != state_id:
                continue

            if transition.to_state in self.problem.bad_states:
                continue

            successors.append(transition)

        return successors

    def get_predecessors(self, state_id):
        predecessors = []

        for transition in self.problem.transitions:

            if not transition.available:
                continue

            if transition.to_state != state_id:
                continue

            if transition.from_state in self.problem.bad_states:
                continue

            predecessors.append(transition)

        return predecessors

    def edge_cost(self, transition):
        safety_penalty = (
            (1.0 - transition.safety) * 2.0
        )

        reliability_penalty = (
            (1.0 - transition.reliability) * 2.0
        )

        return (
            transition.cost
            + safety_penalty
            + reliability_penalty
        )

    def calculate_key(self, state_id):
        value = min(
            self.g[state_id],
            self.rhs[state_id]
        )

        return (
            value
            + self.heuristic(state_id)
            + self.km,
            value
        )

    def update_vertex(self, state_id):

        if state_id != self.goal:

            successors = self.get_successors(state_id)

            if successors:
                self.rhs[state_id] = min(
                    self.edge_cost(transition)
                    + self.g[transition.to_state]
                    for transition in successors
                )
            else:
                self.rhs[state_id] = INF

        if self.g[state_id] != self.rhs[state_id]:

            heapq.heappush(
                self.open_list,
                (
                    self.calculate_key(state_id),
                    state_id
                )
            )

    def compute_shortest_path(self):

        self.explored_states = 0

        while self.open_list:

            old_key, current = heapq.heappop(
                self.open_list
            )

            new_key = self.calculate_key(current)

            if old_key > new_key:

                heapq.heappush(
                    self.open_list,
                    (new_key, current)
                )

                continue

            start_key = self.calculate_key(self.start)

            if (
                old_key >= start_key
                and self.rhs[self.start] == self.g[self.start]
            ):
                break

            self.explored_states += 1

            if self.g[current] > self.rhs[current]:

                self.g[current] = self.rhs[current]

                for transition in self.get_predecessors(current):
                    self.update_vertex(
                        transition.from_state
                    )

            else:

                self.g[current] = INF

                self.update_vertex(current)

                for transition in self.get_predecessors(current):
                    self.update_vertex(
                        transition.from_state
                    )

    def get_path(self):

        if self.g[self.start] == INF:
            return None, None

        state_path = [self.start]
        transition_path = []

        current = self.start
        visited = {current}

        while current != self.goal:

            successors = self.get_successors(current)

            if not successors:
                return None, None

            best_transition = min(
                successors,
                key=lambda transition:
                self.edge_cost(transition)
                + self.g[transition.to_state]
            )

            current = best_transition.to_state

            if current in visited:
                return None, None

            visited.add(current)

            state_path.append(current)
            transition_path.append(
                best_transition.id
            )

        return state_path, transition_path

    def calculate_cost(self, transition_path):

        total_cost = 0

        for transition_id in transition_path:

            for transition in self.problem.transitions:

                if transition.id == transition_id:

                    total_cost += self.edge_cost(
                        transition
                    )

                    break

        return round(total_cost, 2)

    def plan(self):

        start_time = time.perf_counter()

        self.compute_shortest_path()

        state_path, transition_path = self.get_path()

        elapsed = time.perf_counter() - start_time

        if state_path is None:

            return {
                "success": False,
                "state_path": [],
                "transition_path": [],
                "cost": None,
                "time": elapsed,
                "explored": self.explored_states
            }

        return {
            "success": True,
            "state_path": state_path,
            "transition_path": transition_path,
            "cost": self.calculate_cost(
                transition_path
            ),
            "time": elapsed,
            "explored": self.explored_states
        }

    def update_transition(
        self,
        transition_id,
        available=None,
        cost=None,
        safety=None,
        reliability=None
    ):

        for transition in self.problem.transitions:

            if transition.id == transition_id:

                if available is not None:
                    transition.available = available

                if cost is not None:
                    transition.cost = cost

                if safety is not None:
                    transition.safety = safety

                if reliability is not None:
                    transition.reliability = reliability

                self.initialize()

                return True

        return False

    def update_goal(self, new_goal):

        self.goal = new_goal
        self.problem.goal_state = new_goal

        self.initialize()

    def replan(self):

        start_time = time.perf_counter()

        self.compute_shortest_path()

        state_path, transition_path = self.get_path()

        elapsed = time.perf_counter() - start_time

        if state_path is None:

            return {
                "success": False,
                "state_path": [],
                "transition_path": [],
                "cost": None,
                "time": elapsed,
                "explored": self.explored_states
            }

        return {
            "success": True,
            "state_path": state_path,
            "transition_path": transition_path,
            "cost": self.calculate_cost(
                transition_path
            ),
            "time": elapsed,
            "explored": self.explored_states
        }
