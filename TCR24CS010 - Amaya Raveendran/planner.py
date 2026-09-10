import math
import heapq
import time
import tracemalloc

from models import PlanningResult


class SafeSemanticPlanner:

    def __init__(self, problem):
        self.problem = problem

    def distance(self, state1, state2):
        a = state1.embedding
        b = state2.embedding

        return math.sqrt(
            sum((x - y) ** 2 for x, y in zip(a, b))
        )

    def safety_distance(self, state_id):
        if not self.problem.bad_states:
            return float("inf")

        state = self.get_state(state_id)

        distances = []

        for bad_id in self.problem.bad_states:
            bad_state = self.get_state(bad_id)
            distances.append(self.distance(state, bad_state))

        return min(distances)

    def get_state(self, state_id):
        for state in self.problem.states:
            if state.id == state_id:
                return state

        return None

    def heuristic(self, state_id):
        current = self.get_state(state_id)
        goal = self.get_state(self.problem.goal_state)

        return self.distance(current, goal)

    def transition_score(self, transition):
        """
        Lower score is better.

        Cost is the main factor.
        Low safety and reliability increase the score.
        """

        safety_penalty = (1.0 - transition.safety) * 2.0
        reliability_penalty = (1.0 - transition.reliability) * 2.0

        return (
            transition.cost
            + safety_penalty
            + reliability_penalty
        )

    def minimum_safety_distance(self, path):
        if not path:
            return 0.0

        distances = [
            self.safety_distance(state_id)
            for state_id in path
            if state_id not in self.problem.bad_states
        ]

        if not distances:
            return 0.0

        return min(distances)

    def plan(self):

        start_time = time.perf_counter()
        tracemalloc.start()

        start = self.problem.initial_state
        goal = self.problem.goal_state
        bad_states = self.problem.bad_states

        if start in bad_states or goal in bad_states:
            tracemalloc.stop()

            return PlanningResult(
                False, [], [], float("inf"), 0.0, 0,
                time.perf_counter() - start_time, 0
            )

        queue = []
        heapq.heappush(queue, (0, start))

        cost_so_far = {start: 0}
        parent = {start: None}
        parent_transition = {start: None}

        explored = set()

        while queue:

            current_cost, current = heapq.heappop(queue)

            if current in explored:
                continue

            explored.add(current)

            if current == goal:
                break

            for transition in self.problem.transitions:

                if not transition.available:
                    continue

                if transition.from_state != current:
                    continue

                next_state = transition.to_state

                # Never enter a bad state
                if next_state in bad_states:
                    continue

                edge_cost = self.transition_score(transition)

                new_cost = cost_so_far[current] + edge_cost

                if (
                    next_state not in cost_so_far
                    or new_cost < cost_so_far[next_state]
                ):
                    cost_so_far[next_state] = new_cost

                    priority = (
                        new_cost
                        + self.heuristic(next_state)
                    )

                    heapq.heappush(
                        queue,
                        (priority, next_state)
                    )

                    parent[next_state] = current
                    parent_transition[next_state] = transition.id

        # Goal not reached
        if goal not in parent:
            current_memory, peak_memory = tracemalloc.get_traced_memory()
            tracemalloc.stop()

            return PlanningResult(
                False,
                [],
                [],
                float("inf"),
                0.0,
                len(explored),
                time.perf_counter() - start_time,
                peak_memory
            )

        # Reconstruct state path
        state_path = []
        transition_path = []

        current = goal

        while current is not None:
            state_path.append(current)

            if parent_transition[current] is not None:
                transition_path.append(
                    parent_transition[current]
                )

            current = parent[current]

        state_path.reverse()
        transition_path.reverse()

        total_cost = cost_so_far[goal]

        minimum_distance = self.minimum_safety_distance(
            state_path
        )

        current_memory, peak_memory = tracemalloc.get_traced_memory()
        tracemalloc.stop()

        return PlanningResult(
            True,
            state_path,
            transition_path,
            round(total_cost, 2),
            round(minimum_distance, 2),
            len(explored),
            time.perf_counter() - start_time,
            peak_memory
        )
