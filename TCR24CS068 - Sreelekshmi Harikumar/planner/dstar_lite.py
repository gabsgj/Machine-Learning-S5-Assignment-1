import heapq
import math
import time
from typing import Dict, List, Tuple

from .models import PlanningProblem, PlanningResult, Transition
from .heuristic import euclidean_heuristic
from .safety import minimum_path_safety


INF = float("inf")


class DStarLite:
    """
    Incremental D* Lite planner.

    The planner maintains g and rhs values between planning
    operations. When the environment changes, only affected
    vertices are updated instead of rebuilding the entire
    search state.

    Supported dynamic changes:
        - transition availability
        - transition addition
        - transition removal
        - goal changes
        - bad-state changes
    """

    def __init__(
        self,
        problem: PlanningProblem,
        cost_weight: float = 1.0,
        safety_weight: float = 0.0,
        reliability_weight: float = 0.0,
    ):
        self.problem = problem

        self.cost_weight = cost_weight
        self.safety_weight = safety_weight
        self.reliability_weight = reliability_weight

        self.states: Dict[int, object] = {}
        self.transitions: Dict[int, Transition] = {}

        self.g: Dict[int, float] = {}
        self.rhs: Dict[int, float] = {}

        self.open_list: List[Tuple[float, float, int]] = []

        self.explored_states = 0
        self.initialized = False

        self.refresh_graph()

        self.initialize()

    # ======================================================
    # GRAPH MANAGEMENT
    # ======================================================

    def refresh_graph(self):
        """
        Refresh internal state/transition dictionaries from
        the current planning problem.
        """

        self.states = {
            state.id: state
            for state in self.problem.states
        }

        self.transitions = {
            transition.id: transition
            for transition in self.problem.transitions
        }

    # ======================================================
    # INITIALIZATION
    # ======================================================

    def initialize(self):
        """
        Initialize the D* Lite search.

        rhs(goal) = 0 and all other values start at infinity.
        """

        self.refresh_graph()

        self.g = {
            state_id: INF
            for state_id in self.states
        }

        self.rhs = {
            state_id: INF
            for state_id in self.states
        }

        self.open_list = []

        goal = self.problem.goal_state

        if goal not in self.states:
            self.initialized = False
            return

        self.rhs[goal] = 0.0

        self.push_to_open(goal)

        self.explored_states = 0
        self.initialized = True

    # ======================================================
    # PRIORITY QUEUE
    # ======================================================

    def calculate_key(
        self,
        state_id: int
    ) -> Tuple[float, float]:

        value = min(
            self.g.get(state_id, INF),
            self.rhs.get(state_id, INF),
        )

        start = self.problem.initial_state

        heuristic_value = self.heuristic(
            start,
            state_id
        )

        return (
            value + heuristic_value,
            value,
        )

    def push_to_open(self, state_id: int):

        key = self.calculate_key(state_id)

        heapq.heappush(
            self.open_list,
            (
                key[0],
                key[1],
                state_id,
            ),
        )

    # ======================================================
    # GRAPH OPERATIONS
    # ======================================================

    def outgoing(
        self,
        state_id: int
    ) -> List[Transition]:
        """
        Available outgoing transitions.

        A transition entering a bad state is ignored.
        """

        result = []

        for transition in self.transitions.values():

            if not transition.available:
                continue

            if transition.from_state != state_id:
                continue

            if transition.to_state in self.problem.bad_states:
                continue

            result.append(transition)

        return result

    def incoming(
        self,
        state_id: int
    ) -> List[Transition]:
        """
        Available incoming transitions.

        Transitions originating from bad states are ignored.
        """

        result = []

        for transition in self.transitions.values():

            if not transition.available:
                continue

            if transition.to_state != state_id:
                continue

            if transition.from_state in self.problem.bad_states:
                continue

            result.append(transition)

        return result

    # ======================================================
    # COST
    # ======================================================

    def transition_cost(
        self,
        transition: Transition
    ) -> float:
        """
        Effective search cost.

        The original transition cost remains available for
        reporting. Safety and reliability act as penalties.
        """

        safety_penalty = 1.0 - transition.safety

        reliability_penalty = (
            1.0 - transition.reliability
        )

        return (
            self.cost_weight * transition.cost
            + self.safety_weight * safety_penalty
            + self.reliability_weight * reliability_penalty
        )

    # ======================================================
    # HEURISTIC
    # ======================================================

    def heuristic(
        self,
        state_a: int,
        state_b: int
    ) -> float:

        if (
            state_a not in self.states
            or state_b not in self.states
        ):
            return 0.0

        return euclidean_heuristic(
            self.states[state_a],
            self.states[state_b],
        )

    # ======================================================
    # UPDATE VERTEX
    # ======================================================

    def update_vertex(
        self,
        state_id: int
    ):
        """
        Recalculate rhs(state_id).

        This is the key operation used by D* Lite when
        the environment changes.
        """

        if state_id not in self.states:
            return

        if state_id == self.problem.goal_state:

            self.rhs[state_id] = 0.0

        elif state_id in self.problem.bad_states:

            self.rhs[state_id] = INF

        else:

            successors = self.outgoing(state_id)

            if not successors:

                self.rhs[state_id] = INF

            else:

                self.rhs[state_id] = min(
                    self.transition_cost(transition)
                    + self.g.get(
                        transition.to_state,
                        INF
                    )
                    for transition in successors
                )

        # We use lazy deletion for the priority queue.
        # Old entries remain in the heap and are ignored
        # when popped if their key is outdated.

        if (
            self.g.get(state_id, INF)
            != self.rhs.get(state_id, INF)
        ):

            self.push_to_open(state_id)

    # ======================================================
    # SHORTEST PATH
    # ======================================================

    def compute_shortest_path(self):
        """
        Compute the shortest consistent path.

        Existing g/rhs values are reused during replanning.
        """

        if not self.initialized:
            return

        start = self.problem.initial_state

        while self.open_list:

            top_key = (
                self.open_list[0][0],
                self.open_list[0][1],
            )

            start_key = self.calculate_key(start)

            if (
                top_key >= start_key
                and self.g[start] == self.rhs[start]
            ):
                break

            old_key_0, old_key_1, current = (
                heapq.heappop(self.open_list)
            )

            current_key = self.calculate_key(
                current
            )

            # Lazy deletion of obsolete entries.
            if (
                old_key_0 != current_key[0]
                or old_key_1 != current_key[1]
            ):
                continue

            self.explored_states += 1

            if self.g[current] > self.rhs[current]:

                self.g[current] = self.rhs[current]

                for transition in self.incoming(current):

                    self.update_vertex(
                        transition.from_state
                    )

            else:

                self.g[current] = INF

                self.update_vertex(current)

                for transition in self.incoming(current):

                    self.update_vertex(
                        transition.from_state
                    )

    # ======================================================
    # ENVIRONMENT UPDATES
    # ======================================================

    def update_transition(
        self,
        transition_id: int,
        available: bool
    ):
        """
        Change transition availability without rebuilding
        the entire D* Lite search.

        Both the source and affected predecessor states
        are updated.
        """

        if transition_id not in self.transitions:
            raise ValueError(
                f"Unknown transition ID: {transition_id}"
            )

        transition = self.transitions[
            transition_id
        ]

        transition.available = available

        affected_states = {
            transition.from_state,
            transition.to_state,
        }

        for state_id in affected_states:

            self.update_vertex(state_id)

        # Also update predecessors of the destination.
        for incoming_transition in self.incoming(
            transition.to_state
        ):

            self.update_vertex(
                incoming_transition.from_state
            )

    def add_transition(
        self,
        transition: Transition
    ):
        """
        Add a new transition and update affected vertices.
        """

        self.transitions[transition.id] = transition

        # Also update the problem representation.
        existing_ids = {
            item.id
            for item in self.problem.transitions
        }

        if transition.id not in existing_ids:

            self.problem.transitions.append(
                transition
            )

        self.update_vertex(
            transition.from_state
        )

    def remove_transition(
        self,
        transition_id: int
    ):
        """
        Remove a transition from the planner.
        """

        if transition_id not in self.transitions:
            return

        transition = self.transitions[
            transition_id
        ]

        del self.transitions[
            transition_id
        ]

        self.problem.transitions = [
            item
            for item in self.problem.transitions
            if item.id != transition_id
        ]

        self.update_vertex(
            transition.from_state
        )

    def update_goal(
        self,
        new_goal: int
    ):
        """
        Change the goal while retaining existing search
        information as much as possible.
        """

        if new_goal not in self.states:
            raise ValueError(
                f"Unknown goal state: {new_goal}"
            )

        old_goal = self.problem.goal_state

        if old_goal == new_goal:
            return

        self.problem.goal_state = new_goal

        # Old goal is no longer automatically zero.
        self.update_vertex(old_goal)

        # New goal becomes the root of the reverse search.
        self.rhs[new_goal] = 0.0

        self.push_to_open(new_goal)

    def update_bad_states(
        self,
        new_bad_states
    ):
        """
        Update bad states without rebuilding all g/rhs values.
        """

        old_bad_states = set(
            self.problem.bad_states
        )

        new_bad_states = set(
            new_bad_states
        )

        changed_states = (
            old_bad_states
            ^ new_bad_states
        )

        self.problem.bad_states = new_bad_states

        for state_id in changed_states:

            self.update_vertex(state_id)

            for transition in self.incoming(
                state_id
            ):

                self.update_vertex(
                    transition.from_state
                )

    # ======================================================
    # PATH EXTRACTION
    # ======================================================

    def extract_path(
        self
    ) -> Tuple[List[int], List[int]]:

        start = self.problem.initial_state
        goal = self.problem.goal_state

        if start not in self.states:
            return [], []

        if goal not in self.states:
            return [], []

        if start in self.problem.bad_states:
            return [], []

        if goal in self.problem.bad_states:
            return [], []

        if self.g.get(start, INF) == INF:
            return [], []

        state_path = [start]
        transition_path = []

        current = start

        visited = {current}

        # Safety limit prevents malformed graphs from
        # causing an infinite loop.
        max_steps = len(self.states) + 1

        for _ in range(max_steps):

            if current == goal:
                return (
                    state_path,
                    transition_path
                )

            candidates = self.outgoing(
                current
            )

            if not candidates:
                return [], []

            best_transition = min(
                candidates,
                key=lambda transition: (
                    self.transition_cost(
                        transition
                    )
                    + self.g.get(
                        transition.to_state,
                        INF
                    )
                ),
            )

            next_state = (
                best_transition.to_state
            )

            if next_state in visited:
                return [], []

            transition_path.append(
                best_transition.id
            )

            state_path.append(
                next_state
            )

            visited.add(next_state)

            current = next_state

        return [], []

    # ======================================================
    # METRICS
    # ======================================================

    def calculate_result_metrics(
        self,
        state_path: List[int],
        transition_path: List[int]
    ):

        total_cost = 0.0

        reliability = 1.0

        for transition_id in transition_path:

            transition = self.transitions[
                transition_id
            ]

            total_cost += transition.cost

            reliability *= transition.reliability

        safety = minimum_path_safety(
            state_path,
            self.states,
            self.problem.bad_states,
        )

        return (
            total_cost,
            safety,
            reliability,
        )

    # ======================================================
    # INITIAL PLAN
    # ======================================================

    def plan(self) -> PlanningResult:

        start_time = time.perf_counter()

        # Validate states.
        if (
            self.problem.initial_state
            not in self.states
        ):

            return PlanningResult(
                success=False,
                message="Initial state does not exist.",
            )

        if (
            self.problem.goal_state
            not in self.states
        ):

            return PlanningResult(
                success=False,
                message="Goal state does not exist.",
            )

        if (
            self.problem.initial_state
            in self.problem.bad_states
        ):

            return PlanningResult(
                success=False,
                message="Initial state is a bad state.",
            )

        if (
            self.problem.goal_state
            in self.problem.bad_states
        ):

            return PlanningResult(
                success=False,
                message="Goal state is a bad state.",
            )

        if not self.initialized:
            self.initialize()

        self.compute_shortest_path()

        state_path, transition_path = (
            self.extract_path()
        )

        elapsed_ms = (
            time.perf_counter()
            - start_time
        ) * 1000.0

        if not state_path:

            return PlanningResult(
                success=False,
                explored_states=self.explored_states,
                planning_time=elapsed_ms,
                message="No safe path exists.",
            )

        (
            total_cost,
            safety,
            reliability,
        ) = self.calculate_result_metrics(
            state_path,
            transition_path,
        )

        return PlanningResult(
            success=True,
            state_path=state_path,
            transition_path=transition_path,
            total_cost=total_cost,
            safety_score=safety,
            reliability_score=reliability,
            explored_states=self.explored_states,
            planning_time=elapsed_ms,
            message="Safe path found.",
        )

    # ======================================================
    # INCREMENTAL REPLAN
    # ======================================================

    def replan(self) -> PlanningResult:

        start_time = time.perf_counter()

        if not self.initialized:
            self.initialize()

        self.compute_shortest_path()

        state_path, transition_path = (
            self.extract_path()
        )

        elapsed_ms = (
            time.perf_counter()
            - start_time
        ) * 1000.0

        if not state_path:

            return PlanningResult(
                success=False,
                explored_states=self.explored_states,
                planning_time=elapsed_ms,
                message="No safe path exists after update.",
            )

        (
            total_cost,
            safety,
            reliability,
        ) = self.calculate_result_metrics(
            state_path,
            transition_path,
        )

        return PlanningResult(
            success=True,
            state_path=state_path,
            transition_path=transition_path,
            total_cost=total_cost,
            safety_score=safety,
            reliability_score=reliability,
            explored_states=self.explored_states,
            planning_time=elapsed_ms,
            message="Incremental replanning completed.",
        )


# ==========================================================
# CONVENIENCE FUNCTION
# ==========================================================

def plan_problem(
    problem: PlanningProblem,
    cost_weight: float = 1.0,
    safety_weight: float = 0.0,
    reliability_weight: float = 0.0,
) -> PlanningResult:

    planner = DStarLite(
        problem=problem,
        cost_weight=cost_weight,
        safety_weight=safety_weight,
        reliability_weight=reliability_weight,
    )

    return planner.plan()