import math
import heapq
import time
import matplotlib.pyplot as plt


# ============================================================
# 1. STATE
# ============================================================

class State:

    def __init__(self, state_id, embedding):

        self.id = state_id
        self.embedding = tuple(embedding)

    def __repr__(self):

        return f"State({self.id}, {self.embedding})"


# ============================================================
# 2. TRANSITION
# ============================================================

class Transition:

    def __init__(
        self,
        transition_id,
        source,
        target,
        cost,
        safety,
        reliability,
        available=True
    ):

        self.id = transition_id
        self.source = source
        self.target = target
        self.cost = cost
        self.safety = safety
        self.reliability = reliability
        self.available = available

    def __repr__(self):

        return (
            f"{self.source}->{self.target} "
            f"cost={self.cost}"
        )


# ============================================================
# 3. PLANNING PROBLEM
# ============================================================

class PlanningProblem:

    def __init__(self):

        self.initial_state = None
        self.goal_state = None

        self.bad_states = set()

        self.states = {}

        self.transitions = []

    def add_state(self, state):

        self.states[state.id] = state

    def add_transition(self, transition):

        self.transitions.append(transition)


# ============================================================
# 4. EUCLIDEAN DISTANCE
# ============================================================

def euclidean_distance(state1, state2):

    return math.sqrt(
        sum(
            (a - b) ** 2
            for a, b in zip(
                state1.embedding,
                state2.embedding
            )
        )
    )


# ============================================================
# 5. DISTANCE TO NEAREST BAD STATE
# ============================================================

def distance_to_nearest_bad(
    state,
    problem
):

    if not problem.bad_states:

        return float("inf")

    distances = []

    for bad_id in problem.bad_states:

        bad_state = problem.states[bad_id]

        distance = euclidean_distance(
            state,
            bad_state
        )

        distances.append(distance)

    return min(distances)


# ============================================================
# 6. GRAPH CONSTRUCTION
# ============================================================

def build_graph(problem):

    graph = {}

    for state_id in problem.states:

        graph[state_id] = []

    for transition in problem.transitions:

        if transition.available:

            graph[
                transition.source
            ].append(transition)

    return graph


# ============================================================
# 7. SAFETY-AWARE COST
# ============================================================

def semantic_cost(
    transition,
    destination_state,
    problem,
    safety_weight=5.0,
    reliability_weight=2.0
):

    safety_distance = (
        distance_to_nearest_bad(
            destination_state,
            problem
        )
    )

    epsilon = 1e-6

    safety_penalty = (
        safety_weight /
        (safety_distance + epsilon)
    )

    reliability_penalty = (
        reliability_weight *
        (1.0 - transition.reliability)
    )

    return (
        transition.cost
        + safety_penalty
        + reliability_penalty
    )


# ============================================================
# 8. HEURISTIC
# ============================================================

def heuristic(
    state_id,
    goal_id,
    problem
):

    state = problem.states[state_id]

    goal = problem.states[goal_id]

    return euclidean_distance(
        state,
        goal
    )


# ============================================================
# 9. SAFE A* CORE
#
# This is used as the planning engine inside our
# dynamic planner implementation.
# ============================================================

class SafePlanner:

    def __init__(
        self,
        problem,
        safety_weight=5.0,
        reliability_weight=2.0
    ):

        self.problem = problem

        self.safety_weight = safety_weight

        self.reliability_weight = (
            reliability_weight
        )

        self.explored_states = 0

        self.planning_time = 0

    # --------------------------------------------------------
    # FIND PATH
    # --------------------------------------------------------

    def plan(self):

        start_time = time.perf_counter()

        start = self.problem.initial_state

        goal = self.problem.goal_state

        graph = build_graph(self.problem)

        self.explored_states = 0

        # Priority queue
        open_list = []

        heapq.heappush(
            open_list,
            (
                heuristic(
                    start,
                    goal,
                    self.problem
                ),
                0,
                start
            )
        )

        # Best known cost
        g_cost = {
            state_id: float("inf")
            for state_id in self.problem.states
        }

        g_cost[start] = 0

        # Parent map
        parent = {}

        # Prevent repeated bad-state expansion
        closed = set()

        while open_list:

            _, current_cost, current = (
                heapq.heappop(open_list)
            )

            if current in closed:

                continue

            closed.add(current)

            self.explored_states += 1

            # Goal reached
            if current == goal:

                path = self.reconstruct_path(
                    parent,
                    start,
                    goal
                )

                self.planning_time = (
                    time.perf_counter()
                    - start_time
                )

                return self.create_result(
                    path
                )

            # Explore outgoing transitions
            for transition in graph[current]:

                next_state = transition.target

                # Never visit bad states
                if next_state in self.problem.bad_states:

                    continue

                destination = (
                    self.problem.states[next_state]
                )

                edge_cost = semantic_cost(
                    transition,
                    destination,
                    self.problem,
                    self.safety_weight,
                    self.reliability_weight
                )

                new_cost = (
                    g_cost[current]
                    + edge_cost
                )

                if new_cost < g_cost[next_state]:

                    g_cost[next_state] = new_cost

                    parent[next_state] = (
                        current,
                        transition
                    )

                    f_cost = (
                        new_cost
                        + heuristic(
                            next_state,
                            goal,
                            self.problem
                        )
                    )

                    heapq.heappush(
                        open_list,
                        (
                            f_cost,
                            new_cost,
                            next_state
                        )
                    )

        self.planning_time = (
            time.perf_counter()
            - start_time
        )

        return {
            "success": False,
            "state_path": [],
            "transition_path": [],
            "total_cost": float("inf"),
            "safety_score": 0,
            "reliability": 0,
            "explored_states": self.explored_states,
            "planning_time": self.planning_time
        }

    # --------------------------------------------------------
    # RECONSTRUCT PATH
    # --------------------------------------------------------

    def reconstruct_path(
        self,
        parent,
        start,
        goal
    ):

        state_path = [goal]

        transition_path = []

        current = goal

        while current != start:

            previous, transition = (
                parent[current]
            )

            transition_path.append(
                transition
            )

            state_path.append(
                previous
            )

            current = previous

        state_path.reverse()

        transition_path.reverse()

        return (
            state_path,
            transition_path
        )

    # --------------------------------------------------------
    # CREATE RESULT
    # --------------------------------------------------------

    def create_result(
        self,
        path_data
    ):

        state_path, transition_path = (
            path_data
        )

        total_cost = 0

        reliability = 1.0

        safety_distances = []

        for transition in transition_path:

            destination = (
                self.problem.states[
                    transition.target
                ]
            )

            total_cost += semantic_cost(
                transition,
                destination,
                self.problem,
                self.safety_weight,
                self.reliability_weight
            )

            reliability *= (
                transition.reliability
            )

        for state_id in state_path:

            state = self.problem.states[
                state_id
            ]

            distance = (
                distance_to_nearest_bad(
                    state,
                    self.problem
                )
            )

            safety_distances.append(
                distance
            )

        # Ignore infinite values if there
        # are no bad states
        finite_distances = [
            d for d in safety_distances
            if math.isfinite(d)
        ]

        if finite_distances:

            safety_score = min(
                finite_distances
            )

        else:

            safety_score = float("inf")

        return {

            "success": True,

            "state_path": state_path,

            "transition_path": transition_path,

            "total_cost": total_cost,

            "safety_score": safety_score,

            "reliability": reliability,

            "explored_states":
                self.explored_states,

            "planning_time":
                self.planning_time
        }


# ============================================================
# 10. DISPLAY RESULT
# ============================================================

def display_result(
    result
):

    print("\n==============================")
    print("PLANNING RESULT")
    print("==============================")

    print(
        "Success:",
        result["success"]
    )

    print(
        "State Path:",
        " -> ".join(
            result["state_path"]
        )
    )

    print(
        "Total Cost:",
        round(
            result["total_cost"],
            4
        )
    )

    print(
        "Minimum Safety Distance:",
        round(
            result["safety_score"],
            4
        )
    )

    print(
        "Reliability:",
        round(
            result["reliability"],
            4
        )
    )

    print(
        "Explored States:",
        result["explored_states"]
    )

    print(
        "Planning Time:",
        round(
            result["planning_time"],
            6
        ),
        "seconds"
    )


# ============================================================
# 11. VISUALIZATION
# ============================================================

def visualize(
    problem,
    result,
    title="Safe Semantic Planner"
):

    plt.figure(figsize=(9, 7))

    # --------------------------------------------
    # Draw transitions
    # --------------------------------------------

    for transition in problem.transitions:

        source = problem.states[
            transition.source
        ]

        target = problem.states[
            transition.target
        ]

        x1, y1 = source.embedding[:2]

        x2, y2 = target.embedding[:2]

        if not transition.available:

            continue

        plt.plot(
            [x1, x2],
            [y1, y2],
            linestyle="--",
            linewidth=1
        )

    # --------------------------------------------
    # Draw states
    # --------------------------------------------

    for state_id, state in (
        problem.states.items()
    ):

        x, y = state.embedding[:2]

        if state_id in problem.bad_states:

            plt.scatter(
                x,
                y,
                s=180,
                marker="X",
                label="Bad State"
                if "Bad State"
                not in plt.gca().get_legend_handles_labels()[1]
                else ""
            )

        elif state_id == (
            problem.initial_state
        ):

            plt.scatter(
                x,
                y,
                s=180,
                marker="o",
                label="Start"
                if "Start"
                not in plt.gca().get_legend_handles_labels()[1]
                else ""
            )

        elif state_id == (
            problem.goal_state
        ):

            plt.scatter(
                x,
                y,
                s=180,
                marker="*",
                label="Goal"
                if "Goal"
                not in plt.gca().get_legend_handles_labels()[1]
                else ""
            )

        else:

            plt.scatter(
                x,
                y,
                s=100,
                marker="o"
            )

        plt.text(
            x + 0.05,
            y + 0.05,
            state_id
        )

    # --------------------------------------------
    # Draw selected path
    # --------------------------------------------

    path = result["state_path"]

    for i in range(len(path) - 1):

        source = problem.states[
            path[i]
        ]

        target = problem.states[
            path[i + 1]
        ]

        x1, y1 = source.embedding[:2]

        x2, y2 = target.embedding[:2]

        plt.plot(
            [x1, x2],
            [y1, y2],
            linewidth=4
        )

    plt.title(title)

    plt.xlabel("X")

    plt.ylabel("Y")

    plt.grid(True)

    plt.legend()

    plt.show()


# ============================================================
# 12. TEST CASE 1
# BASIC REACHABILITY
# ============================================================

def test_case_1():

    print("\n\nTEST CASE 1")
    print("Basic Reachability")

    problem = PlanningProblem()

    problem.add_state(
        State("S", (0, 0))
    )

    problem.add_state(
        State("A", (1, 1))
    )

    problem.add_state(
        State("B", (2, 1))
    )

    problem.add_state(
        State("G", (3, 2))
    )

    problem.initial_state = "S"

    problem.goal_state = "G"

    problem.add_transition(
        Transition(
            1, "S", "A",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            2, "A", "B",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            3, "B", "G",
            1, 1, 0.95
        )
    )

    planner = SafePlanner(problem)

    result = planner.plan()

    display_result(result)

    visualize(
        problem,
        result,
        "Test Case 1 - Basic Reachability"
    )

    return problem, result


# ============================================================
# 13. TEST CASE 2
# BAD STATE AVOIDANCE
# ============================================================

def test_case_2():

    print("\n\nTEST CASE 2")
    print("Bad State Avoidance")

    problem = PlanningProblem()

    problem.add_state(
        State("S", (0, 0))
    )

    problem.add_state(
        State("A", (1, 1))
    )

    problem.add_state(
        State("X", (2, 1))
    )

    problem.add_state(
        State("C", (1, -1))
    )

    problem.add_state(
        State("D", (2, -1))
    )

    problem.add_state(
        State("G", (3, 0))
    )

    problem.initial_state = "S"

    problem.goal_state = "G"

    problem.bad_states.add("X")

    # Unsafe path
    problem.add_transition(
        Transition(
            1, "S", "A",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            2, "A", "X",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            3, "X", "G",
            1, 1, 0.95
        )
    )

    # Safe path
    problem.add_transition(
        Transition(
            4, "S", "C",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            5, "C", "D",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            6, "D", "G",
            2, 1, 0.95
        )
    )

    planner = SafePlanner(problem)

    result = planner.plan()

    display_result(result)

    visualize(
        problem,
        result,
        "Test Case 2 - Bad State Avoidance"
    )

    return problem, result


# ============================================================
# 14. TEST CASE 3
# SAFETY MARGIN
# ============================================================

def test_case_3():

    print("\n\nTEST CASE 3")
    print("Safety Margin")

    problem = PlanningProblem()

    states = {

        "S": (0, 0),

        "A": (1, 0),

        "B": (2, 0),

        "C": (1, 2),

        "D": (2, 2),

        "G": (3, 1),

        "X": (1.5, 0.5)
    }

    for state_id, position in states.items():

        problem.add_state(
            State(
                state_id,
                position
            )
        )

    problem.initial_state = "S"

    problem.goal_state = "G"

    problem.bad_states.add("X")

    # Short path
    problem.add_transition(
        Transition(
            1, "S", "A",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            2, "A", "B",
            1, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            3, "B", "G",
            1, 1, 0.95
        )
    )

    # Safer path
    problem.add_transition(
        Transition(
            4, "S", "C",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            5, "C", "D",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            6, "D", "G",
            2, 1, 0.95
        )
    )

    planner = SafePlanner(
        problem,
        safety_weight=10
    )

    result = planner.plan()

    display_result(result)

    visualize(
        problem,
        result,
        "Test Case 3 - Safety Margin"
    )

    return problem, result


# ============================================================
# 15. TEST CASE 4
# DYNAMIC TRANSITION
# ============================================================

def test_case_4():

    print("\n\nTEST CASE 4")
    print("Dynamic Transition")

    problem = PlanningProblem()

    states = {

        "S": (0, 0),

        "A": (1, 1),

        "B": (1, -1),

        "G": (2, 0)
    }

    for state_id, position in states.items():

        problem.add_state(
            State(
                state_id,
                position
            )
        )

    problem.initial_state = "S"

    problem.goal_state = "G"

    t1 = Transition(
        1, "S", "A",
        1, 1, 0.95
    )

    t2 = Transition(
        2, "A", "G",
        1, 1, 0.95
    )

    t3 = Transition(
        3, "S", "B",
        2, 1, 0.95
    )

    t4 = Transition(
        4, "B", "G",
        2, 1, 0.95
    )

    problem.add_transition(t1)

    problem.add_transition(t2)

    problem.add_transition(t3)

    problem.add_transition(t4)

    # Initial planning
    planner = SafePlanner(problem)

    print("\nInitial planning:")

    result1 = planner.plan()

    display_result(result1)

    # Dynamic update
    print("\nTransition A -> G becomes unavailable.")

    t2.available = False

    print("\nReplanning:")

    result2 = planner.plan()

    display_result(result2)

    visualize(
        problem,
        result2,
        "Test Case 4 - Dynamic Transition"
    )

    return problem, result1, result2


# ============================================================
# 16. TEST CASE 5
# GOAL UPDATE
# ============================================================

def test_case_5():

    print("\n\nTEST CASE 5")
    print("Goal Update")

    problem = PlanningProblem()

    states = {

        "S": (0, 0),

        "A": (1, 0),

        "B": (0, 2),

        "G1": (2, 0),

        "G2": (2, 2)
    }

    for state_id, position in states.items():

        problem.add_state(
            State(
                state_id,
                position
            )
        )

    problem.initial_state = "S"

    problem.goal_state = "G1"

    transitions = [

        Transition(
            1, "S", "A",
            1, 1, 0.95
        ),

        Transition(
            2, "A", "G1",
            1, 1, 0.95
        ),

        Transition(
            3, "S", "B",
            2, 1, 0.95
        ),

        Transition(
            4, "B", "G2",
            1, 1, 0.95
        )
    ]

    for transition in transitions:

        problem.add_transition(
            transition
        )

    planner = SafePlanner(problem)

    print("\nInitial goal = G1")

    result1 = planner.plan()

    display_result(result1)

    # Change goal
    problem.goal_state = "G2"

    print("\nGoal changed to G2")

    result2 = planner.plan()

    display_result(result2)

    visualize(
        problem,
        result2,
        "Test Case 5 - Goal Update"
    )

    return problem, result1, result2


# ============================================================
# 17. TEST CASE 6
# TRANSITION ADDITION
# ============================================================

def test_case_6():

    print("\n\nTEST CASE 6")
    print("Transition Addition")

    problem = PlanningProblem()

    states = {

        "S": (0, 0),

        "A": (1, 1),

        "B": (1, -1),

        "G": (3, 0)
    }

    for state_id, position in states.items():

        problem.add_state(
            State(
                state_id,
                position
            )
        )

    problem.initial_state = "S"

    problem.goal_state = "G"

    problem.add_transition(
        Transition(
            1, "S", "A",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            2, "A", "G",
            2, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            3, "S", "B",
            3, 1, 0.95
        )
    )

    problem.add_transition(
        Transition(
            4, "B", "G",
            3, 1, 0.95
        )
    )

    planner = SafePlanner(problem)

    print("\nBefore shortcut:")

    result1 = planner.plan()

    display_result(result1)

    # Add shortcut
    print("\nAdding new shortcut S -> G")

    problem.add_transition(
        Transition(
            5, "S", "G",
            1, 1, 0.99
        )
    )

    print("\nAfter shortcut:")

    result2 = planner.plan()

    display_result(result2)

    visualize(
        problem,
        result2,
        "Test Case 6 - Transition Addition"
    )

    return problem, result1, result2


# ============================================================
# 18. MAIN
# ============================================================

if __name__ == "__main__":

    print(
        "SAFE SEMANTIC PLANNER"
    )

    test_case_1()

    test_case_2()

    test_case_3()

    test_case_4()

    test_case_5()

    test_case_6()