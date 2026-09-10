# planner.py
# A* based planner that finds safe paths while avoiding bad states.
# Includes safety distance calculation, scoring function, and replanning.

import heapq
import math
import time
from structures import PlanningProblem, PlanningResult


# ============================================================
# Helper function: Euclidean distance between two points
# ============================================================
def euclidean_distance(point1, point2):
    """Calculate the straight-line distance between two points."""
    total = 0.0
    for i in range(len(point1)):
        diff = point1[i] - point2[i]
        total += diff * diff
    return math.sqrt(total)


# ============================================================
# Helper function: Calculate min distance from a state to bad states
# ============================================================
def min_distance_to_bad_states(state_embedding, bad_state_embeddings):
    """
    Find the smallest Euclidean distance from a state to any bad state.
    If there are no bad states, returns infinity (meaning perfectly safe).
    """
    if len(bad_state_embeddings) == 0:
        return float('inf')

    smallest = float('inf')
    for bad_emb in bad_state_embeddings:
        dist = euclidean_distance(state_embedding, bad_emb)
        if dist < smallest:
            smallest = dist
    return smallest


# ============================================================
# Scoring function from the assignment
# Score(P) = alpha*G - beta*C + gamma*D + delta*R
# ============================================================
def compute_score(goal_reached, total_cost, min_safety_distance, total_reliability,
                  alpha=10.0, beta=1.0, gamma=2.0, delta=1.0):
    """
    Compute the objective score for a path.
    - G = 1 if goal reached, 0 otherwise
    - C = total transition cost (lower is better, so we subtract)
    - D = minimum safety distance to bad states (higher is better)
    - R = total reliability of transitions used (higher is better)
    """
    G = 1.0 if goal_reached else 0.0
    # Cap the safety distance so it doesn't blow up when there are no bad states
    D = min(min_safety_distance, 100.0)
    score = alpha * G - beta * total_cost + gamma * D + delta * total_reliability
    return round(score, 4)


# ============================================================
# The main A* Planner class
# ============================================================
class SafePlanner:
    """
    A safe path planner using A* search.

    It finds the cheapest path from start to goal while:
    1. Avoiding all bad states
    2. Skipping unavailable transitions
    3. Tracking safety distance (how far the path stays from bad states)
    4. Computing a combined score using the objective function

    For replanning (dynamic environments), just modify the PlanningProblem
    and call plan() again. This is the simplest approach for beginners.
    """

    def plan(self, problem):
        """
        Run A* search to find a safe path.
        Returns a PlanningResult with the path, cost, safety, and metrics.
        """
        start_time = time.time()

        # ----------------------------------------------------------
        # Step 1: Build lookup tables for quick access
        # ----------------------------------------------------------

        # Map state ID -> State object (so we can get embeddings fast)
        state_map = {}
        for s in problem.states:
            state_map[s.id] = s

        # Get the goal state's position for the heuristic
        goal_embedding = state_map[problem.goalState].embedding

        # Get all bad state positions for safety distance calculation
        bad_state_embeddings = []
        for bad_id in problem.badStates:
            bad_state_embeddings.append(state_map[bad_id].embedding)

        # Build adjacency list: state_id -> list of transitions leaving it
        adjacency = {}
        for t in problem.transitions:
            if t.from_state not in adjacency:
                adjacency[t.from_state] = []
            adjacency[t.from_state].append(t)

        # Also make a map of transition ID -> Transition for later lookup
        transition_map = {}
        for t in problem.transitions:
            transition_map[t.id] = t

        # ----------------------------------------------------------
        # Step 2: Set up A* data structures
        # ----------------------------------------------------------

        # The open set is a priority queue (min-heap)
        # Each entry is (f_score, counter, state_id)
        # Counter is used to break ties (so heapq doesn't compare state IDs)
        counter = 0
        open_set = []
        heapq.heappush(open_set, (0.0, counter, problem.initialState))
        counter += 1

        # g_score[state_id] = cheapest known cost to reach that state
        g_score = {}
        g_score[problem.initialState] = 0.0

        # came_from[state_id] = (parent_state_id, transition_id)
        # Used to reconstruct the path at the end
        came_from = {}

        # States we've already fully explored
        closed_set = set()

        # Count how many states we explore (for evaluation)
        explored_count = 0

        # ----------------------------------------------------------
        # Step 3: Run A* search
        # ----------------------------------------------------------

        while len(open_set) > 0:
            # Get the state with the lowest f_score
            f, _, current = heapq.heappop(open_set)

            # Skip if we already explored this state
            if current in closed_set:
                continue

            # Skip bad states (never visit them!)
            if current in problem.badStates:
                closed_set.add(current)
                continue

            explored_count += 1

            # Check if we reached the goal!
            if current == problem.goalState:
                # Reconstruct the path by following parent pointers backwards
                state_path = []
                transition_path = []
                step = current

                while step in came_from:
                    parent, trans_id = came_from[step]
                    state_path.append(step)
                    transition_path.append(trans_id)
                    step = parent

                # Add the start state
                state_path.append(problem.initialState)

                # Reverse because we built it backwards
                state_path.reverse()
                transition_path.reverse()

                # Calculate total cost
                total_cost = g_score[current]

                # Calculate cumulative reliability
                total_reliability = 0.0
                for tid in transition_path:
                    total_reliability += transition_map[tid].reliability

                # Calculate minimum safety distance along the path
                # (How close does the path get to any bad state?)
                min_safety_dist = float('inf')
                for sid in state_path:
                    dist = min_distance_to_bad_states(
                        state_map[sid].embedding,
                        bad_state_embeddings
                    )
                    if dist < min_safety_dist:
                        min_safety_dist = dist

                # Compute the objective score
                score = compute_score(
                    goal_reached=True,
                    total_cost=total_cost,
                    min_safety_distance=min_safety_dist,
                    total_reliability=total_reliability
                )

                end_time = time.time()
                planning_time = round(end_time - start_time, 6)

                return PlanningResult(
                    success=True,
                    statePath=state_path,
                    transitionPath=transition_path,
                    totalCost=round(total_cost, 4),
                    safetyScore=round(min_safety_dist, 4),
                    exploredStates=explored_count,
                    planningTime=planning_time,
                    reliabilityScore=round(total_reliability, 4),
                    score=score
                )

            # Mark current state as explored
            closed_set.add(current)

            # ----------------------------------------------------------
            # Step 4: Explore neighbors
            # ----------------------------------------------------------
            neighbors = adjacency.get(current, [])

            for transition in neighbors:
                next_state = transition.to_state

                # Skip unavailable transitions
                if not transition.available:
                    continue

                # Skip already explored states
                if next_state in closed_set:
                    continue

                # Skip bad states
                if next_state in problem.badStates:
                    continue

                # Calculate cost to reach this neighbor through current
                new_cost = g_score[current] + transition.cost

                # Only update if this is a better path
                if next_state not in g_score or new_cost < g_score[next_state]:
                    g_score[next_state] = new_cost
                    came_from[next_state] = (current, transition.id)

                    # Heuristic: straight-line distance to goal
                    h = euclidean_distance(
                        state_map[next_state].embedding,
                        goal_embedding
                    )
                    f_score = new_cost + h

                    heapq.heappush(open_set, (f_score, counter, next_state))
                    counter += 1

        # ----------------------------------------------------------
        # No path found
        # ----------------------------------------------------------
        end_time = time.time()
        planning_time = round(end_time - start_time, 6)

        return PlanningResult(
            success=False,
            statePath=[],
            transitionPath=[],
            totalCost=0.0,
            safetyScore=0.0,
            exploredStates=explored_count,
            planningTime=planning_time,
            reliabilityScore=0.0,
            score=compute_score(False, 0, 0, 0)
        )


# ============================================================
# Replanning helper functions
# These modify the problem and re-run the planner.
# This is the simplest way to handle dynamic environments.
# ============================================================

def replan_with_unavailable_transition(planner, problem, from_state, to_state):
    """
    Make a transition unavailable and replan.
    Simulates a broken connection in the environment.
    """
    for t in problem.transitions:
        if t.from_state == from_state and t.to_state == to_state:
            t.available = False
            break
    return planner.plan(problem)


def replan_with_new_goal(planner, problem, new_goal):
    """
    Change the goal state and replan.
    Simulates the goal changing during execution.
    """
    problem.goalState = new_goal
    return planner.plan(problem)


def replan_with_new_transition(planner, problem, new_transition):
    """
    Add a new transition to the problem and replan.
    Simulates discovering a new shortcut.
    """
    problem.transitions.append(new_transition)
    return planner.plan(problem)