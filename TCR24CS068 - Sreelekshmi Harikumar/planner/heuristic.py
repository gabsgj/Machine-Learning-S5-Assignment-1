from .models import State
from .safety import euclidean_distance


def euclidean_heuristic(
    current: State,
    goal: State
) -> float:
    """
    Euclidean distance between the current
    state and the goal state.

    This is used as the heuristic function
    for the planner.
    """

    return euclidean_distance(
        current.embedding,
        goal.embedding
    )