import math
from typing import Dict, List, Set

from .models import State


def euclidean_distance(
    point_a: List[float],
    point_b: List[float]
) -> float:
    """
    Calculate Euclidean distance between
    two points.
    """

    if len(point_a) != len(point_b):
        raise ValueError(
            "Points must have the same dimension."
        )

    return math.sqrt(
        sum(
            (a - b) ** 2
            for a, b in zip(point_a, point_b)
        )
    )


def distance_to_nearest_bad_state(
    state_id: int,
    states: Dict[int, State],
    bad_states: Set[int]
) -> float:
    """
    Calculate the distance from a state to
    its nearest bad state.
    """

    if not bad_states:
        return float("inf")

    if state_id not in states:
        raise ValueError(
            f"Unknown state ID: {state_id}"
        )

    current_state = states[state_id]

    distances = []

    for bad_id in bad_states:

        if bad_id not in states:
            continue

        bad_state = states[bad_id]

        distance = euclidean_distance(
            current_state.embedding,
            bad_state.embedding
        )

        distances.append(distance)

    if not distances:
        return float("inf")

    return min(distances)


def minimum_path_safety(
    path: List[int],
    states: Dict[int, State],
    bad_states: Set[int]
) -> float:
    """
    Calculate the minimum distance from
    any state on the path to the nearest
    bad state.
    """

    if not path:
        return 0.0

    if not bad_states:
        return float("inf")

    distances = []

    for state_id in path:

        distance = distance_to_nearest_bad_state(
            state_id,
            states,
            bad_states
        )

        distances.append(distance)

    return min(distances)