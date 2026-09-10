from dataclasses import dataclass


@dataclass
class State:
    id: int
    embedding: list


@dataclass
class Transition:
    id: int
    from_state: int
    to_state: int
    cost: float
    safety: float
    reliability: float
    available: bool = True


@dataclass
class PlanningProblem:
    initial_state: int
    goal_state: int
    bad_states: set
    states: list
    transitions: list


@dataclass
class PlanningResult:
    success: bool
    state_path: list
    transition_path: list
    total_cost: float
    safety_score: float
    explored_states: int
    planning_time: float
    memory_usage: int
