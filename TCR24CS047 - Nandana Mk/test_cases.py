from state import State
from transition import Transition


def build_test_graph(planner):
    """
    Creates the graph used for all test cases.
    """

    # ----------------------------
    # States
    # ----------------------------

    planner.add_state(State("S", 0, 0))
    planner.add_state(State("A", 1, 0))
    planner.add_state(State("B", 2, 0))
    planner.add_state(State("C", 1, 1))
    planner.add_state(State("D", 2, 1))
    planner.add_state(State("G", 3, 0))

    # ----------------------------
    # Transitions
    # ----------------------------

    planner.add_transition(Transition("S", "A", 2, 0.95, 0.90))
    planner.add_transition(Transition("A", "B", 2, 0.95, 0.90))
    planner.add_transition(Transition("B", "G", 2, 0.95, 0.90))

    planner.add_transition(Transition("S", "C", 3, 0.90, 0.95))
    planner.add_transition(Transition("C", "D", 2, 0.90, 0.95))
    planner.add_transition(Transition("D", "G", 2, 0.90, 0.95))