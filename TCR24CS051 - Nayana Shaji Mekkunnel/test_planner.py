import math
import unittest

from safe_planner import PlanningProblem, SafeDStarLitePlanner, State, Transition


def states(*names):
    return [State(i, (float(i), 0.0)) for i, _ in enumerate(names)]


def edge(i, source, target, cost=1, **kwargs):
    return Transition(i, source, target, cost, **kwargs)


class SafePlannerTests(unittest.TestCase):
    def test_1_basic_reachability(self):
        planner = SafeDStarLitePlanner(PlanningProblem(0, 3, states("S", "A", "B", "G"),
            [edge(0, 0, 1), edge(1, 1, 2), edge(2, 2, 3)]))
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertEqual(result.state_path, [0, 1, 2, 3])

    def test_2_bad_state_avoidance(self):
        planner = SafeDStarLitePlanner(PlanningProblem(0, 5, states("S", "A", "X", "C", "D", "G"),
            [edge(0, 0, 1), edge(1, 1, 2), edge(2, 2, 5), edge(3, 0, 3), edge(4, 3, 4), edge(5, 4, 5)], {2}))
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertEqual(result.state_path, [0, 3, 4, 5])
        self.assertNotIn(2, result.state_path)

    def test_3_safety_margin_tradeoff(self):
        graph_states = [
            State(0, (0, 0)), State(1, (1, 0)), State(2, (1, 0.1)),
            State(3, (0, 10)), State(4, (10, 10)), State(5, (10, 0)),
        ]
        graph = PlanningProblem(0, 5, graph_states,
            [edge(0, 0, 1, 1), edge(1, 1, 5, 1), edge(2, 0, 3, 2), edge(3, 3, 4, 2), edge(4, 4, 5, 2)], {2})
        result = SafeDStarLitePlanner(graph, safety_weight=10).plan()
        self.assertTrue(result.success)
        self.assertEqual(result.state_path, [0, 3, 4, 5])
        self.assertGreater(result.minimum_safety_distance, 1.0)

    def test_4_dynamic_transition_removal(self):
        planner = SafeDStarLitePlanner(PlanningProblem(0, 3, states("S", "A", "B", "G"),
            [edge(0, 0, 1), edge(1, 1, 3), edge(2, 0, 2, 2), edge(3, 2, 3, 2)]))
        self.assertEqual(planner.plan().state_path, [0, 1, 3])
        planner.update_transition(1, available=False)
        self.assertEqual(planner.plan().state_path, [0, 2, 3])

    def test_5_goal_update(self):
        planner = SafeDStarLitePlanner(PlanningProblem(0, 3, states("S", "A", "B", "G"),
            [edge(0, 0, 1), edge(1, 1, 3), edge(2, 1, 2)]))
        planner.update_goal(2)
        self.assertEqual(planner.plan().state_path, [0, 1, 2])

    def test_6_transition_addition(self):
        planner = SafeDStarLitePlanner(PlanningProblem(0, 3, states("S", "A", "B", "G"),
            [edge(0, 0, 1, 3), edge(1, 1, 3, 3), edge(2, 0, 2, 4), edge(3, 2, 3, 4)]))
        self.assertEqual(planner.plan().total_cost, 6)
        planner.add_transition(edge(4, 0, 3, 1))
        self.assertEqual(planner.plan().state_path, [0, 3])


if __name__ == "__main__":
    unittest.main(verbosity=2)