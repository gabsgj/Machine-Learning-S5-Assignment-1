import unittest
from planner import DStarLitePlanner, build_demo_problem


class TestSafeSemanticPlanner(unittest.TestCase):

    def make_planner(self):
        states, transitions = build_demo_problem()
        return DStarLitePlanner(states, transitions, 0, 3, {6})

    def test_1_basic_reachability(self):
        planner = self.make_planner()
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertEqual(result.state_path[0], 0)
        self.assertEqual(result.state_path[-1], 3)

    def test_2_bad_state_avoidance(self):
        planner = self.make_planner()
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertNotIn(6, result.state_path)

    def test_3_safety_margin(self):
        planner = self.make_planner()
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertGreater(result.safety_score, 0.0)

    def test_4_dynamic_transition(self):
        planner = self.make_planner()
        first = planner.plan()
        self.assertTrue(first.success)

        # Disable the direct S-A-G shortcut and force a replanning path.
        planner.update_transition(13, available=False)
        second = planner.plan()
        self.assertTrue(second.success)
        self.assertEqual(second.state_path[-1], 3)

    def test_5_goal_update(self):
        planner = self.make_planner()
        planner.set_goal(11)
        result = planner.plan()
        self.assertTrue(result.success)
        self.assertEqual(result.state_path[-1], 11)

    def test_6_transition_addition(self):
        planner = self.make_planner()
        planner.update_transition(13, available=False)
        before = planner.plan()
        self.assertTrue(before.success)

        # Insert a new shortcut to the goal.
        from planner import Transition
        planner.add_transition(
            Transition(99, 0, 3, 0.5, 0.99, 0.99, True)
        )
        after = planner.plan()
        self.assertTrue(after.success)
        self.assertEqual(after.state_path, [0, 3])
        self.assertAlmostEqual(after.total_cost, 0.5)

if __name__ == "__main__":
    unittest.main()
