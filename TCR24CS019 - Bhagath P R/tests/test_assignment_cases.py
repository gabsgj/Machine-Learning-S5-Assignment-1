"""
Explicit test cases from PCCST503 Assignment 1 specification:
- Test Case 1: Basic Reachability (S -> A -> B -> G)
- Test Case 2: Bad State Avoidance (S -> A -> X -> G vs S -> C -> D -> G)
- Test Case 3: Safety Margin (Low cost close to bad vs higher cost far from bad)
- Test Case 4: Dynamic Transition (Edge failure and dynamic replanning)
- Test Case 5: Goal Update (Goal modification and incremental replanning)
- Test Case 6: Transition Addition (Shortcut discovery upon edge insertion)
"""

import unittest
import numpy as np
from src.semantic_planner.models import State, Transition, PlanningProblem, ObjectiveWeights
from src.semantic_planner.lpa_star import LPAStarPlanner
from src.semantic_planner.d_star_lite import DStarLitePlanner
from src.semantic_planner.a_star import AStarPlanner
from src.semantic_planner.parallel_planner import ParallelBidirectionalPlanner


class TestAssignmentCases(unittest.TestCase):

    def test_case_1_basic_reachability(self):
        """
        Test Case 1: Basic Reachability
        Graph: S -> A -> B -> G
        Expected result: planner returns the unique valid path.
        """
        def make_problem():
            states = [
                State(id=1, embedding=[0.0, 0.0], name="S"),
                State(id=2, embedding=[1.0, 0.0], name="A"),
                State(id=3, embedding=[2.0, 0.0], name="B"),
                State(id=4, embedding=[3.0, 0.0], name="G"),
            ]
            transitions = [
                Transition(id=101, from_state=1, to_state=2, cost=1.0),
                Transition(id=102, from_state=2, to_state=3, cost=1.0),
                Transition(id=103, from_state=3, to_state=4, cost=1.0),
            ]
            return PlanningProblem(
                initial_state=1,
                goal_state=4,
                bad_states=[],
                states=states,
                transitions=transitions
            )

        for planner_cls in [LPAStarPlanner, DStarLitePlanner, AStarPlanner, ParallelBidirectionalPlanner]:
            problem = make_problem()
            planner = planner_cls()
            result = planner.plan(problem)

            self.assertTrue(result.success, f"{planner_cls.__name__} failed reachability")
            self.assertEqual(result.state_path, [1, 2, 3, 4])
            self.assertEqual(result.transition_path, [101, 102, 103])
            self.assertAlmostEqual(result.total_cost, 3.0)

    def test_case_2_bad_state_avoidance(self):
        """
        Test Case 2: Bad State Avoidance
        Paths:
          Path 1: S -> A -> X -> G (where X is a bad state)
          Path 2: S -> C -> D -> G
        Expected result: second path (S -> C -> D -> G) must be selected.
        """
        def make_problem():
            states = [
                State(id=1, embedding=[0.0, 0.0], name="S"),
                State(id=2, embedding=[1.0, 1.0], name="A"),
                State(id=3, embedding=[2.0, 1.0], name="X"),  # Bad State
                State(id=4, embedding=[1.0, -1.0], name="C"),
                State(id=5, embedding=[2.0, -1.0], name="D"),
                State(id=6, embedding=[3.0, 0.0], name="G"),
            ]
            transitions = [
                # Path 1 (traverses bad state X)
                Transition(id=201, from_state=1, to_state=2, cost=1.0),
                Transition(id=202, from_state=2, to_state=3, cost=1.0),
                Transition(id=203, from_state=3, to_state=6, cost=1.0),
                # Path 2 (safe alternative)
                Transition(id=204, from_state=1, to_state=4, cost=1.5),
                Transition(id=205, from_state=4, to_state=5, cost=1.5),
                Transition(id=206, from_state=5, to_state=6, cost=1.5),
            ]
            return PlanningProblem(
                initial_state=1,
                goal_state=6,
                bad_states=[3],
                states=states,
                transitions=transitions
            )

        for planner_cls in [LPAStarPlanner, DStarLitePlanner, AStarPlanner, ParallelBidirectionalPlanner]:
            problem = make_problem()
            planner = planner_cls()
            result = planner.plan(problem)

            self.assertTrue(result.success, f"{planner_cls.__name__} failed avoidance")
            self.assertEqual(result.state_path, [1, 4, 5, 6])
            self.assertEqual(result.transition_path, [204, 205, 206])
            self.assertNotIn(3, result.state_path, "Bad state X was visited!")

    def test_case_3_safety_margin(self):
        """
        Test Case 3: Safety Margin
        Two valid paths exist:
          Path 1: S -> M1 -> G (lower raw cost, but passes close to bad state B)
          Path 2: S -> M2 -> G (higher raw cost, but far from bad state B)
        Expected result: With safety margin objective, planner balances cost and clearance.
        """
        def make_problem(weights):
            states = [
                State(id=1, embedding=[0.0, 2.0], name="S"),
                State(id=2, embedding=[1.0, 0.2], name="M1"),
                State(id=3, embedding=[1.0, 3.5], name="M2"),
                State(id=4, embedding=[2.0, 2.0], name="G"),
                State(id=99, embedding=[1.0, 0.0], name="B"), # Bad state
            ]
            transitions = [
                # Path 1: raw cost = 1.0 + 1.0 = 2.0
                Transition(id=301, from_state=1, to_state=2, cost=1.0),
                Transition(id=302, from_state=2, to_state=4, cost=1.0),
                # Path 2: raw cost = 2.0 + 2.0 = 4.0
                Transition(id=303, from_state=1, to_state=3, cost=2.0),
                Transition(id=304, from_state=3, to_state=4, cost=2.0),
            ]
            return PlanningProblem(
                initial_state=1,
                goal_state=4,
                bad_states=[99],
                states=states,
                transitions=transitions,
                weights=weights
            )

        # 1. Safety-conscious planner picks Path 2 (M2) to maintain clearance from B
        weights_safe = ObjectiveWeights(beta=1.0, gamma=5.0, safety_margin=1.5, safety_penalty_coeff=20.0)
        planner_safe = LPAStarPlanner()
        result_safe = planner_safe.plan(make_problem(weights_safe))
        self.assertTrue(result_safe.success)
        self.assertEqual(result_safe.state_path, [1, 3, 4])
        self.assertGreater(result_safe.min_safety_distance, 1.5)

        # 2. Cost-only planner (gamma=0, penalty=0) picks lower cost Path 1 (M1)
        weights_cheap = ObjectiveWeights(beta=1.0, gamma=0.0, safety_margin=0.0, safety_penalty_coeff=0.0)
        planner_cheap = LPAStarPlanner()
        result_cheap = planner_cheap.plan(make_problem(weights_cheap))
        self.assertTrue(result_cheap.success)
        self.assertEqual(result_cheap.state_path, [1, 2, 4])

    def test_case_4_dynamic_transition(self):
        """
        Test Case 4: Dynamic Transition
        Initially: S -> A -> G is shortest (alternative: S -> B -> C -> G).
        Later: Transition (A, G) becomes unavailable.
        Expected result: Planner incrementally replans and discovers alternative path.
        """
        def make_problem():
            states = [
                State(id=1, embedding=[0.0, 0.0], name="S"),
                State(id=2, embedding=[1.0, 0.5], name="A"),
                State(id=3, embedding=[0.5, -1.0], name="B"),
                State(id=4, embedding=[1.5, -1.0], name="C"),
                State(id=5, embedding=[2.0, 0.0], name="G"),
            ]
            transitions = [
                Transition(id=401, from_state=1, to_state=2, cost=1.0),
                Transition(id=402, from_state=2, to_state=5, cost=1.0),  # Will be disabled
                Transition(id=403, from_state=1, to_state=3, cost=1.5),
                Transition(id=404, from_state=3, to_state=4, cost=1.5),
                Transition(id=405, from_state=4, to_state=5, cost=1.5),
            ]
            return PlanningProblem(
                initial_state=1,
                goal_state=5,
                bad_states=[],
                states=states,
                transitions=transitions
            )

        for planner_cls in [LPAStarPlanner, DStarLitePlanner]:
            problem = make_problem()
            planner = planner_cls()
            init_res = planner.plan(problem)
            self.assertTrue(init_res.success)
            self.assertEqual(init_res.state_path, [1, 2, 5])

            # Transition (2, 5) becomes unavailable
            planner.update_edge(2, 5, available=False)
            replan_res = planner.replan()

            self.assertTrue(replan_res.success, f"{planner_cls.__name__} failed dynamic replan")
            self.assertEqual(replan_res.state_path, [1, 3, 4, 5])
            self.assertEqual(replan_res.transition_path, [403, 404, 405])

    def test_case_5_goal_update(self):
        """
        Test Case 5: Goal Update
        Goal changes during execution (G1 -> G2).
        Expected result: Planner produces revised path without rebuilding all data structures.
        """
        states = [
            State(id=1, embedding=[0.0, 0.0], name="S"),
            State(id=2, embedding=[1.0, 0.0], name="A"),
            State(id=3, embedding=[2.0, 1.0], name="G1"),
            State(id=4, embedding=[2.0, -1.0], name="G2"),
        ]
        transitions = [
            Transition(id=501, from_state=1, to_state=2, cost=1.0),
            Transition(id=502, from_state=2, to_state=3, cost=1.0), # To G1
            Transition(id=503, from_state=2, to_state=4, cost=1.0), # To G2
        ]
        problem = PlanningProblem(
            initial_state=1,
            goal_state=3,
            bad_states=[],
            states=states,
            transitions=transitions
        )

        planner = LPAStarPlanner()
        res1 = planner.plan(problem)
        self.assertTrue(res1.success)
        self.assertEqual(res1.state_path, [1, 2, 3])

        # Goal updates to G2 (4)
        planner.update_goal(4)
        res2 = planner.replan()
        self.assertTrue(res2.success)
        self.assertEqual(res2.state_path, [1, 2, 4])

    def test_case_6_transition_addition(self):
        """
        Test Case 6: Transition Addition
        A new shortcut transition is inserted.
        Expected result: Planner discovers the improved shortcut solution.
        """
        states = [
            State(id=1, embedding=[0.0, 0.0], name="S"),
            State(id=2, embedding=[1.0, 0.0], name="A"),
            State(id=3, embedding=[2.0, 0.0], name="B"),
            State(id=4, embedding=[3.0, 0.0], name="G"),
        ]
        transitions = [
            Transition(id=601, from_state=1, to_state=2, cost=2.0),
            Transition(id=602, from_state=2, to_state=3, cost=2.0),
            Transition(id=603, from_state=3, to_state=4, cost=2.0),
        ]
        problem = PlanningProblem(
            initial_state=1,
            goal_state=4,
            bad_states=[],
            states=states,
            transitions=transitions
        )

        planner = LPAStarPlanner()
        res_initial = planner.plan(problem)
        self.assertTrue(res_initial.success)
        self.assertEqual(res_initial.state_path, [1, 2, 3, 4])
        self.assertAlmostEqual(res_initial.total_cost, 6.0)

        # Insert direct shortcut transition from S(1) to G(4) with cost 2.5
        shortcut = Transition(id=699, from_state=1, to_state=4, cost=2.5)
        planner.graph.add_transition(shortcut)
        planner.update_edge(1, 4, new_cost=2.5, available=True)

        res_shortcut = planner.replan()
        self.assertTrue(res_shortcut.success)
        self.assertEqual(res_shortcut.state_path, [1, 4])
        self.assertEqual(res_shortcut.transition_path, [699])
        self.assertAlmostEqual(res_shortcut.total_cost, 2.5)


if __name__ == "__main__":
    unittest.main()
