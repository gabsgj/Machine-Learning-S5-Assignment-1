"""
Comprehensive property and stress unit tests for LPA*, D* Lite, A*, and Parallel planners.
"""

import unittest
import numpy as np
from src.semantic_planner.models import State, Transition, PlanningProblem, ObjectiveWeights
from src.semantic_planner.graph import CartesianGraph
from src.semantic_planner.lpa_star import LPAStarPlanner
from src.semantic_planner.d_star_lite import DStarLitePlanner
from src.semantic_planner.a_star import AStarPlanner
from src.semantic_planner.parallel_planner import ParallelBidirectionalPlanner


class TestPlannersComprehensive(unittest.TestCase):

    def _create_grid_problem(self, rows=5, cols=5, bad_coords=None, dim=2):
        """Creates a Cartesian grid problem embedded in R^dim."""
        if bad_coords is None:
            bad_coords = []
        
        states = []
        state_map = {}
        sid = 1
        for r in range(rows):
            for c in range(cols):
                coords = [float(r), float(c)] + [0.0] * (dim - 2)
                s = State(id=sid, embedding=coords, name=f"({r},{c})")
                states.append(s)
                state_map[(r, c)] = sid
                sid += 1

        bad_states = [state_map[rc] for rc in bad_coords if rc in state_map]

        transitions = []
        tid = 1
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]
        for (r, c), u_id in state_map.items():
            for dr, dc in directions:
                nr, nc = r + dr, c + dc
                if (nr, nc) in state_map:
                    v_id = state_map[(nr, nc)]
                    transitions.append(
                        Transition(
                            id=tid,
                            from_state=u_id,
                            to_state=v_id,
                            cost=1.0,
                            safety=1.0,
                            reliability=0.98,
                            available=True
                        )
                    )
                    tid += 1

        start_id = state_map[(0, 0)]
        goal_id = state_map[(rows - 1, cols - 1)]

        return PlanningProblem(
            initial_state=start_id,
            goal_state=goal_id,
            bad_states=bad_states,
            states=states,
            transitions=transitions,
            weights=ObjectiveWeights(beta=1.0, gamma=2.0, safety_margin=1.5)
        )

    def test_grid_path_all_planners(self):
        """Test on 5x5 grid with obstacles across all 4 planners."""
        problem = self._create_grid_problem(rows=5, cols=5, bad_coords=[(1, 1), (2, 2), (3, 3)])
        
        planners = [
            ("LPA*", LPAStarPlanner()),
            ("D* Lite", DStarLitePlanner()),
            ("A*", AStarPlanner()),
            ("Parallel", ParallelBidirectionalPlanner()),
        ]

        results = {}
        for name, planner in planners:
            res = planner.plan(problem)
            self.assertTrue(res.success, f"{name} failed to find path")
            self.assertGreater(len(res.state_path), 0)
            # Verify no bad states visited
            for bad_id in problem.bad_states:
                self.assertNotIn(bad_id, res.state_path, f"{name} visited bad state {bad_id}")
            results[name] = res

        # LPA*, D* Lite, and A* should find optimal path of the exact same cost
        self.assertAlmostEqual(results["LPA*"].total_cost, results["A*"].total_cost, places=4)
        self.assertAlmostEqual(results["D* Lite"].total_cost, results["A*"].total_cost, places=4)

    def test_high_dimensional_cartesian_space(self):
        """Test planning in R^16, R^64, R^128 semantic embeddings."""
        for dim in [16, 64, 128]:
            problem = self._create_grid_problem(rows=4, cols=4, bad_coords=[(1, 2)], dim=dim)
            planner = LPAStarPlanner()
            res = planner.plan(problem)
            self.assertTrue(res.success, f"Failed in R^{dim}")
            self.assertNotIn(problem.bad_states[0], res.state_path)

    def test_dynamic_bad_state_addition_and_removal(self):
        """Test dynamically adding bad states and removing them with LPA*."""
        problem = self._create_grid_problem(rows=6, cols=6, bad_coords=[])
        planner = LPAStarPlanner()
        res1 = planner.plan(problem)
        self.assertTrue(res1.success)

        # Introduce obstacle blocking the middle
        mid_id = res1.state_path[len(res1.state_path) // 2]
        planner.update_bad_states(added=[mid_id])
        res2 = planner.replan()
        self.assertTrue(res2.success)
        self.assertNotIn(mid_id, res2.state_path)

        # Remove the obstacle
        planner.update_bad_states(removed=[mid_id])
        res3 = planner.replan()
        self.assertTrue(res3.success)
        self.assertAlmostEqual(res3.total_cost, res1.total_cost, places=4)


if __name__ == "__main__":
    unittest.main()
