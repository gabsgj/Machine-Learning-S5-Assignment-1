from planner import State, Transition, PlanningProblem, SafeSemanticPlanner

def print_result(case_num: int, title: str, res):
    print(f"\n--- Test Case {case_num}: {title} ---")
    print(f"Success: {res.success}")
    print(f"State Path: {res.state_path}")
    print(f"Transition Path: {res.transition_path}")
    print(f"Total Cost: {res.total_cost:.4f}")
    print(f"Min Safety Distance: {res.safety_score}")
    print(f"Explored Nodes: {res.explored_nodes_count}")
    print(f"Time Taken: {res.execution_time_ms:.4f} ms")

def run_all_tests():
    planner = SafeSemanticPlanner()

    # --- Test Case 1: Basic Reachability ---
    states_tc1 = [
        State(0, [0.0, 0.0]), State(1, [1.0, 0.0]), 
        State(2, [2.0, 0.0]), State(3, [3.0, 0.0])
    ]
    transitions_tc1 = [
        Transition(101, 0, 1, 1.0, 1.0, 1.0, True),
        Transition(102, 1, 2, 1.0, 1.0, 1.0, True),
        Transition(103, 2, 3, 1.0, 1.0, 1.0, True)
    ]
    prob1 = PlanningProblem(0, 3, [], states_tc1, transitions_tc1)
    print_result(1, "Basic Reachability", planner.plan(prob1))

    # --- Test Case 2: Bad State Avoidance ---
    states_tc2 = [
        State(0, [0.0, 0.0]), State(1, [1.0, 1.0]), State(2, [2.0, 1.0]), # State 2 is Bad
        State(3, [1.0, -1.0]), State(4, [2.0, -1.0]), State(5, [3.0, 0.0])
    ]
    transitions_tc2 = [
        Transition(201, 0, 1, 1.0, 1.0, 1.0, True), Transition(202, 1, 2, 1.0, 1.0, 1.0, True),
        Transition(203, 2, 5, 1.0, 1.0, 1.0, True), Transition(204, 0, 3, 1.0, 1.0, 1.0, True),
        Transition(205, 3, 4, 1.0, 1.0, 1.0, True), Transition(206, 4, 5, 1.0, 1.0, 1.0, True)
    ]
    prob2 = PlanningProblem(0, 5, [2], states_tc2, transitions_tc2)
    print_result(2, "Bad State Avoidance", planner.plan(prob2))

    # --- Test Case 3: Safety Margin ---
    states_tc3 = [
        State(0, [0.0, 0.0]), State(1, [2.0, 0.1]), State(2, [4.0, 0.0]),
        State(3, [2.0, 3.0]), State(99, [2.0, 0.0]) # Bad state at 99
    ]
    transitions_tc3 = [
        Transition(301, 0, 1, 1.0, 1.0, 1.0, True), Transition(302, 1, 2, 1.0, 1.0, 1.0, True),
        Transition(303, 0, 3, 2.5, 1.0, 1.0, True), Transition(304, 3, 2, 2.5, 1.0, 1.0, True)
    ]
    prob3 = PlanningProblem(0, 2, [99], states_tc3, transitions_tc3)
    print_result(3, "Safety Margin Preference", planner.plan(prob3))

    # --- Test Case 4: Dynamic Transition ---
    states_tc4 = [State(0, [0.0, 0.0]), State(1, [1.0, 0.0]), State(2, [2.0, 0.0]), State(3, [1.0, 1.0])]
    t_ag = Transition(402, 1, 2, 1.0, 1.0, 1.0, True)
    transitions_tc4 = [
        Transition(401, 0, 1, 1.0, 1.0, 1.0, True), t_ag,
        Transition(403, 1, 3, 1.0, 1.0, 1.0, True), Transition(404, 3, 2, 1.0, 1.0, 1.0, True)
    ]
    prob4 = PlanningProblem(0, 2, [], states_tc4, transitions_tc4)
    print_result(4, "Dynamic Transition (Before failure)", planner.plan(prob4))
    
    # Disable (1 -> 2), forces detour via State 3: (0 -> 1 -> 3 -> 2)
    t_ag.available = False  
    print_result(4, "Dynamic Transition (After failure)", planner.plan(prob4))

    # --- Test Case 5: Goal Update ---
    prob5 = PlanningProblem(0, 3, [], states_tc4, transitions_tc4)
    print_result(5, "Goal Update (Goal changed to State 3)", planner.plan(prob5))

    # --- Test Case 6: Transition Addition ---
    # Add direct shortcut (0 -> 2)
    transitions_tc4.append(Transition(601, 0, 2, 0.5, 1.0, 1.0, True))
    prob6 = PlanningProblem(0, 2, [], states_tc4, transitions_tc4)
    print_result(6, "Transition Addition (Shortcut 0 -> 2 added)", planner.plan(prob6))

if __name__ == "__main__":
    run_all_tests()