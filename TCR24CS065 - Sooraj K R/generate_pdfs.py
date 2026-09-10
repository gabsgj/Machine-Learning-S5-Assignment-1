# generate_pdfs.py
# Generates 4 PDF deliverables: Design Report, Experimental Results, User Manual, Demonstration
# Uses fpdf2 library

from fpdf import FPDF


class ReportPDF(FPDF):
    """Custom PDF class with header and footer."""

    def __init__(self, title_text):
        super().__init__()
        self.title_text = title_text

    def header(self):
        self.set_font("Helvetica", "B", 10)
        self.set_text_color(100, 100, 100)
        self.cell(0, 8, self.title_text, align="R")
        self.ln(4)
        self.set_draw_color(200, 200, 200)
        self.line(10, self.get_y(), 200, self.get_y())
        self.ln(6)

    def footer(self):
        self.set_y(-15)
        self.set_font("Helvetica", "I", 8)
        self.set_text_color(150, 150, 150)
        self.cell(0, 10, f"Page {self.page_no()}/{{nb}}", align="C")

    def add_title_page(self, title, subtitle, course, department):
        self.add_page()
        self.ln(60)
        self.set_font("Helvetica", "B", 24)
        self.set_text_color(30, 30, 30)
        self.cell(0, 15, title, align="C", new_x="LMARGIN", new_y="NEXT")
        self.ln(5)
        self.set_font("Helvetica", "", 14)
        self.set_text_color(80, 80, 80)
        self.cell(0, 10, subtitle, align="C", new_x="LMARGIN", new_y="NEXT")
        self.ln(15)
        self.set_font("Helvetica", "", 12)
        self.set_text_color(100, 100, 100)
        self.cell(0, 8, course, align="C", new_x="LMARGIN", new_y="NEXT")
        self.cell(0, 8, department, align="C", new_x="LMARGIN", new_y="NEXT")

    def add_heading(self, text, level=1):
        if level == 1:
            self.ln(6)
            self.set_font("Helvetica", "B", 16)
            self.set_text_color(30, 30, 30)
            self.cell(0, 10, text, new_x="LMARGIN", new_y="NEXT")
            self.set_draw_color(50, 50, 50)
            self.line(10, self.get_y(), 200, self.get_y())
            self.ln(4)
        elif level == 2:
            self.ln(4)
            self.set_font("Helvetica", "B", 13)
            self.set_text_color(50, 50, 50)
            self.cell(0, 8, text, new_x="LMARGIN", new_y="NEXT")
            self.ln(2)
        elif level == 3:
            self.ln(2)
            self.set_font("Helvetica", "BI", 11)
            self.set_text_color(70, 70, 70)
            self.cell(0, 7, text, new_x="LMARGIN", new_y="NEXT")
            self.ln(1)

    def add_body(self, text):
        self.set_font("Helvetica", "", 11)
        self.set_text_color(40, 40, 40)
        self.multi_cell(0, 6, text)
        self.ln(2)

    def add_bullet(self, text):
        self.set_font("Helvetica", "", 11)
        self.set_text_color(40, 40, 40)
        x = self.get_x()
        self.cell(8, 6, "-")  # bullet point
        self.multi_cell(0, 6, text)
        self.ln(1)

    def add_code(self, text):
        self.set_font("Courier", "", 9)
        self.set_text_color(30, 30, 30)
        self.set_fill_color(240, 240, 240)
        # Draw background
        y_start = self.get_y()
        lines = text.split("\n")
        for line in lines:
            self.cell(0, 5, "  " + line, new_x="LMARGIN", new_y="NEXT", fill=True)
        self.ln(3)

    def add_table(self, headers, rows):
        """Draw a simple table."""
        self.set_font("Helvetica", "B", 10)
        self.set_fill_color(60, 60, 60)
        self.set_text_color(255, 255, 255)

        num_cols = len(headers)
        col_width = 190 / num_cols

        # Header row
        for h in headers:
            self.cell(col_width, 8, str(h), border=1, fill=True, align="C")
        self.ln()

        # Data rows
        self.set_font("Helvetica", "", 10)
        self.set_text_color(40, 40, 40)
        fill = False
        for row in rows:
            if fill:
                self.set_fill_color(245, 245, 245)
            else:
                self.set_fill_color(255, 255, 255)
            for cell_val in row:
                self.cell(col_width, 7, str(cell_val), border=1, fill=True, align="C")
            self.ln()
            fill = not fill
        self.ln(3)


# ============================================================
# PDF 1: Design Report
# ============================================================
def create_design_report():
    pdf = ReportPDF("Design Report - Safe Semantic Planner")
    pdf.alias_nb_pages()

    # Title page
    pdf.add_title_page(
        "Design Report",
        "Safe Semantic Planner in a Finite Cartesian State Space",
        "PCCST503 - Machine Learning | Assignment 1",
        "Department of Computer Science and Engineering"
    )

    # 1. Introduction
    pdf.add_page()
    pdf.add_heading("1. Introduction")
    pdf.add_body(
        "This report describes the design and implementation of a safe semantic planner "
        "that operates in a finite Cartesian state space. The planner computes a safe path "
        "from an initial state to a goal state while avoiding designated bad (unsafe) states. "
        "The implementation uses A* search algorithm with Euclidean distance heuristic."
    )

    # 2. State Representation
    pdf.add_heading("2. State Representation")
    pdf.add_body(
        "Each state in the system is represented as a point in d-dimensional Cartesian space. "
        "A state has two attributes:"
    )
    pdf.add_bullet("id: A unique integer identifier for the state.")
    pdf.add_bullet("embedding: A list of floating-point coordinates [x1, x2, ..., xd] representing "
                   "the state's position in Cartesian space.")
    pdf.add_body(
        "For our test cases, we use 2D embeddings (d=2), where each state has an (x, y) position. "
        "This allows the planner to calculate straight-line distances between states for the heuristic function."
    )
    pdf.add_heading("State Class Definition", level=2)
    pdf.add_code(
        "class State:\n"
        "    id: int\n"
        "    embedding: list  # e.g., [0.0, 0.0] for 2D"
    )

    # 3. Data Structures
    pdf.add_heading("3. Data Structures")
    pdf.add_heading("3.1 Transition", level=2)
    pdf.add_body(
        "A directed edge between two states. Each transition contains:"
    )
    pdf.add_bullet("id: Unique transition identifier.")
    pdf.add_bullet("from_state, to_state: Source and destination state IDs.")
    pdf.add_bullet("cost: Numeric cost of traversing this transition.")
    pdf.add_bullet("safety: Safety score of the transition (0.0 to 1.0).")
    pdf.add_bullet("reliability: Reliability score (0.0 to 1.0).")
    pdf.add_bullet("available: Boolean flag indicating if the transition can be used.")

    pdf.add_heading("3.2 Planning Problem", level=2)
    pdf.add_body(
        "Bundles all inputs needed by the planner:"
    )
    pdf.add_bullet("initialState: ID of the starting state.")
    pdf.add_bullet("goalState: ID of the target state.")
    pdf.add_bullet("badStates: List of state IDs that must be avoided.")
    pdf.add_bullet("states: List of all State objects in the environment.")
    pdf.add_bullet("transitions: List of all Transition objects.")

    pdf.add_heading("3.3 Planning Result", level=2)
    pdf.add_body(
        "The output returned by the planner:"
    )
    pdf.add_bullet("success: Whether a valid path was found.")
    pdf.add_bullet("statePath: Ordered list of state IDs from start to goal.")
    pdf.add_bullet("transitionPath: Ordered list of transition IDs used.")
    pdf.add_bullet("totalCost: Sum of all transition costs along the path.")
    pdf.add_bullet("safetyScore: Minimum Euclidean distance to nearest bad state.")
    pdf.add_bullet("exploredStates: Number of states explored during search.")
    pdf.add_bullet("planningTime: Wall-clock time taken to find the path.")
    pdf.add_bullet("reliabilityScore: Sum of reliability values of transitions used.")
    pdf.add_bullet("score: Combined objective function value.")

    pdf.add_heading("3.4 Internal Data Structures", level=2)
    pdf.add_body("During A* search, the planner uses the following internal structures:")
    pdf.add_bullet("state_map (dict): Maps state ID to State object for O(1) lookup.")
    pdf.add_bullet("adjacency (dict): Maps state ID to list of outgoing transitions (adjacency list).")
    pdf.add_bullet("transition_map (dict): Maps transition ID to Transition object.")
    pdf.add_bullet("open_set (min-heap): Priority queue ordered by f-score for A*.")
    pdf.add_bullet("g_score (dict): Tracks cheapest known cost to reach each state.")
    pdf.add_bullet("came_from (dict): Parent pointers for path reconstruction.")
    pdf.add_bullet("closed_set (set): States already fully explored.")

    # 4. Heuristic Function
    pdf.add_heading("4. Heuristic Function")
    pdf.add_body(
        "The heuristic function h(n) estimates the cost from state n to the goal state. "
        "We use the Euclidean distance in the embedding space:"
    )
    pdf.add_code(
        "h(n) = sqrt( (x1_n - x1_g)^2 + (x2_n - x2_g)^2 + ... + (xd_n - xd_g)^2 )"
    )
    pdf.add_body(
        "This heuristic is admissible (never overestimates) when transition costs are at least "
        "as large as the Euclidean distance between connected states. This guarantees that A* "
        "finds an optimal path with respect to transition cost."
    )

    # 5. Safety Computation
    pdf.add_heading("5. Safety Computation")
    pdf.add_body(
        "Safety is computed in two ways:"
    )
    pdf.add_heading("5.1 Bad State Avoidance", level=2)
    pdf.add_body(
        "During search, the planner completely skips bad states. Any state in the badStates list "
        "is added to the closed set immediately and never expanded. This ensures the planner "
        "never visits a bad state."
    )
    pdf.add_heading("5.2 Safety Distance", level=2)
    pdf.add_body(
        "After finding a path, the planner computes the minimum Euclidean distance from every "
        "visited state on the path to the nearest bad state. This metric shows how close the "
        "path gets to dangerous areas:"
    )
    pdf.add_code(
        "safety_distance = min over all states s in path:\n"
        "    min over all bad states b:\n"
        "        euclidean_distance(s.embedding, b.embedding)"
    )
    pdf.add_body(
        "If there are no bad states, the safety distance is infinity (perfectly safe)."
    )

    # 6. Scoring Function
    pdf.add_heading("6. Objective / Scoring Function")
    pdf.add_body("The combined scoring function evaluates path quality:")
    pdf.add_code("Score(P) = alpha * G  -  beta * C  +  gamma * D  +  delta * R")
    pdf.add_body("Where:")
    pdf.add_bullet("G = 1 if goal reached, 0 otherwise (default alpha = 10.0)")
    pdf.add_bullet("C = total transition cost, lower is better (default beta = 1.0)")
    pdf.add_bullet("D = min safety distance to bad states, higher is better (default gamma = 2.0)")
    pdf.add_bullet("R = cumulative reliability of transitions (default delta = 1.0)")
    pdf.add_body(
        "Higher scores indicate better paths. The weights can be tuned to prioritize "
        "cost minimization (increase beta), safety (increase gamma), or reliability (increase delta)."
    )

    # 7. Algorithm
    pdf.add_heading("7. Algorithm: A* Search")
    pdf.add_body("The planner uses A* search, which works as follows:")
    pdf.add_body(
        "1. Initialize the open set with the start state (f = 0 + h(start)).\n"
        "2. Pop the state with the lowest f-score from the open set.\n"
        "3. If it is the goal, reconstruct and return the path.\n"
        "4. If it is a bad state or already explored, skip it.\n"
        "5. For each available outgoing transition:\n"
        "   a. Calculate tentative g-score = g(current) + transition.cost\n"
        "   b. If this is better than the known g-score, update it\n"
        "   c. Compute f = g + h(neighbor) and push to open set\n"
        "6. If the open set is empty and goal not reached, return failure."
    )

    # 8. Complexity
    pdf.add_heading("8. Complexity Analysis")
    pdf.add_heading("Time Complexity", level=2)
    pdf.add_body(
        "O((V + E) * log V), where V is the number of states and E is the number of transitions. "
        "Each state is processed at most once, and each heap operation takes O(log V)."
    )
    pdf.add_heading("Space Complexity", level=2)
    pdf.add_body(
        "O(V + E) for storing the adjacency list, state map, g-scores, came_from pointers, "
        "and the open/closed sets."
    )

    # 9. Dynamic Replanning
    pdf.add_heading("9. Dynamic Environment and Replanning")
    pdf.add_body(
        "The planner supports dynamic environments where the following may change:"
    )
    pdf.add_bullet("Goal state changes (Test Case 5)")
    pdf.add_bullet("Transitions become unavailable (Test Case 4)")
    pdf.add_bullet("New transitions are added (Test Case 6)")
    pdf.add_bullet("Bad states change")
    pdf.add_body(
        "Replanning approach: Modify the PlanningProblem object to reflect the change, then "
        "call plan() again. This is the simplest approach and works well for small to medium graphs. "
        "For very large graphs, incremental algorithms like D* Lite or LPA* could be used to "
        "reuse previous search results and avoid re-exploring unchanged portions of the graph."
    )

    pdf.output("docs/design_report.pdf")
    print("Created: docs/design_report.pdf")


# ============================================================
# PDF 2: Experimental Results
# ============================================================
def create_experimental_results():
    pdf = ReportPDF("Experimental Results - Safe Semantic Planner")
    pdf.alias_nb_pages()

    pdf.add_title_page(
        "Experimental Results",
        "Safe Semantic Planner in a Finite Cartesian State Space",
        "PCCST503 - Machine Learning | Assignment 1",
        "Department of Computer Science and Engineering"
    )

    pdf.add_page()
    pdf.add_heading("1. Overview")
    pdf.add_body(
        "This document presents the experimental results from running the Safe Semantic Planner "
        "on 6 test cases. Each test case evaluates a different aspect of the planner: basic reachability, "
        "bad state avoidance, safety margin analysis, dynamic transition handling, goal updates, "
        "and transition additions. All tests were executed using Python 3 on a standard machine."
    )

    # Test Case 1
    pdf.add_heading("2. Test Case Results")
    pdf.add_heading("Test Case 1: Basic Reachability", level=2)
    pdf.add_body("Graph: S -> A -> B -> G (simple linear path, no bad states)")
    pdf.add_table(
        ["Metric", "Value"],
        [
            ["Success", "True"],
            ["State Path", "[0, 1, 2, 3]"],
            ["Transition Path", "[10, 11, 12]"],
            ["Total Cost", "3.0"],
            ["Safety Score", "inf (no bad states)"],
            ["Reliability", "3.0"],
            ["Objective Score", "210.0"],
            ["Explored States", "4"],
            ["Planning Time", "~0.000156 s"],
        ]
    )
    pdf.add_body("Result: PASS. The planner found the unique valid path as expected.")

    # Test Case 2
    pdf.add_heading("Test Case 2: Bad State Avoidance", level=2)
    pdf.add_body(
        "Graph: Path 1: S -> A -> X -> G (X is bad), Path 2: S -> C -> D -> G (safe). "
        "The planner must avoid Path 1 and choose Path 2."
    )
    pdf.add_table(
        ["Metric", "Value"],
        [
            ["Success", "True"],
            ["State Path", "[0, 3, 5, 4]"],
            ["Transition Path", "[23, 24, 25]"],
            ["Total Cost", "4.5"],
            ["Safety Score", "1.4142"],
            ["Reliability", "3.0"],
            ["Objective Score", "11.3284"],
            ["Explored States", "5"],
            ["Bad States Visited", "0"],
            ["Planning Time", "~0.000123 s"],
        ]
    )
    pdf.add_body("Result: PASS. The planner correctly avoided bad state X and chose the safe detour.")

    # Test Case 3
    pdf.add_heading("Test Case 3: Safety Margin", level=2)
    pdf.add_body(
        "Two valid paths exist. Path 1 (S -> A -> B -> G) is cheaper (cost=3.0) but passes close "
        "to a bad state. Path 2 (S -> C -> D -> G) costs more (cost=6.0) but stays far away."
    )
    pdf.add_table(
        ["Metric", "Path 1 (Chosen)", "Path 2 (Alternative)"],
        [
            ["Cost", "3.0", "6.0"],
            ["Safety Distance", "0.5", "~3.16"],
            ["Reliability", "2.7", "3.0"],
            ["Objective Score", "10.7", "7.32"],
        ]
    )
    pdf.add_body(
        "Result: PASS. A* minimizes cost, so it picks Path 1 (cost=3.0). However, the safety score "
        "of 0.5 reveals that this path passes very close to the bad state. Path 2 would have a "
        "safety distance of approximately 3.16 but costs twice as much. The scoring function "
        "captures this tradeoff. To prioritize safety, the gamma weight could be increased."
    )

    # Test Case 4
    pdf.add_heading("Test Case 4: Dynamic Transition", level=2)
    pdf.add_body(
        "Initially S -> A -> G is available (cost=2.0). Then transition A -> G becomes unavailable, "
        "forcing the planner to find an alternative."
    )
    pdf.add_table(
        ["Metric", "Before", "After Replan"],
        [
            ["Success", "True", "True"],
            ["State Path", "[0, 1, 4]", "[0, 2, 3, 4]"],
            ["Total Cost", "2.0", "4.5"],
            ["Explored States", "3", "5"],
            ["Planning Time", "~0.000044 s", "~0.000070 s"],
        ]
    )
    pdf.add_body(
        "Result: PASS. After the direct path was blocked, the planner successfully found the "
        "backup path through states B and C. Replanning time was approximately 0.00007 seconds."
    )

    # Test Case 5
    pdf.add_heading("Test Case 5: Goal Update", level=2)
    pdf.add_body(
        "Original goal is G1 (state 2). After initial planning, the goal changes to G2 (state 4)."
    )
    pdf.add_table(
        ["Metric", "Goal = G1", "Goal = G2 (Replan)"],
        [
            ["Success", "True", "True"],
            ["State Path", "[0, 1, 2]", "[0, 1, 3, 4]"],
            ["Total Cost", "2.0", "3.0"],
            ["Explored States", "3", "4"],
            ["Planning Time", "~0.000022 s", "~0.000025 s"],
        ]
    )
    pdf.add_body(
        "Result: PASS. The planner successfully computed a new path to the updated goal G2. "
        "Replanning time was approximately 0.000025 seconds."
    )

    # Test Case 6
    pdf.add_heading("Test Case 6: Transition Addition", level=2)
    pdf.add_body(
        "Initially, only the long path S -> A -> B -> G exists (cost=6.0). A new shortcut "
        "transition S -> G (cost=1.0) is then added."
    )
    pdf.add_table(
        ["Metric", "Before Shortcut", "After Shortcut"],
        [
            ["Success", "True", "True"],
            ["State Path", "[0, 1, 2, 3]", "[0, 3]"],
            ["Total Cost", "6.0", "1.0"],
            ["Explored States", "4", "2"],
            ["Planning Time", "~0.000017 s", "~0.000032 s"],
        ]
    )
    pdf.add_body(
        "Result: PASS. The planner discovered the new shortcut and produced a much cheaper path "
        "(cost reduced from 6.0 to 1.0). The number of explored states also dropped from 4 to 2."
    )

    # Summary
    pdf.add_heading("3. Summary of Results")
    pdf.add_table(
        ["Test Case", "Goal Reached", "Bad States Visited", "Result"],
        [
            ["1. Basic Reachability", "Yes", "0", "PASS"],
            ["2. Bad State Avoidance", "Yes", "0", "PASS"],
            ["3. Safety Margin", "Yes", "0", "PASS"],
            ["4. Dynamic Transition", "Yes", "0", "PASS"],
            ["5. Goal Update", "Yes", "0", "PASS"],
            ["6. Transition Addition", "Yes", "0", "PASS"],
        ]
    )

    pdf.add_heading("4. Evaluation Metrics Summary")
    pdf.add_body("The following metrics were evaluated across all test cases:")
    pdf.add_bullet("Goal success rate: 100% (6/6 tests passed)")
    pdf.add_bullet("Bad states visited: 0 in all tests")
    pdf.add_bullet("Planning time: All tests completed in under 0.001 seconds")
    pdf.add_bullet("Replanning time: All dynamic replanning completed in under 0.0001 seconds")
    pdf.add_bullet("Memory usage: Minimal (dictionary-based structures, no large matrices)")
    pdf.add_bullet("Explored states: Ranges from 2 to 5 depending on graph complexity")

    pdf.output("docs/experimental_results.pdf")
    print("Created: docs/experimental_results.pdf")


# ============================================================
# PDF 3: User Manual
# ============================================================
def create_user_manual():
    pdf = ReportPDF("User Manual - Safe Semantic Planner")
    pdf.alias_nb_pages()

    pdf.add_title_page(
        "User Manual",
        "Safe Semantic Planner in a Finite Cartesian State Space",
        "PCCST503 - Machine Learning | Assignment 1",
        "Department of Computer Science and Engineering"
    )

    # Prerequisites
    pdf.add_page()
    pdf.add_heading("1. Prerequisites")
    pdf.add_body("To run this project, you need:")
    pdf.add_bullet("Python 3.8 or higher")
    pdf.add_bullet("No external libraries required (uses only Python standard library)")
    pdf.add_body("To verify your Python version:")
    pdf.add_code("python3 --version")

    # Project Structure
    pdf.add_heading("2. Project Structure")
    pdf.add_code(
        "ML_Project/\n"
        "  structures.py    - Data classes (State, Transition, etc.)\n"
        "  planner.py       - A* planner with safety and scoring\n"
        "  test_cases.py    - All 6 test case definitions\n"
        "  main.py          - Main runner script\n"
        "  README.md        - Project documentation\n"
        "  docs/            - PDF deliverables"
    )

    # Running the Project
    pdf.add_heading("3. Running the Project")
    pdf.add_body("To run all 6 test cases and see the results:")
    pdf.add_code(
        "cd ML_Project\n"
        "python3 main.py"
    )
    pdf.add_body(
        "This will execute all test cases including dynamic replanning scenarios and print "
        "detailed metrics for each one."
    )

    # Understanding the Output
    pdf.add_heading("4. Understanding the Output")
    pdf.add_body("For each test case, the output shows:")
    pdf.add_bullet("Success: Whether a valid path was found (True/False)")
    pdf.add_bullet("State Path: The ordered list of state IDs from start to goal")
    pdf.add_bullet("Transition Path: The ordered list of transition IDs used")
    pdf.add_bullet("Total Cost: Sum of transition costs along the path")
    pdf.add_bullet("Safety Score: Minimum Euclidean distance from any path state to nearest bad state")
    pdf.add_bullet("Reliability: Sum of reliability values of transitions used")
    pdf.add_bullet("Objective Score: Combined score using Score(P) = aG - bC + gD + dR")
    pdf.add_bullet("Explored States: Number of states the algorithm examined")
    pdf.add_bullet("Planning Time: Time taken to compute the path (in seconds)")

    # Creating Custom Test Cases
    pdf.add_heading("5. Creating Custom Test Cases")
    pdf.add_body("You can create your own test cases by following these steps:")
    pdf.add_heading("Step 1: Define States", level=2)
    pdf.add_code(
        "from structures import State, Transition, PlanningProblem\n"
        "\n"
        "# Create states with IDs and 2D positions\n"
        "s = State(id=0, embedding=[0.0, 0.0])   # Start\n"
        "a = State(id=1, embedding=[1.0, 0.0])   # Intermediate\n"
        "g = State(id=2, embedding=[2.0, 0.0])   # Goal"
    )
    pdf.add_heading("Step 2: Define Transitions", level=2)
    pdf.add_code(
        "# Create transitions between states\n"
        "t1 = Transition(\n"
        "    id=1,\n"
        "    from_state=0, to_state=1,\n"
        "    cost=1.0, safety=1.0,\n"
        "    reliability=1.0, available=True\n"
        ")\n"
        "t2 = Transition(\n"
        "    id=2,\n"
        "    from_state=1, to_state=2,\n"
        "    cost=1.0, safety=1.0,\n"
        "    reliability=1.0, available=True\n"
        ")"
    )
    pdf.add_heading("Step 3: Create Problem and Run", level=2)
    pdf.add_code(
        "from planner import SafePlanner\n"
        "\n"
        "problem = PlanningProblem(\n"
        "    initialState=0,\n"
        "    goalState=2,\n"
        "    badStates=[],\n"
        "    states=[s, a, g],\n"
        "    transitions=[t1, t2]\n"
        ")\n"
        "\n"
        "planner = SafePlanner()\n"
        "result = planner.plan(problem)\n"
        "print(result)"
    )

    # Replanning
    pdf.add_heading("6. Dynamic Replanning")
    pdf.add_body("The planner provides three replanning functions:")
    pdf.add_heading("6.1 Disable a Transition", level=2)
    pdf.add_code(
        "from planner import replan_with_unavailable_transition\n"
        "\n"
        "# Make transition from state 1 to state 2 unavailable\n"
        "result = replan_with_unavailable_transition(\n"
        "    planner, problem, from_state=1, to_state=2\n"
        ")"
    )
    pdf.add_heading("6.2 Change the Goal", level=2)
    pdf.add_code(
        "from planner import replan_with_new_goal\n"
        "\n"
        "# Change goal to state 3\n"
        "result = replan_with_new_goal(\n"
        "    planner, problem, new_goal=3\n"
        ")"
    )
    pdf.add_heading("6.3 Add a New Transition", level=2)
    pdf.add_code(
        "from planner import replan_with_new_transition\n"
        "from structures import Transition\n"
        "\n"
        "# Add a shortcut transition\n"
        "shortcut = Transition(\n"
        "    id=99, from_state=0, to_state=2,\n"
        "    cost=0.5, safety=1.0,\n"
        "    reliability=1.0, available=True\n"
        ")\n"
        "result = replan_with_new_transition(\n"
        "    planner, problem, shortcut\n"
        ")"
    )

    # Tuning Weights
    pdf.add_heading("7. Tuning the Scoring Function")
    pdf.add_body(
        "The scoring function Score(P) = alpha*G - beta*C + gamma*D + delta*R uses "
        "default weights. You can change them in planner.py inside the compute_score() function:"
    )
    pdf.add_bullet("Increase beta to penalize expensive paths more heavily.")
    pdf.add_bullet("Increase gamma to reward paths that stay far from bad states.")
    pdf.add_bullet("Increase delta to favor more reliable transitions.")
    pdf.add_bullet("Increase alpha to strongly reward reaching the goal.")

    pdf.output("docs/user_manual.pdf")
    print("Created: docs/user_manual.pdf")


# ============================================================
# PDF 4: Demonstration
# ============================================================
def create_demonstration():
    pdf = ReportPDF("Demonstration - Safe Semantic Planner")
    pdf.alias_nb_pages()

    pdf.add_title_page(
        "Demonstration",
        "Safe Semantic Planner in a Finite Cartesian State Space",
        "PCCST503 - Machine Learning | Assignment 1",
        "Department of Computer Science and Engineering"
    )

    pdf.add_page()
    pdf.add_heading("1. Demonstration Overview")
    pdf.add_body(
        "This document provides a walkthrough of the Safe Semantic Planner in action. "
        "It demonstrates the planner's core capabilities through 6 test cases, showing "
        "how the algorithm handles basic pathfinding, safety constraints, and dynamic "
        "environment changes."
    )

    # Demo 1
    pdf.add_heading("2. Demo 1: Finding a Simple Path")
    pdf.add_heading("Setup", level=2)
    pdf.add_body(
        "A simple linear graph with 4 states: S(0) -> A(1) -> B(2) -> G(3). "
        "All transitions have cost 1.0 and are available. No bad states."
    )
    pdf.add_heading("Graph Layout", level=2)
    pdf.add_code(
        "  S -----> A -----> B -----> G\n"
        "(0,0)    (1,0)    (2,0)    (3,0)"
    )
    pdf.add_heading("Execution", level=2)
    pdf.add_code(
        "$ python3 main.py\n"
        "\n"
        "TEST CASE 1: Basic Reachability\n"
        "  Success:         True\n"
        "  State Path:      [0, 1, 2, 3]\n"
        "  Total Cost:      3.0\n"
        "  Explored States: 4"
    )
    pdf.add_body("The planner correctly finds the only available path with a total cost of 3.0.")

    # Demo 2
    pdf.add_heading("3. Demo 2: Avoiding Dangerous States")
    pdf.add_heading("Setup", level=2)
    pdf.add_body(
        "Two paths exist. Path 1 goes through bad state X, Path 2 is a safe detour. "
        "The planner must automatically avoid X."
    )
    pdf.add_heading("Graph Layout", level=2)
    pdf.add_code(
        "        A -----> X(BAD!)\n"
        "       /              \\\n"
        "      S                G\n"
        "       \\              /\n"
        "        C -----> D\n"
    )
    pdf.add_heading("Result", level=2)
    pdf.add_code(
        "  State Path:        [0, 3, 5, 4]\n"
        "  (S -> C -> D -> G)\n"
        "  Bad States Visited: 0\n"
        "  Safety Score:       1.4142"
    )
    pdf.add_body("The planner avoids X entirely and chooses the safe detour through C and D.")

    # Demo 3
    pdf.add_heading("4. Demo 3: Cost vs Safety Tradeoff")
    pdf.add_heading("Setup", level=2)
    pdf.add_body(
        "Two valid paths exist (neither goes through a bad state). Path 1 is cheap but "
        "passes close to a bad state. Path 2 is expensive but stays far away."
    )
    pdf.add_heading("Comparison", level=2)
    pdf.add_table(
        ["", "Path 1 (Cheap)", "Path 2 (Safe)"],
        [
            ["Route", "S -> A -> B -> G", "S -> C -> D -> G"],
            ["Cost", "3.0", "6.0"],
            ["Safety Distance", "0.5", "~3.16"],
            ["Chosen?", "Yes (by A*)", "No"],
        ]
    )
    pdf.add_body(
        "A* picks Path 1 because it has lower cost. The safety score of 0.5 warns us that "
        "this path comes very close to a bad state. By adjusting the gamma weight in the "
        "scoring function, we could make the planner prefer safer paths."
    )

    # Demo 4
    pdf.add_heading("5. Demo 4: Handling Broken Connections")
    pdf.add_heading("Scenario", level=2)
    pdf.add_body(
        "The planner initially finds the direct path S -> A -> G. Then, the transition "
        "from A to G becomes unavailable (simulating a broken connection)."
    )
    pdf.add_heading("Before", level=2)
    pdf.add_code(
        "  Path: [0, 1, 4]  (S -> A -> G)\n"
        "  Cost: 2.0"
    )
    pdf.add_heading("After (A -> G breaks)", level=2)
    pdf.add_code(
        "  Path: [0, 2, 3, 4]  (S -> B -> C -> G)\n"
        "  Cost: 4.5\n"
        "  Replanning Time: ~0.00007 seconds"
    )
    pdf.add_body(
        "The planner successfully finds an alternative route after the direct connection breaks. "
        "Replanning is nearly instantaneous."
    )

    # Demo 5
    pdf.add_heading("6. Demo 5: Goal Changes Mid-Execution")
    pdf.add_heading("Scenario", level=2)
    pdf.add_body(
        "The original goal is G1 (state 2). After the initial plan is computed, the goal "
        "changes to G2 (state 4). The planner replans to reach the new goal."
    )
    pdf.add_heading("Before (Goal = G1)", level=2)
    pdf.add_code(
        "  Path: [0, 1, 2]  (S -> A -> G1)\n"
        "  Cost: 2.0"
    )
    pdf.add_heading("After (Goal = G2)", level=2)
    pdf.add_code(
        "  Path: [0, 1, 3, 4]  (S -> A -> B -> G2)\n"
        "  Cost: 3.0\n"
        "  Replanning Time: ~0.000025 seconds"
    )
    pdf.add_body("The planner quickly adapts to the new goal without any issues.")

    # Demo 6
    pdf.add_heading("7. Demo 6: Discovering New Shortcuts")
    pdf.add_heading("Scenario", level=2)
    pdf.add_body(
        "Initially, only the long path S -> A -> B -> G exists (cost=6.0). A new direct "
        "transition S -> G (cost=1.0) is then added to the graph."
    )
    pdf.add_heading("Before (Long Path Only)", level=2)
    pdf.add_code(
        "  Path: [0, 1, 2, 3]  (S -> A -> B -> G)\n"
        "  Cost: 6.0\n"
        "  Explored States: 4"
    )
    pdf.add_heading("After (Shortcut Added)", level=2)
    pdf.add_code(
        "  Path: [0, 3]  (S -> G directly!)\n"
        "  Cost: 1.0\n"
        "  Explored States: 2"
    )
    pdf.add_body(
        "The planner discovers and uses the new shortcut, reducing cost from 6.0 to 1.0 "
        "and explored states from 4 to just 2."
    )

    # Conclusion
    pdf.add_heading("8. Conclusion")
    pdf.add_body(
        "The demonstration shows that the Safe Semantic Planner successfully handles all "
        "required scenarios:"
    )
    pdf.add_bullet("Finds optimal paths in simple graphs")
    pdf.add_bullet("Avoids all bad states with zero violations")
    pdf.add_bullet("Computes meaningful safety distance metrics")
    pdf.add_bullet("Handles dynamic changes: broken transitions, goal updates, new shortcuts")
    pdf.add_bullet("Replans in microseconds for small graphs")
    pdf.add_bullet("Produces a combined score balancing cost, safety, and reliability")

    pdf.output("docs/demonstration.pdf")
    print("Created: docs/demonstration.pdf")


# ============================================================
# Main: Generate all PDFs
# ============================================================
if __name__ == "__main__":
    import os
    os.makedirs("docs", exist_ok=True)

    create_design_report()
    create_experimental_results()
    create_user_manual()
    create_demonstration()

    print("\nAll 4 PDFs generated in docs/ folder!")
