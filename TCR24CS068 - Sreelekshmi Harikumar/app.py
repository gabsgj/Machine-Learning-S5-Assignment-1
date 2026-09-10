import time

import streamlit as st
import plotly.graph_objects as go
import pandas as pd

from experiments.benchmark import run_benchmark

from planner.dstar_lite import DStarLite
from planner.models import Transition

from experiments.test_cases import (
    test_case_1,
    test_case_2,
    test_case_3,
    test_case_4,
    test_case_5,
    test_case_6,
)


# ============================================================
# PAGE CONFIGURATION
# ============================================================

st.set_page_config(
    page_title="Safe Semantic Planner",
    page_icon="",
    layout="wide",
)


# ============================================================
# TITLE
# ============================================================

st.title("Safe Semantic Planner")

st.markdown(
    """
    ### D* Lite based planning in a finite Cartesian state space

    The planner finds safe paths while considering:

    - Transition cost
    - Safety
    - Reliability
    - Bad states
    - Dynamic environment changes
    """
)


# ============================================================
# TEST CASES
# ============================================================

TEST_CASES = {
    "Test 1 - Basic Reachability": test_case_1,
    "Test 2 - Bad State Avoidance": test_case_2,
    "Test 3 - Safety Margin": test_case_3,
    "Test 4 - Dynamic Transition": test_case_4,
    "Test 5 - Goal Update": test_case_5,
    "Test 6 - Transition Addition": test_case_6,
}


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def state_lookup(problem):
    return {
        state.id: state
        for state in problem.states
    }


def transition_lookup(problem):
    return {
        transition.id: transition
        for transition in problem.transitions
    }


def path_to_names(problem, state_path):
    states = state_lookup(problem)

    return [
        states[state_id].name
        for state_id in state_path
    ]


def path_string(problem, state_path):

    if not state_path:
        return "No path"

    return " → ".join(
        path_to_names(problem, state_path)
    )


def get_transition_name(problem, transition):

    states = state_lookup(problem)

    source = states[transition.from_state]
    target = states[transition.to_state]

    return (
        f"{source.name} → {target.name}"
        f"  (cost={transition.cost})"
    )


# ============================================================
# SIDEBAR
# ============================================================

st.sidebar.header("Planner Settings")


selected_test = st.sidebar.selectbox(
    "Select Test Case",
    list(TEST_CASES.keys()),
)


st.sidebar.subheader("Optimization Weights")


cost_weight = st.sidebar.slider(
    "Cost Weight",
    0.0,
    5.0,
    1.0,
    0.1,
)


safety_weight = st.sidebar.slider(
    "Safety Weight",
    0.0,
    5.0,
    0.0,
    0.1,
)


reliability_weight = st.sidebar.slider(
    "Reliability Weight",
    0.0,
    5.0,
    0.0,
    0.1,
)


# ============================================================
# LOAD PROBLEM
# ============================================================

problem = TEST_CASES[selected_test]()

states = state_lookup(problem)


# ============================================================
# RESET PLANNER WHEN TEST CASE CHANGES
# ============================================================

if (
    st.session_state.get("planner_problem_id")
    != selected_test
):

    st.session_state["planner"] = None
    st.session_state["result"] = None
    st.session_state["planner_problem_id"] = (
        selected_test
    )

    st.session_state["last_problem_signature"] = None


# ============================================================
# DYNAMIC ENVIRONMENT
# ============================================================

st.sidebar.divider()

st.sidebar.subheader("Dynamic Environment")

st.sidebar.caption(
    "Modify the environment and use Replan "
    "to update the existing D* Lite search."
)


# ============================================================
# DISABLE TRANSITIONS
# ============================================================

transition_options = {
    get_transition_name(problem, transition):
    transition.id
    for transition in problem.transitions
}


disabled_transitions = st.sidebar.multiselect(
    "Disable transitions",
    options=list(transition_options.keys()),
)


disabled_ids = {
    transition_options[name]
    for name in disabled_transitions
}


# ============================================================
# CHANGE GOAL
# ============================================================

goal_options = {
    state.name: state.id
    for state in problem.states
    if state.id not in problem.bad_states
}


goal_names = list(goal_options.keys())

current_goal_index = 0

if problem.goal_state in goal_options.values():

    current_goal_index = list(
        goal_options.values()
    ).index(
        problem.goal_state
    )


selected_goal_name = st.sidebar.selectbox(
    "Goal State",
    options=goal_names,
    index=current_goal_index,
)


selected_goal_id = goal_options[
    selected_goal_name
]


# ============================================================
# CHANGE BAD STATES
# ============================================================

bad_state_options = {
    state.name: state.id
    for state in problem.states
    if state.id != problem.initial_state
    and state.id != selected_goal_id
}


current_bad_names = [
    state.name
    for state in problem.states
    if state.id in problem.bad_states
    and state.id != selected_goal_id
]


selected_bad_names = st.sidebar.multiselect(
    "Bad States",
    options=list(bad_state_options.keys()),
    default=current_bad_names,
)


selected_bad_ids = {
    bad_state_options[name]
    for name in selected_bad_names
}


# ============================================================
# ADD NEW TRANSITION
# ============================================================

st.sidebar.markdown("**Add New Transition**")


state_names = {
    state.name: state.id
    for state in problem.states
}


add_from = st.sidebar.selectbox(
    "From",
    list(state_names.keys()),
    key="add_from",
)


add_to = st.sidebar.selectbox(
    "To",
    list(state_names.keys()),
    key="add_to",
)


add_cost = st.sidebar.number_input(
    "Cost",
    min_value=0.1,
    value=1.0,
    step=0.5,
    key="add_cost",
)


add_safety = st.sidebar.slider(
    "Safety",
    min_value=0.0,
    max_value=1.0,
    value=1.0,
    step=0.05,
    key="add_safety",
)


add_reliability = st.sidebar.slider(
    "Reliability",
    min_value=0.0,
    max_value=1.0,
    value=1.0,
    step=0.05,
    key="add_reliability",
)


add_transition_button = st.sidebar.button(
    "➕Add Transition",
    use_container_width=True,
)


# ============================================================
# RUN / REPLAN BUTTONS
# ============================================================

st.sidebar.divider()


run_button = st.sidebar.button(
    "Run Planner",
    use_container_width=True,
)


replan_button = st.sidebar.button(
    "🔄 Incremental Replan",
    use_container_width=True,
)


# ============================================================
# APPLY ADDITIONAL TRANSITION
# ============================================================

if add_transition_button:

    new_id = (
        max(
            [
                transition.id
                for transition in problem.transitions
            ],
            default=-1,
        )
        + 1
    )

    new_transition = Transition(
        id=new_id,
        from_state=state_names[add_from],
        to_state=state_names[add_to],
        cost=float(add_cost),
        safety=float(add_safety),
        reliability=float(add_reliability),
        available=True,
    )

    problem.transitions.append(
        new_transition
    )

    st.sidebar.success(
        f"Added transition: "
        f"{add_from} → {add_to}"
    )


# ============================================================
# INITIAL PLANNING
# ============================================================

if run_button:

    start_time = time.perf_counter()

    # Apply current UI settings to problem
    problem.goal_state = selected_goal_id

    problem.bad_states = set(
        selected_bad_ids
    )

    for transition in problem.transitions:

        transition.available = (
            transition.id not in disabled_ids
        )

    planner = DStarLite(
        problem=problem,
        cost_weight=cost_weight,
        safety_weight=safety_weight,
        reliability_weight=reliability_weight,
    )

    result = planner.plan()

    elapsed_ms = (
        time.perf_counter()
        - start_time
    ) * 1000.0

    st.session_state["planner"] = planner

    st.session_state["planner_problem_id"] = (
        selected_test
    )

    st.session_state["result"] = result

    st.session_state["replanning_time"] = (
        elapsed_ms
    )

    st.session_state["last_problem_signature"] = (
        None
    )


# ============================================================
# INCREMENTAL REPLANNING
# ============================================================

if replan_button:

    planner = st.session_state.get(
        "planner"
    )

    if planner is None:

        st.warning(
            "Please click 'Run Planner' first."
        )

    else:

        start_time = time.perf_counter()

        # ----------------------------------------------------
        # GOAL UPDATE
        # ----------------------------------------------------

        if (
            planner.problem.goal_state
            != selected_goal_id
        ):

            planner.update_goal(
                selected_goal_id
            )

        # ----------------------------------------------------
        # BAD STATE UPDATE
        # ----------------------------------------------------

        new_bad_states = set(
            selected_bad_ids
        )

        old_bad_states = set(
            planner.problem.bad_states
        )

        if (
            new_bad_states
            != old_bad_states
        ):

            planner.update_bad_states(
                new_bad_states
            )

        # ----------------------------------------------------
        # TRANSITION UPDATES
        # ----------------------------------------------------

        planner_transition_ids = set(
            planner.transitions.keys()
        )

        for transition in problem.transitions:

            # Existing transition
            if transition.id in planner_transition_ids:

                current_available = (
                    planner.transitions[
                        transition.id
                    ].available
                )

                desired_available = (
                    transition.id
                    not in disabled_ids
                )

                if (
                    current_available
                    != desired_available
                ):

                    planner.update_transition(
                        transition.id,
                        desired_available,
                    )

            # New transition
            else:

                transition.available = (
                    transition.id
                    not in disabled_ids
                )

                planner.add_transition(
                    transition
                )

        # ----------------------------------------------------
        # INCREMENTAL REPLAN
        # ----------------------------------------------------

        result = planner.replan()

        elapsed_ms = (
            time.perf_counter()
            - start_time
        ) * 1000.0

        st.session_state["result"] = result

        st.session_state[
            "replanning_time"
        ] = elapsed_ms


# ============================================================
# GET RESULT
# ============================================================

result = st.session_state.get(
    "result",
    None,
)


# ============================================================
# GRAPH DRAWING
# ============================================================

def draw_graph(problem, result):

    fig = go.Figure()

    states = state_lookup(problem)

    # --------------------------------------------------------
    # EDGES
    # --------------------------------------------------------

    for transition in problem.transitions:

        if not transition.available:
            continue

        source = states[
            transition.from_state
        ]

        target = states[
            transition.to_state
        ]

        is_path_edge = False

        if result and result.success:

            path = result.state_path

            for i in range(
                len(path) - 1
            ):

                if (
                    path[i]
                    == transition.from_state
                    and
                    path[i + 1]
                    == transition.to_state
                ):

                    is_path_edge = True
                    break

        if is_path_edge:

            color = "#00CC66"
            width = 6

        else:

            color = "#888888"
            width = 2

        fig.add_trace(
            go.Scatter(
                x=[
                    source.embedding[0],
                    target.embedding[0],
                ],
                y=[
                    source.embedding[1],
                    target.embedding[1],
                ],
                mode="lines+markers",
                line=dict(
                    color=color,
                    width=width,
                ),
                marker=dict(
                    size=7,
                ),
                hoverinfo="text",
                text=(
                    f"{source.name} → {target.name}<br>"
                    f"Cost: {transition.cost}<br>"
                    f"Safety: {transition.safety}<br>"
                    f"Reliability: "
                    f"{transition.reliability}"
                ),
                showlegend=False,
            )
        )

    # --------------------------------------------------------
    # STATES
    # --------------------------------------------------------

    for state in problem.states:

        if state.id in problem.bad_states:

            color = "#FF3333"

        elif state.id == problem.initial_state:

            color = "#3399FF"

        elif state.id == problem.goal_state:

            color = "#FFD700"

        elif (
            result
            and result.success
            and state.id in result.state_path
        ):

            color = "#00CC66"

        else:

            color = "#DDDDDD"

        fig.add_trace(
            go.Scatter(
                x=[state.embedding[0]],
                y=[state.embedding[1]],
                mode="markers+text",
                marker=dict(
                    size=30,
                    color=color,
                    line=dict(
                        color="black",
                        width=2,
                    ),
                ),
                text=[state.name],
                textposition="top center",
                hoverinfo="text",
                hovertext=(
                    f"State: {state.name}<br>"
                    f"ID: {state.id}<br>"
                    f"Coordinates: "
                    f"{state.embedding}"
                ),
                showlegend=False,
            )
        )

    # --------------------------------------------------------
    # LAYOUT
    # --------------------------------------------------------

    fig.update_layout(
        title="Cartesian State Space",
        xaxis_title="X",
        yaxis_title="Y",
        height=600,
        template="plotly_white",
        showlegend=False,
    )

    return fig


# ============================================================
# MAIN LAYOUT
# ============================================================

left, right = st.columns(
    [2, 1]
)


# ============================================================
# GRAPH
# ============================================================

with left:

    st.subheader("State Space")

    st.plotly_chart(
        draw_graph(
            problem,
            result,
        ),
        use_container_width=True,
    )


# ============================================================
# RESULTS
# ============================================================

with right:

    st.subheader("Planning Results")

    if result is None:

        st.info(
            "Choose a test case and click "
            "'Run Planner'."
        )

    elif result.success:

        st.success(
            "✓ Safe path found"
        )

        st.metric(
            "Total Cost",
            f"{result.total_cost:.2f}",
        )

        if result.safety_score == float("inf"):

            safety_text = "∞"

        else:

            safety_text = (
                f"{result.safety_score:.3f}"
            )

        st.metric(
            "Minimum Safety Distance",
            safety_text,
        )

        st.metric(
            "Reliability",
            f"{result.reliability_score:.3f}",
        )

        st.metric(
            "Explored States",
            result.explored_states,
        )

        st.metric(
            "Planning Time",
            f"{result.planning_time:.3f} ms",
        )

        replanning_time = (
            st.session_state.get(
                "replanning_time"
            )
        )

        if replanning_time is not None:

            st.metric(
                "Replanning Time",
                f"{replanning_time:.3f} ms",
            )

        st.markdown(
            "### Selected Path"
        )

        st.code(
            path_string(
                problem,
                result.state_path,
            )
        )

    else:

        st.error(
            "✗ No safe path found"
        )

        st.write(
            result.message
        )


# ============================================================
# PROBLEM INFORMATION
# ============================================================

st.divider()

st.subheader(
    "Current Planning Problem"
)


col1, col2, col3 = st.columns(3)


with col1:

    initial_state = states[
        problem.initial_state
    ]

    st.write(
        f"**Initial:** "
        f"{initial_state.name}"
    )


with col2:

    current_goal = states[
        problem.goal_state
    ]

    st.write(
        f"**Goal:** "
        f"{current_goal.name}"
    )


with col3:

    bad_names = [
        states[state_id].name
        for state_id in problem.bad_states
        if state_id in states
    ]

    st.write(
        "**Bad States:** "
        + (
            ", ".join(bad_names)
            if bad_names
            else "None"
        )
    )


# ============================================================
# TRANSITION INFORMATION
# ============================================================

st.subheader(
    "🔗 Current Transitions"
)


transition_data = []


for transition in problem.transitions:

    source = states[
        transition.from_state
    ]

    target = states[
        transition.to_state
    ]

    transition_data.append(
        {
            "ID": transition.id,
            "From": source.name,
            "To": target.name,
            "Cost": transition.cost,
            "Safety": transition.safety,
            "Reliability": transition.reliability,
            "Available": transition.available,
        }
    )


st.dataframe(
    transition_data,
    use_container_width=True,
)

# ============================================================
# EXPERIMENTAL EVALUATION
# ============================================================

st.divider()

st.header("Experimental Evaluation")

st.markdown(
    """
    The planner is evaluated using the six assignment test cases.
    
    The following metrics are measured:
    
    - Goal success
    - Bad states visited
    - Total path cost
    - Minimum safety distance
    - Reliability
    - Number of explored states
    - Planning time
    """
)


# ------------------------------------------------------------
# Run benchmark
# ------------------------------------------------------------

if st.button(
    "Run All Experiments",
    use_container_width=True,
):

    with st.spinner(
        "Running experimental evaluation..."
    ):

        benchmark_results = run_benchmark()

    st.session_state[
        "benchmark_results"
    ] = benchmark_results


# ------------------------------------------------------------
# Display benchmark results
# ------------------------------------------------------------

benchmark_results = st.session_state.get(
    "benchmark_results",
    None,
)


if benchmark_results:

    df = pd.DataFrame(
        benchmark_results
    )

    # --------------------------------------------------------
    # Summary metrics
    # --------------------------------------------------------

    total_tests = len(df)

    successful_tests = int(
        df["Success"].sum()
    )

    total_bad_states = int(
        df["Bad States"].sum()
    )

    average_time = (
        df["Planning Time (ms)"].mean()
    )

    col1, col2, col3, col4 = st.columns(4)

    with col1:

        st.metric(
            "Tests",
            total_tests,
        )

    with col2:

        st.metric(
            "Successful",
            f"{successful_tests}/{total_tests}",
        )

    with col3:

        st.metric(
            "Bad States Visited",
            total_bad_states,
        )

    with col4:

        st.metric(
            "Average Planning Time",
            f"{average_time:.3f} ms",
        )


    # --------------------------------------------------------
    # Results table
    # --------------------------------------------------------

    st.subheader(
        "Benchmark Results"
    )

    display_df = df.copy()

    display_df["Success"] = (
        display_df["Success"]
        .map(
            {
                True: "✓",
                False: "✗",
            }
        )
    )

    display_df["Safety"] = (
        display_df["Safety"]
        .replace(
            float("inf"),
            "∞",
        )
    )

    st.dataframe(
        display_df,
        use_container_width=True,
        hide_index=True,
    )


    # --------------------------------------------------------
    # Cost chart
    # --------------------------------------------------------

    st.subheader(
        "Path Cost"
    )

    cost_fig = go.Figure()

    cost_fig.add_trace(
        go.Bar(
            x=df["Test Case"],
            y=df["Cost"],
            marker_color="#3399FF",
            text=df["Cost"].round(2),
            textposition="auto",
        )
    )

    cost_fig.update_layout(
        xaxis_title="Test Case",
        yaxis_title="Total Cost",
        template="plotly_white",
        height=400,
    )

    st.plotly_chart(
        cost_fig,
        use_container_width=True,
    )


    # --------------------------------------------------------
    # Planning time chart
    # --------------------------------------------------------

    st.subheader(
        "Planning Time"
    )

    time_fig = go.Figure()

    time_fig.add_trace(
        go.Bar(
            x=df["Test Case"],
            y=df["Planning Time (ms)"],
            marker_color="#8E44AD",
            text=df[
                "Planning Time (ms)"
            ].round(3),
            textposition="auto",
        )
    )

    time_fig.update_layout(
        xaxis_title="Test Case",
        yaxis_title="Planning Time (ms)",
        template="plotly_white",
        height=400,
    )

    st.plotly_chart(
        time_fig,
        use_container_width=True,
    )


    # --------------------------------------------------------
    # Explored states chart
    # --------------------------------------------------------

    st.subheader(
        "Search Effort"
    )

    explored_fig = go.Figure()

    explored_fig.add_trace(
        go.Bar(
            x=df["Test Case"],
            y=df["Explored States"],
            marker_color="#E67E22",
            text=df["Explored States"],
            textposition="auto",
        )
    )

    explored_fig.update_layout(
        xaxis_title="Test Case",
        yaxis_title="Explored States",
        template="plotly_white",
        height=400,
    )

    st.plotly_chart(
        explored_fig,
        use_container_width=True,
    )


    # --------------------------------------------------------
    # Safety chart
    # --------------------------------------------------------

    st.subheader(
        "Minimum Safety Distance"
    )

    safety_values = []

    for value in df["Safety"]:

        if value == float("inf"):

            safety_values.append(0)

        else:

            safety_values.append(value)


    safety_fig = go.Figure()

    safety_fig.add_trace(
        go.Bar(
            x=df["Test Case"],
            y=safety_values,
            marker_color="#00CC66",
        )
    )

    safety_fig.update_layout(
        xaxis_title="Test Case",
        yaxis_title=(
            "Minimum Distance "
            "(∞ shown as 0 for visualization)"
        ),
        template="plotly_white",
        height=400,
    )

    st.plotly_chart(
        safety_fig,
        use_container_width=True,
    )


    # --------------------------------------------------------
    # Reliability chart
    # --------------------------------------------------------

    st.subheader(
        "Reliability"
    )

    reliability_fig = go.Figure()

    reliability_fig.add_trace(
        go.Bar(
            x=df["Test Case"],
            y=df["Reliability"],
            marker_color="#F1C40F",
            text=df["Reliability"].round(3),
            textposition="auto",
        )
    )

    reliability_fig.update_layout(
        xaxis_title="Test Case",
        yaxis_title="Cumulative Reliability",
        yaxis=dict(
            range=[0, 1.1]
        ),
        template="plotly_white",
        height=400,
    )

    st.plotly_chart(
        reliability_fig,
        use_container_width=True,
    )


    # --------------------------------------------------------
    # Evaluation conclusion
    # --------------------------------------------------------

    st.subheader(
        "Evaluation Summary"
    )

    if (
        successful_tests == total_tests
        and total_bad_states == 0
    ):

        st.success(
            "All benchmark tests successfully "
            "reached their goals without visiting "
            "any bad states."
        )

    else:

        st.warning(
            "Some benchmark requirements were "
            "not completely satisfied."
        )

else:

    st.info(
        "Click 'Run All Experiments' to generate "
        "the experimental evaluation."
    )
    
# ============================================================
# DYNAMIC PLANNER STATUS
# ============================================================

st.divider()

st.subheader(
    "Incremental Planner Status"
)


planner = st.session_state.get(
    "planner"
)


if planner is None:

    st.info(
        "Planner not initialized. "
        "Click 'Run Planner'."
    )

else:

    status_col1, status_col2, status_col3 = (
        st.columns(3)
    )

    with status_col1:

        st.metric(
            "Planner",
            "Active",
        )

    with status_col2:

        st.metric(
            "D* Lite States",
            len(planner.states),
        )

    with status_col3:

        st.metric(
            "Search Queue",
            len(planner.open_list),
        )
