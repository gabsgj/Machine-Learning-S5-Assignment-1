import math
import matplotlib.pyplot as plt
import networkx as nx
import os



def euclidean_distance(state1, state2):
    return math.sqrt(
        (state1.x - state2.x) ** 2 +
        (state1.y - state2.y) ** 2
    )


def calculate_edge_score(transition):
    """
    Lower score = better transition
    """

    SAFETY_WEIGHT = 2.0
    RELIABILITY_WEIGHT = 1.0

    return (
            transition.cost
            - SAFETY_WEIGHT * transition.safety
            - RELIABILITY_WEIGHT * transition.reliability
    )


def draw_graph(planner, path=None, filename="graph.png"):
    """
    Draws the graph and highlights the selected path.
    """

    G = nx.DiGraph()

    # Add nodes
    for state_id, state in planner.states.items():
        G.add_node(state_id, pos=(state.x, state.y))

        # Add edges
    for from_state in planner.graph:
        for edge in planner.graph[from_state]:

            if edge.available:
                G.add_edge(
                    edge.from_state,
                    edge.to_state,
                    weight=edge.cost
                )

    pos = nx.get_node_attributes(G, "pos")

    plt.figure(figsize=(8, 6))

    # Draw nodes
    nx.draw_networkx_nodes(
        G,
        pos,
        node_color="skyblue",
        node_size=900
    )

    # Draw labels
    nx.draw_networkx_labels(
        G,
        pos,
        font_weight="bold"
    )

    # Draw normal edges
    nx.draw_networkx_edges(
        G,
        pos,
        edge_color="gray",
        arrows=True
    )

    # Edge labels
    labels = nx.get_edge_attributes(G, "weight")

    nx.draw_networkx_edge_labels(
        G,
        pos,
        edge_labels=labels
    )

    # Highlight chosen path
    if path and len(path) > 1:

        edges = []

        for i in range(len(path) - 1):
            edges.append((path[i], path[i + 1]))

        nx.draw_networkx_edges(
            G,
            pos,
            edgelist=edges,
            edge_color="green",
            width=4,
            arrows=True
        )

    os.makedirs("output", exist_ok=True)

    plt.title("Safe Semantic Planner")

    plt.savefig(f"output/{filename}")

    plt.close()