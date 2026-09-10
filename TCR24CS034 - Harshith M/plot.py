import matplotlib.pyplot as plt
from data import State

def plot_graph():
    # Simple setup of our nodes
    states = [
        State(0, [0.0, 0.0]), State(1, [1.0, 0.0]), State(2, [2.0, 0.0]),
        State(3, [3.0, 0.0]), State(4, [1.0, 1.0]), State(5, [1.0, -2.0]), 
        State(6, [2.0, -2.0]), State(7, [2.0, 0.5])
    ]
    
    bad_states = [4, 7]
    
    plt.figure(figsize=(8, 6))
    
    # Plot states
    for s in states:
        x, y = s.em[0], s.em[1]
        if s.i in bad_states:
            plt.scatter(x, y, color='red', s=200, label='Bad State' if s.i==4 else "")
            plt.text(x, y+0.2, f"Bad (X)", color='red', ha='center')
        elif s.i == 0:
            plt.scatter(x, y, color='green', s=200, label='Start (S)')
            plt.text(x, y+0.2, f"Start (S)", color='green', ha='center')
        elif s.i == 3:
            plt.scatter(x, y, color='blue', s=200, label='Goal (G)')
            plt.text(x, y+0.2, f"Goal (G)", color='blue', ha='center')
        else:
            plt.scatter(x, y, color='gray', s=150)
            plt.text(x, y+0.2, f"Node {s.i}", ha='center')
            
    # Example paths based on Test Case 2
    # S(0) -> C(5) -> D(6) -> G(3)
    path_x = [0.0, 1.0, 2.0, 3.0]
    path_y = [0.0, -2.0, -2.0, 0.0]
    
    plt.plot(path_x, path_y, 'g--', linewidth=2, label="Safe Path")
    
    # Dangerous path
    # S(0) -> A(1) -> X(4) -> G(3)
    dang_x = [0.0, 1.0, 1.0, 3.0]
    dang_y = [0.0, 0.0, 1.0, 0.0]
    plt.plot(dang_x, dang_y, 'r:', linewidth=2, label="Dangerous Path")

    plt.grid(True)
    plt.legend()
    plt.title("Cartesian State Space Plot")
    plt.xlabel("X Coordinate")
    plt.ylabel("Y Coordinate")
    plt.show()

if __name__ == "__main__":
    plot_graph()
