import matplotlib.pyplot as plt
import matplotlib.patches as patches

fig, ax = plt.subplots(figsize=(13, 7.5), dpi=300)
fig.patch.set_facecolor('#ffffff')
ax.set_facecolor('#ffffff')
ax.set_xlim(0, 13)
ax.set_ylim(0, 7.5)
ax.axis('off')

# Title
ax.text(6.5, 7.1, 'Safe Semantic Planner — System Architecture & Data Flow',
        ha='center', va='center', fontsize=14, fontweight='bold', color='#111111')

def draw_box(ax, xy, w, h, title, items, header_bg='#212529', body_bg='#f8f9fa', ec='#111111'):
    x, y = xy
    # Body
    body = patches.Rectangle((x, y), w, h, facecolor=body_bg, edgecolor=ec, linewidth=1.5, zorder=2)
    ax.add_patch(body)
    # Header
    hh = 0.55
    header = patches.Rectangle((x, y + h - hh), w, hh, facecolor=header_bg, edgecolor=ec, linewidth=1.5, zorder=3)
    ax.add_patch(header)
    ax.text(x + w/2, y + h - hh/2, title, ha='center', va='center', fontsize=9.5, fontweight='bold', color='#ffffff', zorder=4)
    # Items
    for i, item in enumerate(items):
        ax.text(x + 0.2, y + h - hh - 0.35 - i*0.36, item, fontsize=8.5, color='#212529', zorder=4, va='center')

# Box 1: Inputs
draw_box(ax, (0.5, 2.2), 3.2, 4.2, '1. Problem Definition', [
    '• State Space S in R^d',
    '• State Embeddings x_i',
    '• Directed Transitions T',
    '• Forbidden States B',
    '• Initial State s_I',
    '• Goal State s_G',
    '• Weights: alpha, beta, gamma, delta'
], header_bg='#343a40', body_bg='#f8f9fa')

# Box 2: Precomputation & Safety
draw_box(ax, (4.3, 3.8), 4.2, 2.6, '2. Safety & Graph Processing', [
    '• Euclidean Distance: ||s_i - s_j||_2',
    '• Safety Map: D_min(s) = min ||s - b||',
    '• Forward & Reverse Adjacency Maps',
    '• Strict Infeasibility Pruning (t.to in B)'
], header_bg='#495057', body_bg='#f8f9fa')

# Box 3: Objective Cost
draw_box(ax, (4.3, 0.8), 4.2, 2.6, '3. Effective Cost Formulation', [
    'c_eff(t) = beta*cost + gamma*(1/D_min)',
    '          + delta*(1 - reliability)',
    '• Proximity Penalty (1/D_min)',
    '• Unreliability Penalty (1 - R)',
    '• Admissible Heuristic: beta * dist'
], header_bg='#495057', body_bg='#f8f9fa')

# Box 4: D* Lite Core Engine
draw_box(ax, (9.1, 2.0), 3.4, 4.4, '4. D* Lite Replanning Engine', [
    '• Backward Search (Goal -> Start)',
    '• G-Values & RHS One-Step Lookahead',
    '• Priority Queue (KeyPair k1, k2)',
    '• UpdateVertex & Inconsistency Queue',
    '• ComputeShortestPath()',
    '• Incremental Graph Repair on:',
    '   - Edge failure / addition',
    '   - Goal updates / Hazard shifts'
], header_bg='#212529', body_bg='#f8f9fa')

# Connectors / Arrows
def draw_arrow(ax, p1, p2, text=''):
    ax.annotate('', xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle='->', color='#111111', lw=1.8, mutation_scale=14), zorder=5)
    if text:
        mid = ((p1[0]+p2[0])/2, (p1[1]+p2[1])/2 + 0.15)
        ax.text(mid[0], mid[1], text, fontsize=7.5, fontweight='bold', ha='center', va='bottom', color='#495057')

draw_arrow(ax, (3.7, 5.1), (4.3, 5.1))
draw_arrow(ax, (3.7, 2.8), (4.3, 2.1))
draw_arrow(ax, (8.5, 5.1), (9.1, 4.5))
draw_arrow(ax, (8.5, 2.1), (9.1, 3.5))

# Output Box
out_patch = patches.FancyBboxPatch((4.3, 6.55), 4.2, 0.45, boxstyle="round,pad=0.08", facecolor='#e9ecef', edgecolor='#212529', lw=1.2)
ax.add_patch(out_patch)
ax.text(6.4, 6.78, 'Outputs: Optimal State/Transition Path | Traversal Cost | Safety Score | Metrics',
        ha='center', va='center', fontsize=8, fontweight='bold', color='#111111')

plt.tight_layout()
plt.savefig('docs/images/system_architecture.png', dpi=300, bbox_inches='tight')
plt.close()
print("Generated docs/images/system_architecture.png")
