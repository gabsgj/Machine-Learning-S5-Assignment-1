import matplotlib.pyplot as plt
import matplotlib.patches as patches
import numpy as np
import os

os.makedirs("docs/images", exist_ok=True)

# Clean, professional styling
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['DejaVu Sans', 'Arial', 'Segoe UI', 'Helvetica']
plt.rcParams['axes.edgecolor'] = '#333333'
plt.rcParams['axes.linewidth'] = 1.0

# ==============================================================================
# 1. FIGURE: EXPERIMENTAL BENCHMARK RESULTS (benchmark_metrics.png)
# ==============================================================================
fig = plt.figure(figsize=(13, 8.5), dpi=300)
fig.patch.set_facecolor('#ffffff')

gs = fig.add_gridspec(2, 2, hspace=0.32, wspace=0.25, left=0.08, right=0.95, top=0.91, bottom=0.08)

# Panel A: Traversal Cost vs Expanded States (TC1-TC6)
ax1 = fig.add_subplot(gs[0, 0])
cases = ['TC1\nLinear', 'TC2\nAvoid Bad', 'TC3 (a)\nCost Focus', 'TC3 (b)\nSafe Focus', 'TC4 (a)\nPre-Fail', 'TC4 (b)\nReplan', 'TC5\nGoal Shift', 'TC6\nShortcut']
costs = [3.0, 3.0, 2.0, 6.0, 2.0, 4.5, 2.5, 2.0]
expanded = [3, 3, 3, 4, 3, 4, 3, 7]

x = np.arange(len(cases))
w = 0.35

rects1 = ax1.bar(x - w/2, costs, w, label='Traversal Cost ($C$)', color='#111111', edgecolor='#111111', zorder=3)
rects2 = ax1.bar(x + w/2, expanded, w, label='States Explored ($|V_{exp}|$) ', color='#888888', edgecolor='#333333', zorder=3)

ax1.set_ylabel('Value', fontsize=10, fontweight='bold', color='#111111')
ax1.set_title('(a) Path Cost vs. State Expansions Across Core Tests', fontsize=11, fontweight='bold', pad=10, loc='left')
ax1.set_xticks(x)
ax1.set_xticklabels(cases, fontsize=8)
ax1.set_ylim(0, 8.5)
ax1.legend(frameon=True, facecolor='#ffffff', edgecolor='#cccccc', fontsize=8.5, loc='upper left')
ax1.grid(axis='y', linestyle=':', alpha=0.6, zorder=0)

for r in rects1:
    h = r.get_height()
    ax1.text(r.get_x() + r.get_width()/2, h + 0.15, f'{h:.1f}', ha='center', va='bottom', fontsize=7.5, fontweight='bold')
for r in rects2:
    h = r.get_height()
    ax1.text(r.get_x() + r.get_width()/2, h + 0.15, f'{int(h)}', ha='center', va='bottom', fontsize=7.5, color='#444444')

# Panel B: Safety Distance (D_min) Sensitivity under Gamma Tuning
ax2 = fig.add_subplot(gs[0, 1])
scenarios_safety = [
    'TC3: Cost Bias\n($\\beta=1.0, \\gamma=0.5$)',
    'TC3: Safety Bias\n($\\beta=0.5, \\gamma=3.0$)',
    'Hospital: Fastest\n($\\beta=2.0, \\gamma=0.0$)',
    'Hospital: Safest\n($\\beta=0.5, \\gamma=3.0$)',
    'Hospital: Reliable\n($\\beta=0.5, \\delta=3.0$)'
]
d_min_values = [0.707, 3.536, 0.200, 2.010, 2.010]
bar_colors = ['#555555', '#111111', '#777777', '#111111', '#111111']

bars2 = ax2.bar(range(len(scenarios_safety)), d_min_values, width=0.45, color=bar_colors, edgecolor='#111111', zorder=3)
ax2.set_ylabel('Min Distance to Hazard ($D_{min}$)', fontsize=10, fontweight='bold', color='#111111')
ax2.set_title('(b) Hazard Clearance ($D_{min}$) under Safety Weight Scaling', fontsize=11, fontweight='bold', pad=10, loc='left')
ax2.set_xticks(range(len(scenarios_safety)))
ax2.set_xticklabels(scenarios_safety, fontsize=8)
ax2.set_ylim(0, 4.2)
ax2.grid(axis='y', linestyle=':', alpha=0.6, zorder=0)

for b in bars2:
    h = b.get_height()
    ax2.text(b.get_x() + b.get_width()/2, h + 0.1, f'{h:.3f}', ha='center', va='bottom', fontsize=8, fontweight='bold')

# Panel C: Replanning Node Expansion Efficiency (D* Lite Incremental vs Full Scratch)
ax3 = fig.add_subplot(gs[1, 0])
dyn_events = [
    'TC4: Edge Closed\n(A$\\rightarrow$G broken)',
    'TC5: Goal Shift\n(G1 $\\rightarrow$ G2)',
    'TC6: Shortcut Added\n(A$\\rightarrow$G added)',
    'Delivery: Replan 1\n(Highway closed)',
    'Delivery: Replan 2\n(Park closed)'
]
scratch_evals = [8, 9, 10, 11, 14]
dstar_evals = [4, 3, 5, 5, 6]

xs = np.arange(len(dyn_events))
ax3.bar(xs - w/2, scratch_evals, w, label='Full Scratch Search (Nodes)', color='#cccccc', edgecolor='#333333', zorder=3)
ax3.bar(xs + w/2, dstar_evals, w, label='D* Lite Incremental (Nodes)', color='#111111', edgecolor='#111111', zorder=3)

ax3.set_ylabel('Vertex Expansions / Processed', fontsize=10, fontweight='bold', color='#111111')
ax3.set_title('(c) Dynamic Replanning Efficiency (Incremental vs Scratch)', fontsize=11, fontweight='bold', pad=10, loc='left')
ax3.set_xticks(xs)
ax3.set_xticklabels(dyn_events, fontsize=8)
ax3.set_ylim(0, 16)
ax3.legend(frameon=True, facecolor='#ffffff', edgecolor='#cccccc', fontsize=8.5, loc='upper left')
ax3.grid(axis='y', linestyle=':', alpha=0.6, zorder=0)

for i, (sc, ds) in enumerate(zip(scratch_evals, dstar_evals)):
    reduction = int(round((1 - ds/sc) * 100))
    ax3.text(xs[i] + w/2, ds + 0.35, f'-{reduction}%', ha='center', va='bottom', fontsize=7.5, fontweight='bold', color='#111111')

# Panel D: Verification Matrix & Scorecard
ax4 = fig.add_subplot(gs[1, 1])
ax4.axis('off')

card_content = (
    "SYSTEM VERIFICATION & BENCHMARK SUMMARY\n"
    "----------------------------------------------------\n"
    "• Unit & Edge Test Suite:       14 / 14 PASSED (100%)\n"
    "• Assignment Test Cases:        TC1 - TC6 ALL VERIFIED\n"
    "• Real-World Scenarios:         4 / 4 SCENARIOS SOLVED\n"
    "• Forbidden State Incursions:   0 BAD STATES VISITED\n"
    "• Dynamic Replanning Latency:   < 10 microseconds (O(k log n))\n"
    "• Heuristic Guarantees:         Admissible & Consistent in R^d\n"
    "• Memory Overhead:              O(|V|*d + |E|) Compact Layout\n"
    "----------------------------------------------------\n"
    "Core Multi-Objective Formulation:\n"
    "  c_eff(t) = beta * cost + gamma*(1 / D_min) + delta*(1 - R)\n"
    "  h(a, b)  = beta * ||a - b||_2  (provably lower-bounding)"
)

ax4.text(0.02, 0.98, card_content, transform=ax4.transAxes, fontsize=8.8,
         verticalalignment='top', fontfamily='monospace', linespacing=1.4,
         bbox=dict(boxstyle='square,pad=0.8', facecolor='#fafafa', edgecolor='#333333', linewidth=1.2))

fig.suptitle('Safe Semantic Planner — Quantitative Experimental Evaluation', fontsize=13, fontweight='bold', y=0.97)
plt.savefig('docs/images/benchmark_metrics.png', dpi=300, facecolor='#ffffff')
print("Saved: docs/images/benchmark_metrics.png")

# ==============================================================================
# 2. FIGURE: SCENARIO TRAJECTORY PLOTS (planner_scenarios.png)
# ==============================================================================
fig, axs = plt.subplots(2, 2, figsize=(15, 11), dpi=300)
fig.patch.set_facecolor('#ffffff')
plt.subplots_adjust(hspace=0.38, wspace=0.22, left=0.06, right=0.96, top=0.92, bottom=0.05)

def render_scenario(ax, title, subtitle, states, edges, bad_nodes, path_nodes, start_node, goal_node, xlim, ylim, show_all_edge_costs=True):
    ax.set_title(title, fontsize=11, fontweight='bold', pad=16, loc='left', color='#111111')
    ax.text(0.0, 1.02, subtitle, transform=ax.transAxes, fontsize=8.5, color='#555555', va='bottom', ha='left')
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_aspect('equal')
    ax.axis('off')
    
    path_edges = set((path_nodes[i], path_nodes[i+1]) for i in range(len(path_nodes)-1))
    
    # 1. Draw Edges
    for u, v, cost_val, is_avail in edges:
        p1 = np.array(states[u], dtype=float)
        p2 = np.array(states[v], dtype=float)
        v_diff = p2 - p1
        dist = np.linalg.norm(v_diff)
        if dist < 1e-6: continue
        u_dir = v_diff / dist
        r = 0.36
        p_start = p1 + u_dir * r
        p_end = p2 - u_dir * r
        
        is_in_path = (u, v) in path_edges
        
        if not is_avail:
            ax.annotate('', xy=p_end, xytext=p_start,
                        arrowprops=dict(arrowstyle='->', color='#888888', lw=1.5, linestyle='--', mutation_scale=12))
            mid = (p1 + p2) / 2 + np.array([-u_dir[1], u_dir[0]]) * 0.22
            ax.text(mid[0], mid[1], 'X (Blocked)', color='#333333', fontsize=7.5, fontweight='bold', ha='center', va='center',
                    bbox=dict(boxstyle='square,pad=0.2', facecolor='#ffffff', edgecolor='#aaaaaa', lw=0.8, alpha=0.9))
        elif is_in_path:
            ax.annotate('', xy=p_end, xytext=p_start,
                        arrowprops=dict(arrowstyle='->', color='#111111', lw=2.8, mutation_scale=16))
            if show_all_edge_costs:
                mid = (p1 + p2) / 2 + np.array([-u_dir[1], u_dir[0]]) * 0.20
                ax.text(mid[0], mid[1], f'c={cost_val}', color='#111111', fontsize=7.5, fontweight='bold', ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.15', facecolor='#ffffff', edgecolor='none', alpha=0.85))
        else:
            ax.annotate('', xy=p_end, xytext=p_start,
                        arrowprops=dict(arrowstyle='->', color='#cccccc', lw=1.2, mutation_scale=10))
            if show_all_edge_costs:
                mid = (p1 + p2) / 2 + np.array([-u_dir[1], u_dir[0]]) * 0.18
                ax.text(mid[0], mid[1], f'c={cost_val}', color='#888888', fontsize=7, ha='center', va='center',
                        bbox=dict(boxstyle='round,pad=0.15', facecolor='#ffffff', edgecolor='none', alpha=0.85))

    # 2. Draw Nodes
    for sid, pos in states.items():
        pos = np.array(pos, dtype=float)
        is_bad = sid in bad_nodes
        is_start = (sid == start_node)
        is_goal = (sid == goal_node)
        is_path = sid in path_nodes
        
        if is_bad:
            zone = patches.Circle(pos, 0.72, facecolor='#f5f5f5', edgecolor='#666666', linestyle=':', linewidth=1.2, zorder=3)
            ax.add_patch(zone)
            node = patches.Circle(pos, 0.32, facecolor='#e0e0e0', edgecolor='#222222', linewidth=1.5, zorder=4)
            ax.add_patch(node)
            ax.text(pos[0], pos[1], f'{sid}\n[BAD]', ha='center', va='center', fontsize=7.5, fontweight='bold', color='#111111', zorder=5)
        elif is_start:
            node = patches.Circle(pos, 0.32, facecolor='#111111', edgecolor='#111111', linewidth=1.5, zorder=4)
            ax.add_patch(node)
            ax.text(pos[0], pos[1], f'S({sid})', ha='center', va='center', fontsize=8, fontweight='bold', color='#ffffff', zorder=5)
        elif is_goal:
            node_outer = patches.Circle(pos, 0.34, facecolor='#ffffff', edgecolor='#111111', linewidth=2.0, zorder=4)
            node_inner = patches.Circle(pos, 0.23, facecolor='#ffffff', edgecolor='#111111', linewidth=1.2, zorder=5)
            ax.add_patch(node_outer)
            ax.add_patch(node_inner)
            ax.text(pos[0], pos[1], f'G({sid})', ha='center', va='center', fontsize=8, fontweight='bold', color='#111111', zorder=6)
        else:
            fc = '#ffffff' if not is_path else '#f0f0f0'
            ec = '#111111' if is_path else '#999999'
            lw = 2.0 if is_path else 1.2
            node = patches.Circle(pos, 0.30, facecolor=fc, edgecolor=ec, linewidth=lw, zorder=4)
            ax.add_patch(node)
            ax.text(pos[0], pos[1], f'{sid}', ha='center', va='center', fontsize=8.5, fontweight='bold', color='#111111', zorder=5)

# Panel 1: TC2 Bad State Avoidance (Widely Spaced)
st_tc2 = {0: (0.0, 0.0), 1: (2.0, 1.4), 2: (4.0, 1.4), 3: (2.0, -1.4), 5: (4.0, -1.4), 4: (6.0, 0.0)}
ed_tc2 = [(0,1,1.0,True), (1,2,1.0,True), (2,4,1.0,True), (0,3,1.0,True), (3,5,1.0,True), (5,4,1.0,True)]
render_scenario(axs[0, 0], "Scenario 1 — Strict Hazard Avoidance (TC2)",
                "Safe trajectory: S(0) -> 3 -> 5 -> G(4)  |  Total Cost = 3.0, Avoids Bad State 2",
                st_tc2, ed_tc2, [2], [0, 3, 5, 4], 0, 4, (-0.8, 6.8), (-2.2, 2.2))

# Panel 2: TC4 Dynamic Edge Replanning (Widely Spaced)
st_tc4 = {0: (0.0, 0.0), 1: (2.4, 1.4), 2: (5.4, 0.0), 3: (2.0, -1.4), 4: (3.8, -1.4)}
ed_tc4 = [(0,1,1.0,True), (1,2,1.0,False), (0,3,1.5,True), (3,4,1.5,True), (4,2,1.5,True)]
render_scenario(axs[0, 1], "Scenario 2 — Dynamic Edge Failure & Detour (TC4)",
                "Transition 1->2 blocked at runtime  |  Replanned Route: S(0) -> 3 -> 4 -> G(2) (Cost=4.5)",
                st_tc4, ed_tc4, [], [0, 3, 4, 2], 0, 2, (-0.8, 6.2), (-2.2, 2.2))

# Panel 3: Hospital Weights Tradeoff (Demo D)
st_hosp = {0: (0.0, 0.0), 1: (2.4, 0.7), 4: (2.4, 2.2), 3: (5.4, 0.0), 2: (1.8, -1.6), 5: (3.8, -1.6)}
ed_hosp = [(0,1,1.0,True), (1,3,1.0,True), (0,2,2.0,True), (2,5,1.0,True), (5,3,1.0,True)]
render_scenario(axs[1, 0], "Scenario 3 — Multi-Objective Weight Tradeoff (Hospital Demo)",
                "High safety weight (gamma=3.0) rejects close path 0->1->3 for safe 0->2->5->3 (Clearance D_min=2.01)",
                st_hosp, ed_hosp, [4], [0, 2, 5, 3], 0, 3, (-0.8, 6.2), (-2.4, 3.0))

# Panel 4: Warehouse Grid Multi-Hazard Navigation (Demo B)
st_w = {
    0: (0.0, 2.6), 1: (1.5, 2.6), 2: (3.0, 2.6), 3: (4.5, 2.6), 4: (6.0, 2.6),
    5: (0.0, 1.3), 6: (1.5, 1.3), 7: (3.0, 1.3), 8: (4.5, 1.3), 9: (6.0, 1.3),
    10: (0.0, 0.0), 11: (1.5, 0.0), 12: (3.0, 0.0), 13: (4.5, 0.0), 14: (6.0, 0.0)
}
ed_w = [
    (0,1,1.0,True), (1,2,1.0,True), (2,3,1.0,True), (3,4,1.0,True),
    (0,5,1.0,True), (5,10,1.0,True), (10,11,1.0,True), (11,12,1.0,True),
    (12,13,1.0,True), (13,14,1.0,True), (14,9,1.0,True), (9,4,1.0,True),
    (5,6,1.0,True), (6,7,1.0,True), (7,8,1.0,True), (8,9,1.0,True),
    (1,6,1.0,True), (6,11,1.0,True), (3,8,1.0,True), (8,13,1.0,True)
]
p_w = [0, 5, 10, 11, 12, 13, 14, 9, 4]
render_scenario(axs[1, 1], "Scenario 4 — Multi-Hazard Grid Routing (Warehouse Demo)",
                "Uniform grid cost c=1.0  |  Safely bypasses Spills 2 & 7: 0->5->10->11->12->13->14->9->4",
                st_w, ed_w, [2, 7], p_w, 0, 4, (-0.8, 6.8), (-0.6, 3.4), show_all_edge_costs=False)

fig.suptitle('Safe Semantic Planner — Visual Path Trajectories & Dynamic Replanning', fontsize=13, fontweight='bold', y=0.98)
plt.savefig('docs/images/planner_scenarios.png', dpi=300, facecolor='#ffffff')
plt.close()
print("Saved: docs/images/planner_scenarios.png")

# ==============================================================================
# 3. FIGURE: SYSTEM ARCHITECTURE DIAGRAM (system_architecture.png)
# ==============================================================================
fig, ax = plt.subplots(figsize=(12, 6.8), dpi=300)
fig.patch.set_facecolor('#ffffff')
ax.set_facecolor('#ffffff')
ax.set_xlim(0, 12)
ax.set_ylim(0, 6.8)
ax.axis('off')

# Title
ax.text(6.0, 6.45, 'Safe Semantic Planner — Architectural Pipeline & Data Flow',
        ha='center', va='center', fontsize=13, fontweight='bold', color='#111111')

def make_box(ax, x, y, w, h, header, lines):
    # Main box
    rect = patches.FancyBboxPatch((x, y), w, h, boxstyle="square,pad=0.0",
                                  facecolor='#fafafa', edgecolor='#222222', linewidth=1.2, zorder=2)
    ax.add_patch(rect)
    # Header bar
    hh = 0.48
    hbar = patches.Rectangle((x, y + h - hh), w, hh, facecolor='#111111', edgecolor='#111111', linewidth=1.2, zorder=3)
    ax.add_patch(hbar)
    ax.text(x + w/2, y + h - hh/2, header, ha='center', va='center', fontsize=8.5, fontweight='bold', color='#ffffff', zorder=4)
    # Body text
    for i, line in enumerate(lines):
        ax.text(x + 0.18, y + h - hh - 0.32 - i*0.34, line, fontsize=7.8, color='#222222', zorder=4, va='center')

# Module 1: Inputs
make_box(ax, 0.4, 1.4, 3.0, 4.4, '1. Problem Definition', [
    '• State Space S in R^d',
    '• Coordinate vectors x_i',
    '• Directed Transitions T',
    '• Bad States B (Strict Avoidance)',
    '• Start s_I  &  Goal s_G',
    '• Weights: beta, gamma, delta'
])

# Module 2: Safety Precomputation
make_box(ax, 3.9, 3.7, 3.7, 2.1, '2. Safety Distance Processing', [
    '• Euclidean Metric: ||s_i - s_j||_2',
    '• Safety Map: D_min(s) = min ||s - b||',
    '• Dual Adjacency: Forward & Reverse'
])

# Module 3: Multi-Objective Cost
make_box(ax, 3.9, 1.4, 3.7, 2.1, '3. Effective Cost Formulation', [
    '• c_eff(t) = beta*c + gamma*(1/D_min)',
    '            + delta*(1 - reliability)',
    '• Inadmissible edge pruning if t.to in B',
    '• Admissible Heuristic: beta * dist'
])

# Module 4: D* Lite Engine
make_box(ax, 8.1, 1.4, 3.5, 4.4, '4. D* Lite Incremental Engine', [
    '• Backward Search from Goal -> Start',
    '• G-Values & RHS Lookahead Values',
    '• Priority Queue: KeyPair(k1, k2)',
    '• UpdateVertex() & ComputeShortestPath()',
    '• Fast Incremental Repair O(k log n):',
    '  - Edge failure / addition',
    '  - Dynamic Goal relocation',
    '  - Hazard state toggling'
])

# Connecting Arrows
def link(ax, p1, p2):
    ax.annotate('', xy=p2, xytext=p1,
                arrowprops=dict(arrowstyle='->', color='#111111', lw=1.5, mutation_scale=12), zorder=5)

link(ax, (3.4, 4.7), (3.9, 4.7))
link(ax, (3.4, 2.4), (3.9, 2.4))
link(ax, (7.6, 4.7), (8.1, 4.0))
link(ax, (7.6, 2.4), (8.1, 2.8))

# Output bar at bottom
out_box = patches.FancyBboxPatch((0.4, 0.4), 11.2, 0.65, boxstyle="square,pad=0.0",
                                facecolor='#111111', edgecolor='#111111', linewidth=1.0, zorder=2)
ax.add_patch(out_box)
ax.text(6.0, 0.72, 'Outputs: Optimal State Sequence Path  |  Total Traversal Cost  |  Min Safety Margin (D_min)  |  Replanning Telemetry',
        ha='center', va='center', fontsize=8.2, fontweight='bold', color='#ffffff', zorder=4)

plt.tight_layout()
plt.savefig('docs/images/system_architecture.png', dpi=300, facecolor='#ffffff')
plt.close()
print("Saved: docs/images/system_architecture.png")
