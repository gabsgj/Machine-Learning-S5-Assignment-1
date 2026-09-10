/**
 * Orchestration Algorithm Web Application - LPA* Pathfinder Engine
 * Based on orchestration.c++
 */

// --- Data Structures & LPA* Algorithm ---

class LPAPlanner {
    static INF = Infinity;
    static EPS = 1e-9;

    static KeyLess(a, b) {
        if (Math.abs(a.k1 - b.k1) > LPAPlanner.EPS) return a.k1 < b.k1;
        return a.k2 < b.k2;
    }

    static KeyEquals(a, b) {
        return Math.abs(a.k1 - b.k1) <= LPAPlanner.EPS && Math.abs(a.k2 - b.k2) <= LPAPlanner.EPS;
    }

    static euclideanDist(a, b) {
        if (!a || !b) return 0;
        let sum = 0;
        const len = Math.min(a.length, b.length);
        for (let i = 0; i < len; i++) {
            sum += (a[i] - b[i]) * (a[i] - b[i]);
        }
        return Math.sqrt(sum);
    }

    plan(problem, alpha = 2.0) {
        const statesMap = new Map();
        const badStatesSet = new Set(problem.badStates);
        const succs = new Map();
        const preds = new Map();
        const g = new Map();
        const rhs = new Map();
        const safetyDist = new Map();

        const s_start = Number(problem.initialState);
        const s_goal = Number(problem.goalState);

        for (const s of problem.states) {
            statesMap.set(Number(s.id), s);
            succs.set(Number(s.id), []);
            preds.set(Number(s.id), []);
        }

        // Safety distance computation (min distance to any bad state)
        for (const s of problem.states) {
            const sid = Number(s.id);
            g.set(sid, LPAPlanner.INF);
            rhs.set(sid, LPAPlanner.INF);

            if (badStatesSet.size === 0 || badStatesSet.has(sid)) {
                safetyDist.set(sid, 0.0);
            } else {
                let minDist = LPAPlanner.INF;
                for (const bId of badStatesSet) {
                    const badStateObj = statesMap.get(Number(bId));
                    if (badStateObj) {
                        const dist = LPAPlanner.euclideanDist(s.embedding, badStateObj.embedding);
                        minDist = Math.min(minDist, dist);
                    }
                }
                safetyDist.set(sid, minDist);
            }
        }

        // Populate adjacency lists
        for (const t of problem.transitions) {
            const u = Number(t.from);
            const v = Number(t.to);
            if (succs.has(u)) {
                succs.get(u).push({ transId: t.id, to: v, cost: Number(t.cost), available: Boolean(t.available) });
            }
            if (preds.has(v)) {
                preds.get(v).push({ transId: t.id, to: u, cost: Number(t.cost), available: Boolean(t.available) });
            }
        }

        const heuristic = (u) => {
            const uObj = statesMap.get(u);
            const gObj = statesMap.get(s_goal);
            if (!uObj || !gObj) return 0.0;
            return LPAPlanner.euclideanDist(uObj.embedding, gObj.embedding);
        };

        const calculateKey = (u) => {
            const min_val = Math.min(g.get(u), rhs.get(u));
            return { k1: min_val + heuristic(u), k2: min_val };
        };

        let openList = [];

        const openListInsert = (u, k) => {
            openList.push({ key: k, id: u });
        };

        const openListRemove = (u) => {
            openList = openList.filter(item => item.id !== u);
        };

        const openListContains = (u) => {
            return openList.some(item => item.id === u);
        };

        const openListTopKey = () => {
            if (openList.length === 0) return { k1: LPAPlanner.INF, k2: LPAPlanner.INF };
            let minItem = openList[0];
            for (let i = 1; i < openList.length; i++) {
                if (LPAPlanner.KeyLess(openList[i].key, minItem.key)) {
                    minItem = openList[i];
                }
            }
            return minItem.key;
        };

        const openListPop = () => {
            let minIndex = 0;
            for (let i = 1; i < openList.length; i++) {
                if (LPAPlanner.KeyLess(openList[i].key, openList[minIndex].key)) {
                    minIndex = i;
                }
            }
            const node = openList[minIndex].id;
            openList.splice(minIndex, 1);
            return node;
        };

        const getEffectiveCost = (e) => {
            if (!e.available || badStatesSet.has(e.to)) return LPAPlanner.INF;
            const distToBad = safetyDist.get(e.to) || 0.0;
            const safetyFactor = (distToBad > 0.0) ? (alpha / distToBad) : 0.0;
            return e.cost + safetyFactor;
        };

        const updateVertex = (u) => {
            if (u !== s_start) {
                rhs.set(u, LPAPlanner.INF);
                const pList = preds.get(u) || [];
                for (const p of pList) {
                    const c = getEffectiveCost(p);
                    if (c < LPAPlanner.INF && g.get(p.to) < LPAPlanner.INF) {
                        rhs.set(u, Math.min(rhs.get(u), g.get(p.to) + c));
                    }
                }
            }
            if (openListContains(u)) {
                openListRemove(u);
            }
            if (Math.abs(g.get(u) - rhs.get(u)) > LPAPlanner.EPS) {
                openListInsert(u, calculateKey(u));
            }
        };

        // Initialize LPA*
        rhs.set(s_start, 0.0);
        openListInsert(s_start, calculateKey(s_start));

        // Shortest Path Loop
        const computeShortestPath = () => {
            let iterations = 0;
            const maxIter = (problem.states.length + 10) * 50;

            while (openList.length > 0 && iterations++ < maxIter) {
                const topKey = openListTopKey();
                const goalKey = calculateKey(s_goal);

                if (!LPAPlanner.KeyLess(topKey, goalKey) && Math.abs(rhs.get(s_goal) - g.get(s_goal)) <= LPAPlanner.EPS) {
                    break;
                }

                const u = openListPop();
                if (g.get(u) > rhs.get(u)) {
                    g.set(u, rhs.get(u));
                    for (const s of (succs.get(u) || [])) {
                        updateVertex(s.to);
                    }
                } else {
                    g.set(u, LPAPlanner.INF);
                    updateVertex(u);
                    for (const s of (succs.get(u) || [])) {
                        updateVertex(s.to);
                    }
                }
            }
        };

        computeShortestPath();

        const result = {
            success: false,
            statePath: [],
            transitionPath: [],
            totalCost: 0.0,
            safetyScore: 0.0
        };

        if ((g.get(s_goal) || LPAPlanner.INF) >= LPAPlanner.INF) {
            result.success = false;
            return result;
        }

        // Path Reconstruction
        let curr = s_goal;
        result.statePath.push(curr);
        let minSafety = safetyDist.get(curr) || 0.0;

        let stepsCount = 0;
        const maxSteps = problem.states.length + 5;

        while (curr !== s_start && stepsCount++ < maxSteps) {
            let bestPrev = curr;
            let bestTrans = 0;
            let minCost = LPAPlanner.INF;

            const pList = preds.get(curr) || [];
            for (const p of pList) {
                const c = getEffectiveCost(p);
                const gVal = g.get(p.to);
                if (c < LPAPlanner.INF && gVal < LPAPlanner.INF) {
                    if (gVal + c < minCost) {
                        minCost = gVal + c;
                        bestPrev = p.to;
                        bestTrans = p.transId;
                    }
                }
            }

            if (bestPrev === curr) {
                result.success = false;
                return result;
            }

            result.totalCost += minCost - g.get(bestPrev);
            result.transitionPath.push(bestTrans);
            curr = bestPrev;
            result.statePath.push(curr);
            minSafety = Math.min(minSafety, safetyDist.get(curr) || 0.0);
        }

        result.statePath.reverse();
        result.transitionPath.reverse();
        result.success = true;
        result.safetyScore = minSafety;

        return result;
    }
}

// --- Application Core & Preset Definitions ---

const PRESETS = {
    default_cpp: {
        initialState: 1,
        goalState: 4,
        badStates: [3],
        alpha: 2.0,
        states: [
            { id: 1, embedding: [100, 250] },
            { id: 2, embedding: [300, 100] },
            { id: 3, embedding: [300, 400] }, // Bad state
            { id: 4, embedding: [500, 250] }
        ],
        transitions: [
            { id: 101, from: 1, to: 3, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 102, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 103, from: 1, to: 2, cost: 2.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 104, from: 2, to: 4, cost: 2.0, safety: 1.0, reliability: 1.0, available: true }
        ]
    },
    amr_navigation: {
        initialState: 1,
        goalState: 8,
        badStates: [4, 6],
        alpha: 3.0,
        states: [
            { id: 1, embedding: [80, 200] },
            { id: 2, embedding: [220, 100] },
            { id: 3, embedding: [220, 320] },
            { id: 4, embedding: [360, 200] }, // Bad state 1
            { id: 5, embedding: [480, 100] },
            { id: 6, embedding: [480, 320] }, // Bad state 2
            { id: 7, embedding: [620, 120] },
            { id: 8, embedding: [720, 220] }
        ],
        transitions: [
            { id: 201, from: 1, to: 2, cost: 1.5, safety: 1.0, reliability: 1.0, available: true },
            { id: 202, from: 1, to: 3, cost: 1.8, safety: 1.0, reliability: 1.0, available: true },
            { id: 203, from: 2, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 204, from: 3, to: 4, cost: 1.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 205, from: 2, to: 5, cost: 2.8, safety: 1.0, reliability: 1.0, available: true },
            { id: 206, from: 3, to: 6, cost: 2.5, safety: 1.0, reliability: 1.0, available: true },
            { id: 207, from: 4, to: 5, cost: 1.2, safety: 1.0, reliability: 1.0, available: true },
            { id: 208, from: 5, to: 7, cost: 1.5, safety: 1.0, reliability: 1.0, available: true },
            { id: 209, from: 6, to: 8, cost: 2.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 210, from: 7, to: 8, cost: 1.4, safety: 1.0, reliability: 1.0, available: true }
        ]
    },
    microservices: {
        initialState: 10,
        goalState: 40,
        badStates: [25],
        alpha: 2.5,
        states: [
            { id: 10, embedding: [100, 250] }, // Gateway
            { id: 20, embedding: [300, 150] }, // Auth Service A
            { id: 25, embedding: [300, 350] }, // Auth Service B (Failing/Compromised)
            { id: 30, embedding: [500, 150] }, // DB Replica
            { id: 40, embedding: [700, 250] }  // Payment Core
        ],
        transitions: [
            { id: 301, from: 10, to: 20, cost: 1.2, safety: 0.95, reliability: 0.99, available: true },
            { id: 302, from: 10, to: 25, cost: 0.8, safety: 0.20, reliability: 0.50, available: true },
            { id: 303, from: 20, to: 30, cost: 1.0, safety: 0.99, reliability: 0.99, available: true },
            { id: 304, from: 25, to: 40, cost: 0.9, safety: 0.10, reliability: 0.40, available: true },
            { id: 305, from: 30, to: 40, cost: 1.5, safety: 0.98, reliability: 0.99, available: true }
        ]
    },
    drone_corridor: {
        initialState: 100,
        goalState: 105,
        badStates: [102],
        alpha: 4.0,
        states: [
            { id: 100, embedding: [100, 200] },
            { id: 101, embedding: [280, 120] },
            { id: 102, embedding: [280, 280] }, // Radar Hazard Zone
            { id: 103, embedding: [460, 120] },
            { id: 104, embedding: [460, 300] },
            { id: 105, embedding: [650, 200] }
        ],
        transitions: [
            { id: 401, from: 100, to: 101, cost: 2.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 402, from: 100, to: 102, cost: 1.5, safety: 0.2, reliability: 1.0, available: true },
            { id: 403, from: 101, to: 103, cost: 2.0, safety: 1.0, reliability: 1.0, available: true },
            { id: 404, from: 102, to: 104, cost: 1.8, safety: 0.3, reliability: 1.0, available: true },
            { id: 405, from: 103, to: 105, cost: 2.2, safety: 1.0, reliability: 1.0, available: true },
            { id: 406, from: 104, to: 105, cost: 2.5, safety: 0.9, reliability: 1.0, available: true }
        ]
    }
};

class OrchestrationApp {
    constructor() {
        this.problem = {
            initialState: 1,
            goalState: 4,
            badStates: [3],
            alpha: 2.0,
            states: [],
            transitions: []
        };

        this.planner = new LPAPlanner();
        this.lastResult = null;

        // Viewport & Drag state
        this.viewOffset = { x: 50, y: 50 };
        this.zoom = 1.0;
        this.isPanning = false;
        this.panStart = { x: 0, y: 0 };

        this.selectedNodeId = null;
        this.draggingNodeId = null;
        this.dragOffset = { x: 0, y: 0 };
        this.shiftSelectedNodeId = null;

        this.showGrid = true;
        this.showSafetyAura = true;

        this.initCanvas();
        this.initUI();
        this.loadPreset('default_cpp');
    }

    initCanvas() {
        this.canvas = document.getElementById('graphCanvas');
        this.ctx = this.canvas.getContext('2d');

        this.resizeCanvas();
        window.addEventListener('resize', () => this.resizeCanvas());

        // Mouse interactions
        this.canvas.addEventListener('mousedown', (e) => this.handleMouseDown(e));
        this.canvas.addEventListener('mousemove', (e) => this.handleMouseMove(e));
        this.canvas.addEventListener('mouseup', () => this.handleMouseUp());
        this.canvas.addEventListener('wheel', (e) => this.handleWheel(e), { passive: false });
        this.canvas.addEventListener('contextmenu', (e) => this.handleContextMenu(e));
    }

    resizeCanvas() {
        const wrapper = this.canvas.parentElement;
        this.canvas.width = wrapper.clientWidth * window.devicePixelRatio;
        this.canvas.height = wrapper.clientHeight * window.devicePixelRatio;
        this.requestRender();
    }

    worldToScreen(x, y) {
        return {
            x: (x * this.zoom) + this.viewOffset.x,
            y: (y * this.zoom) + this.viewOffset.y
        };
    }

    screenToWorld(sx, sy) {
        return {
            x: (sx - this.viewOffset.x) / this.zoom,
            y: (sy - this.viewOffset.y) / this.zoom
        };
    }

    initUI() {
        // Dropdowns & Controls
        document.getElementById('presetDropdown').addEventListener('change', (e) => {
            this.loadPreset(e.target.value);
        });

        document.getElementById('selectInitialState').addEventListener('change', (e) => {
            this.problem.initialState = Number(e.target.value);
            this.solvePath();
            this.requestRender();
        });

        document.getElementById('selectGoalState').addEventListener('change', (e) => {
            this.problem.goalState = Number(e.target.value);
            this.solvePath();
            this.requestRender();
        });

        document.getElementById('btnAddBadState').addEventListener('click', () => {
            const sel = document.getElementById('selectBadStateToAdd');
            const sid = Number(sel.value);
            if (sid && !this.problem.badStates.includes(sid)) {
                this.problem.badStates.push(sid);
                this.updateUI();
                this.solvePath();
                this.requestRender();
            }
        });

        const alphaInput = document.getElementById('inputAlpha');
        alphaInput.addEventListener('input', (e) => {
            this.problem.alpha = parseFloat(e.target.value);
            document.getElementById('alphaValueDisplay').innerText = this.problem.alpha.toFixed(1);
            this.solvePath();
            this.requestRender();
        });

        document.getElementById('btnRunPlanner').addEventListener('click', () => {
            this.solvePath();
            this.requestRender();
        });

        // Quick Node Creation
        document.getElementById('btnAddNodeQuick').addEventListener('click', () => {
            const nextId = this.problem.states.length > 0 ? Math.max(...this.problem.states.map(s => s.id)) + 1 : 1;
            const center = this.screenToWorld(this.canvas.width / (2 * window.devicePixelRatio), this.canvas.height / (2 * window.devicePixelRatio));
            this.problem.states.push({ id: nextId, embedding: [Math.round(center.x), Math.round(center.y)] });
            this.updateUI();
            this.solvePath();
            this.requestRender();
        });

        document.getElementById('btnClearGraph').addEventListener('click', () => {
            if (confirm('Clear all states and transitions?')) {
                this.problem.states = [];
                this.problem.transitions = [];
                this.problem.badStates = [];
                this.problem.initialState = 0;
                this.problem.goalState = 0;
                this.updateUI();
                this.solvePath();
                this.requestRender();
            }
        });

        // Toolbar Buttons
        document.getElementById('btnZoomIn').addEventListener('click', () => { this.zoom *= 1.2; this.requestRender(); });
        document.getElementById('btnZoomOut').addEventListener('click', () => { this.zoom /= 1.2; this.requestRender(); });
        document.getElementById('btnResetView').addEventListener('click', () => {
            this.viewOffset = { x: 50, y: 50 };
            this.zoom = 1.0;
            this.requestRender();
        });
        document.getElementById('btnToggleGrid').addEventListener('click', (e) => {
            this.showGrid = !this.showGrid;
            e.currentTarget.classList.toggle('active', this.showGrid);
            this.requestRender();
        });
        document.getElementById('btnToggleSafetyAura').addEventListener('click', (e) => {
            this.showSafetyAura = !this.showSafetyAura;
            e.currentTarget.classList.toggle('active', this.showSafetyAura);
            this.requestRender();
        });

        // Modals Logic
        document.getElementById('btnExportCpp').addEventListener('click', () => this.openCppModal());
        document.getElementById('btnOpenStateModal').addEventListener('click', () => this.openStatesModal());
        document.getElementById('btnOpenTransModal').addEventListener('click', () => this.openTransitionsModal());

        document.querySelectorAll('.btnCloseModal').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const targetId = e.currentTarget.getAttribute('data-modal');
                document.getElementById(targetId).classList.remove('active');
            });
        });

        document.getElementById('btnCopyCppCode').addEventListener('click', () => {
            const code = document.getElementById('cppCodeContainer').innerText;
            navigator.clipboard.writeText(code);
            alert('C++ setup code copied to clipboard!');
        });

        // Import & Export JSON
        document.getElementById('btnExportJson').addEventListener('click', () => {
            const jsonStr = JSON.stringify(this.problem, null, 2);
            const blob = new Blob([jsonStr], { type: 'application/json' });
            const url = URL.createObjectURL(blob);
            const a = document.createElement('a');
            a.href = url;
            a.download = `orchestration_problem_${Date.now()}.json`;
            a.click();
            URL.revokeObjectURL(url);
        });

        document.getElementById('importJsonFile').addEventListener('change', (e) => {
            const file = e.target.files[0];
            if (!file) return;
            const reader = new FileReader();
            reader.onload = (evt) => {
                try {
                    const imported = JSON.parse(evt.target.result);
                    if (imported.states && imported.transitions) {
                        this.problem = imported;
                        this.updateUI();
                        this.solvePath();
                        this.requestRender();
                    }
                } catch (err) {
                    alert('Invalid JSON file format.');
                }
            };
            reader.readAsText(file);
        });
    }

    loadPreset(key) {
        if (!PRESETS[key]) return;
        this.problem = JSON.parse(JSON.stringify(PRESETS[key]));
        document.getElementById('inputAlpha').value = this.problem.alpha;
        document.getElementById('alphaValueDisplay').innerText = this.problem.alpha.toFixed(1);
        this.updateUI();
        this.solvePath();
        this.requestRender();
    }

    updateUI() {
        const stateSelectInitial = document.getElementById('selectInitialState');
        const stateSelectGoal = document.getElementById('selectGoalState');
        const stateSelectBad = document.getElementById('selectBadStateToAdd');

        stateSelectInitial.innerHTML = '';
        stateSelectGoal.innerHTML = '';
        stateSelectBad.innerHTML = '';

        for (const s of this.problem.states) {
            const opt1 = new Option(`State ${s.id}`, s.id, false, s.id === this.problem.initialState);
            const opt2 = new Option(`State ${s.id}`, s.id, false, s.id === this.problem.goalState);
            const opt3 = new Option(`State ${s.id}`, s.id);
            stateSelectInitial.add(opt1);
            stateSelectGoal.add(opt2);
            stateSelectBad.add(opt3);
        }

        // Bad state chips
        const badContainer = document.getElementById('badStatesContainer');
        badContainer.innerHTML = '';
        for (const bId of this.problem.badStates) {
            const chip = document.createElement('div');
            chip.className = 'chip chip-bad';
            chip.innerHTML = `<span>State ${bId}</span><span class="remove-btn" data-id="${bId}">&times;</span>`;
            chip.querySelector('.remove-btn').addEventListener('click', (e) => {
                const removeId = Number(e.currentTarget.getAttribute('data-id'));
                this.problem.badStates = this.problem.badStates.filter(id => id !== removeId);
                this.updateUI();
                this.solvePath();
                this.requestRender();
            });
            badContainer.appendChild(chip);
        }

        // Count Badges
        document.getElementById('stateCountBadge').innerText = this.problem.states.length;
        document.getElementById('transCountBadge').innerText = this.problem.transitions.length;
    }

    solvePath() {
        if (!this.problem.initialState || !this.problem.goalState) {
            this.lastResult = null;
            this.renderResults();
            return;
        }

        this.lastResult = this.planner.plan(this.problem, this.problem.alpha);
        this.renderResults();
    }

    renderResults() {
        const statusEl = document.getElementById('resultStatus');
        const costEl = document.getElementById('resultTotalCost');
        const safetyEl = document.getElementById('resultSafetyScore');
        const lenEl = document.getElementById('resultPathLength');
        const flowEl = document.getElementById('resultPathFlow');

        if (!this.lastResult || !this.lastResult.success) {
            statusEl.innerText = 'No Path Found';
            statusEl.className = 'stat-value failed';
            costEl.innerText = 'INF';
            safetyEl.innerText = 'N/A';
            lenEl.innerText = '0 steps';
            flowEl.innerHTML = `<span style="color: var(--accent-red); font-size: 0.8rem;">Goal is unreachable or blocked by bad states.</span>`;
            return;
        }

        statusEl.innerText = 'Optimal Plan Found';
        statusEl.className = 'stat-value success';
        costEl.innerText = this.lastResult.totalCost.toFixed(3);
        safetyEl.innerText = this.lastResult.safetyScore === Infinity ? 'Safe' : this.lastResult.safetyScore.toFixed(3);
        lenEl.innerText = `${this.lastResult.statePath.length} states`;

        flowEl.innerHTML = '';
        this.lastResult.statePath.forEach((sid, idx) => {
            const badge = document.createElement('div');
            badge.className = 'path-node-badge';
            badge.innerText = `S${sid}`;
            flowEl.appendChild(badge);

            if (idx < this.lastResult.statePath.length - 1) {
                const arrow = document.createElement('span');
                arrow.className = 'path-arrow';
                const transId = this.lastResult.transitionPath[idx];
                arrow.innerHTML = `<i class="fa-solid fa-arrow-right"></i> <small style="font-size: 0.65rem;">(t:${transId})</small>`;
                flowEl.appendChild(arrow);
            }
        });
    }

    // --- Mouse Handlers & Canvas Visualizer ---

    getMousePos(e) {
        const rect = this.canvas.getBoundingClientRect();
        return {
            x: (e.clientX - rect.left),
            y: (e.clientY - rect.top)
        };
    }

    findNodeAtScreenPos(sx, sy) {
        const radiusScreen = 24 * this.zoom;
        for (const s of this.problem.states) {
            const sp = this.worldToScreen(s.embedding[0], s.embedding[1]);
            const dist = Math.hypot(sp.x - sx, sp.y - sy);
            if (dist <= radiusScreen) {
                return s;
            }
        }
        return null;
    }

    handleMouseDown(e) {
        const mouse = this.getMousePos(e);
        const hitNode = this.findNodeAtScreenPos(mouse.x, mouse.y);

        if (e.shiftKey && hitNode) {
            if (!this.shiftSelectedNodeId) {
                this.shiftSelectedNodeId = hitNode.id;
            } else if (this.shiftSelectedNodeId !== hitNode.id) {
                // Connect transition
                const nextTransId = this.problem.transitions.length > 0 ? Math.max(...this.problem.transitions.map(t => t.id)) + 1 : 101;
                this.problem.transitions.push({
                    id: nextTransId,
                    from: this.shiftSelectedNodeId,
                    to: hitNode.id,
                    cost: 1.0,
                    safety: 1.0,
                    reliability: 1.0,
                    available: true
                });
                this.shiftSelectedNodeId = null;
                this.updateUI();
                this.solvePath();
            }
            this.requestRender();
            return;
        }

        this.shiftSelectedNodeId = null;

        if (hitNode) {
            this.draggingNodeId = hitNode.id;
            this.selectedNodeId = hitNode.id;
            const sp = this.worldToScreen(hitNode.embedding[0], hitNode.embedding[1]);
            this.dragOffset = { x: mouse.x - sp.x, y: mouse.y - sp.y };
        } else {
            this.selectedNodeId = null;
            this.isPanning = true;
            this.panStart = { x: mouse.x - this.viewOffset.x, y: mouse.y - this.viewOffset.y };
        }

        this.requestRender();
    }

    handleMouseMove(e) {
        const mouse = this.getMousePos(e);

        if (this.draggingNodeId) {
            const targetNode = this.problem.states.find(s => s.id === this.draggingNodeId);
            if (targetNode) {
                const worldPos = this.screenToWorld(mouse.x - this.dragOffset.x, mouse.y - this.dragOffset.y);
                targetNode.embedding[0] = Math.round(worldPos.x);
                targetNode.embedding[1] = Math.round(worldPos.y);
                this.solvePath();
                this.requestRender();
            }
        } else if (this.isPanning) {
            this.viewOffset.x = mouse.x - this.panStart.x;
            this.viewOffset.y = mouse.y - this.panStart.y;
            this.requestRender();
        }
    }

    handleMouseUp() {
        this.draggingNodeId = null;
        this.isPanning = false;
    }

    handleWheel(e) {
        e.preventDefault();
        const zoomFactor = e.deltaY < 0 ? 1.1 : 0.9;
        const mouse = this.getMousePos(e);
        const worldBefore = this.screenToWorld(mouse.x, mouse.y);

        this.zoom *= zoomFactor;
        this.zoom = Math.max(0.2, Math.min(5.0, this.zoom));

        const worldAfter = this.screenToWorld(mouse.x, mouse.y);
        this.viewOffset.x += (worldAfter.x - worldBefore.x) * this.zoom;
        this.viewOffset.y += (worldAfter.y - worldBefore.y) * this.zoom;

        this.requestRender();
    }

    handleContextMenu(e) {
        e.preventDefault();
        const mouse = this.getMousePos(e);
        const hitNode = this.findNodeAtScreenPos(mouse.x, mouse.y);
        if (hitNode) {
            const idx = this.problem.badStates.indexOf(hitNode.id);
            if (idx >= 0) {
                this.problem.badStates.splice(idx, 1);
            } else {
                this.problem.badStates.push(hitNode.id);
            }
            this.updateUI();
            this.solvePath();
            this.requestRender();
        }
    }

    requestRender() {
        if (!this.animFrameRequested) {
            this.animFrameRequested = true;
            requestAnimationFrame(() => {
                this.animFrameRequested = false;
                this.render();
            });
        }
    }

    // --- Rendering Engine ---

    render() {
        const dpr = window.devicePixelRatio || 1;
        this.ctx.save();
        this.ctx.scale(dpr, dpr);
        this.ctx.clearRect(0, 0, this.canvas.width / dpr, this.canvas.height / dpr);

        if (this.showGrid) {
            this.drawGrid();
        }

        if (this.showSafetyAura && this.problem.badStates.length > 0) {
            this.drawSafetyAuras();
        }

        this.drawTransitions();
        this.drawNodes();

        this.ctx.restore();
    }

    drawGrid() {
        const width = this.canvas.width / window.devicePixelRatio;
        const height = this.canvas.height / window.devicePixelRatio;
        const gridSize = 40 * this.zoom;

        this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
        this.ctx.lineWidth = 1;

        const startX = this.viewOffset.x % gridSize;
        const startY = this.viewOffset.y % gridSize;

        this.ctx.beginPath();
        for (let x = startX; x < width; x += gridSize) {
            this.ctx.moveTo(x, 0);
            this.ctx.lineTo(x, height);
        }
        for (let y = startY; y < height; y += gridSize) {
            this.ctx.moveTo(0, y);
            this.ctx.lineTo(width, y);
        }
        this.ctx.stroke();
    }

    drawSafetyAuras() {
        for (const bId of this.problem.badStates) {
            const badNode = this.problem.states.find(s => s.id === bId);
            if (!badNode) continue;
            const sp = this.worldToScreen(badNode.embedding[0], badNode.embedding[1]);

            // Draw radial safety penalty gradient
            const auraRadius = 160 * this.zoom;
            const grad = this.ctx.createRadialGradient(sp.x, sp.y, 20 * this.zoom, sp.x, sp.y, auraRadius);
            grad.addColorStop(0, 'rgba(255, 71, 87, 0.35)');
            grad.addColorStop(0.5, 'rgba(255, 71, 87, 0.12)');
            grad.addColorStop(1, 'rgba(255, 71, 87, 0)');

            this.ctx.fillStyle = grad;
            this.ctx.beginPath();
            this.ctx.arc(sp.x, sp.y, auraRadius, 0, Math.PI * 2);
            this.ctx.fill();
        }
    }

    drawTransitions() {
        const pathTransSet = new Set(this.lastResult && this.lastResult.success ? this.lastResult.transitionPath : []);

        for (const t of this.problem.transitions) {
            const uNode = this.problem.states.find(s => s.id === t.from);
            const vNode = this.problem.states.find(s => s.id === t.to);
            if (!uNode || !vNode) continue;

            const uPos = this.worldToScreen(uNode.embedding[0], uNode.embedding[1]);
            const vPos = this.worldToScreen(vNode.embedding[0], vNode.embedding[1]);

            const isInPath = pathTransSet.has(t.id);
            const isAvailable = t.available;

            // Check if reverse transition exists to curve double edges
            const hasReverse = this.problem.transitions.some(other => other.from === t.to && other.to === t.from);

            this.ctx.save();
            if (isInPath) {
                this.ctx.strokeStyle = '#ffb703';
                this.ctx.lineWidth = 4 * this.zoom;
                this.ctx.shadowColor = '#ffb703';
                this.ctx.shadowBlur = 12;
            } else if (!isAvailable) {
                this.ctx.strokeStyle = 'rgba(255, 255, 255, 0.15)';
                this.ctx.lineWidth = 1.5 * this.zoom;
                this.ctx.setLineDash([4, 4]);
            } else {
                this.ctx.strokeStyle = 'rgba(0, 242, 254, 0.4)';
                this.ctx.lineWidth = 2 * this.zoom;
            }

            const dx = vPos.x - uPos.x;
            const dy = vPos.y - uPos.y;
            const angle = Math.atan2(dy, dx);
            const nodeRadius = 22 * this.zoom;

            let startX = uPos.x;
            let startY = uPos.y;
            let endX = vPos.x;
            let endY = vPos.y;
            let midX = (startX + endX) / 2;
            let midY = (startY + endY) / 2;

            if (hasReverse) {
                // Offset curve
                const offset = 30 * this.zoom;
                const normX = -Math.sin(angle);
                const normY = Math.cos(angle);
                midX += normX * offset;
                midY += normY * offset;

                this.ctx.beginPath();
                this.ctx.moveTo(startX, startY);
                this.ctx.quadraticCurveTo(midX, midY, endX, endY);
                this.ctx.stroke();
            } else {
                this.ctx.beginPath();
                this.ctx.moveTo(startX, startY);
                this.ctx.lineTo(endX, endY);
                this.ctx.stroke();
            }

            // Arrow head
            const arrowAngle = hasReverse ? Math.atan2(endY - midY, endX - midX) : angle;
            const arrowX = endX - nodeRadius * Math.cos(arrowAngle);
            const arrowY = endY - nodeRadius * Math.sin(arrowAngle);

            this.ctx.fillStyle = isInPath ? '#ffb703' : (isAvailable ? '#00f2fe' : 'rgba(255, 255, 255, 0.3)');
            this.ctx.beginPath();
            this.ctx.moveTo(arrowX, arrowY);
            this.ctx.lineTo(arrowX - 10 * this.zoom * Math.cos(arrowAngle - Math.PI / 6), arrowY - 10 * this.zoom * Math.sin(arrowAngle - Math.PI / 6));
            this.ctx.lineTo(arrowX - 10 * this.zoom * Math.cos(arrowAngle + Math.PI / 6), arrowY - 10 * this.zoom * Math.sin(arrowAngle + Math.PI / 6));
            this.ctx.closePath();
            this.ctx.fill();

            // Label (Cost)
            this.ctx.font = `${Math.max(10, 11 * this.zoom)}px var(--font-mono)`;
            this.ctx.fillStyle = isInPath ? '#ffb703' : 'rgba(255, 255, 255, 0.7)';
            const labelX = hasReverse ? (startX + 2 * midX + endX) / 4 : midX;
            const labelY = hasReverse ? (startY + 2 * midY + endY) / 4 : midY - 6;
            this.ctx.fillText(`c:${t.cost}`, labelX, labelY);

            this.ctx.restore();
        }
    }

    drawNodes() {
        const pathStateSet = new Set(this.lastResult && this.lastResult.success ? this.lastResult.statePath : []);

        for (const s of this.problem.states) {
            const sp = this.worldToScreen(s.embedding[0], s.embedding[1]);
            const radius = 22 * this.zoom;

            const isInitial = s.id === this.problem.initialState;
            const isGoal = s.id === this.problem.goalState;
            const isBad = this.problem.badStates.includes(s.id);
            const isInPath = pathStateSet.has(s.id);
            const isSelected = s.id === this.selectedNodeId || s.id === this.shiftSelectedNodeId;

            this.ctx.save();

            // Outer Glow & Circle Fill
            this.ctx.beginPath();
            this.ctx.arc(sp.x, sp.y, radius, 0, Math.PI * 2);

            if (isBad) {
                this.ctx.fillStyle = '#2a0a10';
                this.ctx.strokeStyle = '#ff4757';
                this.ctx.shadowColor = '#ff4757';
                this.ctx.shadowBlur = isSelected ? 20 : 10;
            } else if (isInitial) {
                this.ctx.fillStyle = '#0a2a18';
                this.ctx.strokeStyle = '#2ed573';
                this.ctx.shadowColor = '#2ed573';
                this.ctx.shadowBlur = 20;
            } else if (isGoal) {
                this.ctx.fillStyle = '#200a35';
                this.ctx.strokeStyle = '#c77dff';
                this.ctx.shadowColor = '#c77dff';
                this.ctx.shadowBlur = 20;
            } else if (isInPath) {
                this.ctx.fillStyle = '#2a1a00';
                this.ctx.strokeStyle = '#ffb703';
                this.ctx.shadowColor = '#ffb703';
                this.ctx.shadowBlur = 15;
            } else {
                this.ctx.fillStyle = '#0f172a';
                this.ctx.strokeStyle = isSelected ? '#00f2fe' : 'rgba(255, 255, 255, 0.2)';
                this.ctx.shadowBlur = isSelected ? 12 : 0;
                this.ctx.shadowColor = '#00f2fe';
            }

            this.ctx.lineWidth = isSelected ? 4 * this.zoom : 2.5 * this.zoom;
            this.ctx.fill();
            this.ctx.stroke();

            // Node ID Label
            this.ctx.font = `bold ${Math.max(11, 13 * this.zoom)}px var(--font-mono)`;
            this.ctx.fillStyle = '#ffffff';
            this.ctx.textAlign = 'center';
            this.ctx.textBaseline = 'middle';
            this.ctx.fillText(`S${s.id}`, sp.x, sp.y);

            // Sub-label (Coordinates)
            this.ctx.font = `${Math.max(9, 10 * this.zoom)}px var(--font-main)`;
            this.ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
            this.ctx.fillText(`[${s.embedding[0]}, ${s.embedding[1]}]`, sp.x, sp.y + radius + 14 * this.zoom);

            this.ctx.restore();
        }
    }

    // --- Modal Tables & Code Generation ---

    openCppModal() {
        const modal = document.getElementById('modalCppExport');
        const codeBlock = document.getElementById('cppCodeContainer');

        let statesCpp = this.problem.states.map(s => `        {${s.id}, {${s.embedding.join(', ')}}}`).join(',\n');
        let badCpp = this.problem.badStates.join(', ');
        let transCpp = this.problem.transitions.map(t =>
            `        {${t.id}, ${t.from}, ${t.to}, ${t.cost.toFixed(1)}, ${t.safety.toFixed(1)}, ${t.reliability.toFixed(1)}, ${t.available}}`
        ).join(',\n');

        const code = `int main() {
    PlanningProblem problem;
    problem.initialState = ${this.problem.initialState};
    problem.goalState = ${this.problem.goalState};
    problem.badStates = {${badCpp}};

    problem.states = {
${statesCpp}
    };

    problem.transitions = {
${transCpp}
    };

    LPAPlanner planner;
    PlanningResult res = planner.plan(problem);

    if (res.success) {
        std::cout << "Plan found successfully!\\nPath: ";
        for (uint64_t sid : res.statePath) std::cout << sid << " ";
        std::cout << "\\nTotal Cost: " << res.totalCost << "\\nMin Safety Distance: " << res.safetyScore << std::endl;
    } else {
        std::cout << "No path found." << std::endl;
    }

    return 0;
}`;

        codeBlock.innerText = code;
        modal.classList.add('active');
    }

    openStatesModal() {
        const modal = document.getElementById('modalStatesManager');
        const tbody = document.getElementById('statesTableBody');
        tbody.innerHTML = '';

        this.problem.states.forEach(s => {
            const tr = document.createElement('tr');
            const isInitial = s.id === this.problem.initialState;
            const isGoal = s.id === this.problem.goalState;
            const isBad = this.problem.badStates.includes(s.id);
            const typeStr = isInitial ? 'Initial' : (isGoal ? 'Goal' : (isBad ? 'Bad State' : 'Normal'));

            tr.innerHTML = `
                <td><strong>S${s.id}</strong></td>
                <td><input type="number" value="${s.embedding[0]}" data-id="${s.id}" data-idx="0" class="inputStateEmb"></td>
                <td><input type="number" value="${s.embedding[1]}" data-id="${s.id}" data-idx="1" class="inputStateEmb"></td>
                <td><span class="chip ${isBad ? 'chip-bad' : ''}">${typeStr}</span></td>
                <td><button class="btn btn-danger btn-sm btnDeleteState" data-id="${s.id}"><i class="fa-solid fa-trash"></i></button></td>
            `;
            tbody.appendChild(tr);
        });

        tbody.querySelectorAll('.inputStateEmb').forEach(inp => {
            inp.addEventListener('change', (e) => {
                const sid = Number(e.target.getAttribute('data-id'));
                const idx = Number(e.target.getAttribute('data-idx'));
                const state = this.problem.states.find(st => st.id === sid);
                if (state) {
                    state.embedding[idx] = Number(e.target.value);
                    this.solvePath();
                    this.requestRender();
                }
            });
        });

        tbody.querySelectorAll('.btnDeleteState').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const sid = Number(e.currentTarget.getAttribute('data-id'));
                this.problem.states = this.problem.states.filter(st => st.id !== sid);
                this.problem.transitions = this.problem.transitions.filter(t => t.from !== sid && t.to !== sid);
                this.problem.badStates = this.problem.badStates.filter(b => b !== sid);
                this.updateUI();
                this.openStatesModal();
                this.solvePath();
                this.requestRender();
            });
        });

        document.getElementById('btnModalAddState').onclick = () => {
            const nextId = this.problem.states.length > 0 ? Math.max(...this.problem.states.map(s => s.id)) + 1 : 1;
            this.problem.states.push({ id: nextId, embedding: [200, 200] });
            this.updateUI();
            this.openStatesModal();
            this.solvePath();
            this.requestRender();
        };

        modal.classList.add('active');
    }

    openTransitionsModal() {
        const modal = document.getElementById('modalTransitionsManager');
        const tbody = document.getElementById('transitionsTableBody');
        tbody.innerHTML = '';

        this.problem.transitions.forEach(t => {
            const tr = document.createElement('tr');
            tr.innerHTML = `
                <td><strong>t:${t.id}</strong></td>
                <td>S${t.from}</td>
                <td>S${t.to}</td>
                <td><input type="number" step="0.1" value="${t.cost}" data-id="${t.id}" data-field="cost" class="inputTransField"></td>
                <td><input type="number" step="0.1" value="${t.safety}" data-id="${t.id}" data-field="safety" class="inputTransField"></td>
                <td><input type="number" step="0.1" value="${t.reliability}" data-id="${t.id}" data-field="reliability" class="inputTransField"></td>
                <td><input type="checkbox" ${t.available ? 'checked' : ''} data-id="${t.id}" class="chkTransAvail"></td>
                <td><button class="btn btn-danger btn-sm btnDeleteTrans" data-id="${t.id}"><i class="fa-solid fa-trash"></i></button></td>
            `;
            tbody.appendChild(tr);
        });

        tbody.querySelectorAll('.inputTransField').forEach(inp => {
            inp.addEventListener('change', (e) => {
                const tid = Number(e.target.getAttribute('data-id'));
                const field = e.target.getAttribute('data-field');
                const trans = this.problem.transitions.find(tr => tr.id === tid);
                if (trans) {
                    trans[field] = Number(e.target.value);
                    this.solvePath();
                    this.requestRender();
                }
            });
        });

        tbody.querySelectorAll('.chkTransAvail').forEach(chk => {
            chk.addEventListener('change', (e) => {
                const tid = Number(e.target.getAttribute('data-id'));
                const trans = this.problem.transitions.find(tr => tr.id === tid);
                if (trans) {
                    trans.available = e.target.checked;
                    this.solvePath();
                    this.requestRender();
                }
            });
        });

        tbody.querySelectorAll('.btnDeleteTrans').forEach(btn => {
            btn.addEventListener('click', (e) => {
                const tid = Number(e.currentTarget.getAttribute('data-id'));
                this.problem.transitions = this.problem.transitions.filter(tr => tr.id !== tid);
                this.updateUI();
                this.openTransitionsModal();
                this.solvePath();
                this.requestRender();
            });
        });

        document.getElementById('btnModalAddTransition').onclick = () => {
            if (this.problem.states.length < 2) {
                alert('At least 2 states are required to create a transition.');
                return;
            }
            const nextId = this.problem.transitions.length > 0 ? Math.max(...this.problem.transitions.map(t => t.id)) + 1 : 101;
            this.problem.transitions.push({
                id: nextId,
                from: this.problem.states[0].id,
                to: this.problem.states[1].id,
                cost: 1.0,
                safety: 1.0,
                reliability: 1.0,
                available: true
            });
            this.updateUI();
            this.openTransitionsModal();
            this.solvePath();
            this.requestRender();
        };

        modal.classList.add('active');
    }
}

// Instantiate App when DOM is loaded
window.addEventListener('DOMContentLoaded', () => {
    window.app = new OrchestrationApp();
});
