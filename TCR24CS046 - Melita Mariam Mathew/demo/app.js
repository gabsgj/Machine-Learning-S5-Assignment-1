/**
 * Interactive Application Controller for Safe Semantic Planner Visualizer
 */

document.addEventListener('DOMContentLoaded', () => {
    const canvas = document.getElementById('plannerCanvas');
    const ctx = canvas.getContext('2d');

    const engine = new JSPlannerEngine();
    let currentResult = null;
    let currentScenarioKey = 'testcase1';
    let hoveredNode = null;
    let hoveredEdge = null;
    let selectedNode = null;
    let animationOffset = 0;

    // UI Elements
    const scenarioSelect = document.getElementById('scenarioSelect');
    const algoSelect = document.getElementById('algoSelect');
    const sliderAlpha = document.getElementById('sliderAlpha');
    const sliderBeta = document.getElementById('sliderBeta');
    const sliderGamma = document.getElementById('sliderGamma');
    const sliderDelta = document.getElementById('sliderDelta');
    const sliderSafetyRadius = document.getElementById('sliderSafetyRadius');

    const valAlpha = document.getElementById('valAlpha');
    const valBeta = document.getElementById('valBeta');
    const valGamma = document.getElementById('valGamma');
    const valDelta = document.getElementById('valDelta');
    const valSafetyRadius = document.getElementById('valSafetyRadius');

    const metricCost = document.getElementById('metricCost');
    const metricClearance = document.getElementById('metricClearance');
    const metricReliability = document.getElementById('metricReliability');
    const metricScore = document.getElementById('metricScore');
    const metricTime = document.getElementById('metricTime');
    const metricExplored = document.getElementById('metricExplored');
    const metricBadVisits = document.getElementById('metricBadVisits');
    const pathDisplay = document.getElementById('pathDisplay');
    const scenarioDesc = document.getElementById('scenarioDesc');
    const dynamicActionBtn = document.getElementById('dynamicActionBtn');
    const stateTableBody = document.getElementById('stateTableBody');

    // Auto-resize Canvas
    function resizeCanvas() {
        const rect = canvas.parentElement.getBoundingClientRect();
        canvas.width = rect.width * window.devicePixelRatio;
        canvas.height = rect.height * window.devicePixelRatio;
        ctx.scale(window.devicePixelRatio, window.devicePixelRatio);
    }
    window.addEventListener('resize', () => {
        resizeCanvas();
        draw();
    });
    resizeCanvas();

    // Load Scenario
    function loadScenario(key) {
        currentScenarioKey = key;
        let scenarioData;
        if (key === 'random_graph') {
            scenarioData = {
                title: "Large Complex Semantic Mesh",
                desc: "Dense 16-node Cartesian mesh with multi-objective safe corridors and dynamic obstacles.",
                problem: PRESET_SCENARIOS.random_graph.generate()
            };
        } else {
            scenarioData = PRESET_SCENARIOS[key];
        }

        scenarioDesc.innerHTML = `<strong>${scenarioData.title}</strong>: ${scenarioData.desc}`;
        engine.setProblem(scenarioData.problem);

        updateSlidersFromEngine();
        setupDynamicButton(key);
        runPlanner();
    }

    function setupDynamicButton(key) {
        if (key === 'testcase4') {
            dynamicActionBtn.style.display = 'block';
            dynamicActionBtn.textContent = '⚡ Toggle Blockage on Edge (A->G)';
            dynamicActionBtn.className = 'btn-primary btn-danger';
            dynamicActionBtn.onclick = () => {
                engine.toggleTransitionAvailability(2);
                runPlanner();
            };
        } else if (key === 'testcase5') {
            dynamicActionBtn.style.display = 'block';
            dynamicActionBtn.textContent = '🎯 Switch Goal to G2 (State 5)';
            dynamicActionBtn.className = 'btn-primary';
            dynamicActionBtn.onclick = () => {
                engine.setGoal(engine.goalState === 3 ? 5 : 3);
                runPlanner();
            };
        } else if (key === 'testcase6') {
            dynamicActionBtn.style.display = 'block';
            dynamicActionBtn.textContent = '🚀 Insert Shortcut Edge (A->G)';
            dynamicActionBtn.className = 'btn-primary';
            dynamicActionBtn.onclick = () => {
                // Add shortcut from A(1) to G(4)
                engine.addTransition(1, 4, 1.2, 0.99);
                runPlanner();
            };
        } else {
            dynamicActionBtn.style.display = 'none';
        }
    }

    function updateSlidersFromEngine() {
        engine.alpha = parseFloat(sliderAlpha.value);
        engine.beta = parseFloat(sliderBeta.value);
        engine.gamma = parseFloat(sliderGamma.value);
        engine.delta = parseFloat(sliderDelta.value);
        engine.safetyRadius = parseFloat(sliderSafetyRadius.value);

        valAlpha.textContent = engine.alpha.toFixed(0);
        valBeta.textContent = engine.beta.toFixed(1);
        valGamma.textContent = engine.gamma.toFixed(1);
        valDelta.textContent = engine.delta.toFixed(1);
        valSafetyRadius.textContent = engine.safetyRadius.toFixed(1);
    }

    function runPlanner() {
        updateSlidersFromEngine();
        currentResult = engine.computeShortestPath();
        updateTelemetry();
        updateStateTable();
        draw();
    }

    function updateTelemetry() {
        if (!currentResult || !currentResult.success) {
            metricCost.textContent = 'N/A';
            metricClearance.textContent = '0.00';
            metricReliability.textContent = '0.0%';
            metricScore.textContent = '0.00';
            metricTime.textContent = '0.00 ms';
            metricExplored.textContent = engine.exploredCount;
            metricBadVisits.textContent = '0';
            pathDisplay.innerHTML = '<span style="color: #f43f5e;">Unreachable / Blocked</span>';
            return;
        }

        metricCost.textContent = currentResult.totalCost.toFixed(2);
        metricClearance.textContent = currentResult.minSafetyDistance >= 50 ? 'Clear (>10.0)' : currentResult.minSafetyDistance.toFixed(2);
        metricReliability.textContent = (currentResult.cumulativeReliability * 100).toFixed(1) + '%';
        metricScore.textContent = currentResult.compositeScore.toFixed(2);
        metricTime.textContent = currentResult.planningTimeMs.toFixed(3) + ' ms';
        metricExplored.textContent = currentResult.exploredStates;
        metricBadVisits.textContent = currentResult.badStatesVisited;

        const pathNames = currentResult.statePath.map(id => {
            const s = engine.states.get(id);
            return s ? s.name : `S${id}`;
        });
        pathDisplay.innerHTML = pathNames.join(' <span style="color: #6366f1;">&rarr;</span> ');
    }

    function updateStateTable() {
        stateTableBody.innerHTML = '';
        const pathSet = new Set(currentResult && currentResult.success ? currentResult.statePath : []);

        for (const [id, s] of engine.states) {
            const tr = document.createElement('tr');
            if (pathSet.has(id)) tr.className = 'active-row';

            const gVal = engine.g.get(id);
            const rhsVal = engine.rhs.get(id);
            const key = engine.calculateKey(id);

            const gStr = gVal >= Infinity / 2 ? 'inf' : gVal.toFixed(2);
            const rhsStr = rhsVal >= Infinity / 2 ? 'inf' : rhsVal.toFixed(2);
            const k1Str = key.k1 >= Infinity / 2 ? 'inf' : key.k1.toFixed(2);
            const k2Str = key.k2 >= Infinity / 2 ? 'inf' : key.k2.toFixed(2);

            let status = 'Consistent';
            if (engine.badStates.has(id)) status = 'HAZARD [BAD]';
            else if (id === engine.initialState) status = 'Start (s_I)';
            else if (id === engine.goalState) status = 'Goal (s_G)';
            else if (Math.abs(gVal - rhsVal) > 1e-9) status = 'Inconsistent';

            tr.innerHTML = `
                <td>${s.name}</td>
                <td>${gStr}</td>
                <td>${rhsStr}</td>
                <td>[${k1Str}, ${k2Str}]</td>
                <td>${status}</td>
            `;
            stateTableBody.appendChild(tr);
        }
    }

    // Canvas Drawing
    function draw() {
        const w = canvas.parentElement.getBoundingClientRect().width;
        const h = canvas.parentElement.getBoundingClientRect().height;

        ctx.clearRect(0, 0, w, h);

        // Draw Grid Background
        ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
        ctx.lineWidth = 1;
        const gridSize = 40;
        for (let x = 0; x < w; x += gridSize) {
            ctx.beginPath();
            ctx.moveTo(x, 0);
            ctx.lineTo(x, h);
            ctx.stroke();
        }
        for (let y = 0; y < h; y += gridSize) {
            ctx.beginPath();
            ctx.moveTo(0, y);
            ctx.lineTo(w, y);
            ctx.stroke();
        }

        // 1. Draw Safety Halos around Bad States
        const scaleDist = 30.0; // pixel scale per Cartesian distance unit
        for (const bId of engine.badStates) {
            const b = engine.states.get(bId);
            if (!b) continue;

            const radius = engine.safetyRadius * scaleDist;
            const grad = ctx.createRadialGradient(b.x, b.y, 10, b.x, b.y, radius);
            grad.addColorStop(0, 'rgba(244, 63, 94, 0.35)');
            grad.addColorStop(0.6, 'rgba(244, 63, 94, 0.12)');
            grad.addColorStop(1, 'rgba(244, 63, 94, 0.0)');

            ctx.fillStyle = grad;
            ctx.beginPath();
            ctx.arc(b.x, b.y, radius, 0, Math.PI * 2);
            ctx.fill();

            ctx.strokeStyle = 'rgba(244, 63, 94, 0.4)';
            ctx.lineWidth = 1.5;
            ctx.setLineDash([4, 4]);
            ctx.beginPath();
            ctx.arc(b.x, b.y, radius, 0, Math.PI * 2);
            ctx.stroke();
            ctx.setLineDash([]);
        }

        // 2. Draw Transitions (Edges)
        const pathTransSet = new Set(currentResult && currentResult.success ? currentResult.transitionPath : []);

        for (const [tid, t] of engine.transitions) {
            const u = engine.states.get(t.from);
            const v = engine.states.get(t.to);
            if (!u || !v) continue;

            const inPath = pathTransSet.has(tid);
            const isHovered = (hoveredEdge === tid);

            ctx.beginPath();
            ctx.moveTo(u.x, u.y);
            ctx.lineTo(v.x, v.y);

            if (!t.available) {
                ctx.strokeStyle = 'rgba(244, 63, 94, 0.4)';
                ctx.lineWidth = 2;
                ctx.setLineDash([6, 6]);
                ctx.stroke();
                ctx.setLineDash([]);
                // Draw cross marker on disabled edge
                const midX = (u.x + v.x) / 2;
                const midY = (u.y + v.y) / 2;
                ctx.fillStyle = '#f43f5e';
                ctx.font = 'bold 12px JetBrains Mono';
                ctx.fillText('✕ BLOCKED', midX - 25, midY - 6);
            } else if (inPath) {
                // Glowing active path
                ctx.strokeStyle = '#38bdf8';
                ctx.lineWidth = 4;
                ctx.shadowColor = 'rgba(56, 189, 248, 0.8)';
                ctx.shadowBlur = 12;
                ctx.stroke();
                ctx.shadowBlur = 0; // reset
            } else {
                ctx.strokeStyle = isHovered ? 'rgba(99, 102, 241, 0.8)' : 'rgba(255, 255, 255, 0.15)';
                ctx.lineWidth = isHovered ? 2.5 : 1.5;
                ctx.stroke();
            }

            // Draw directional arrow
            drawArrow(ctx, u.x, u.y, v.x, v.y, inPath ? '#38bdf8' : (t.available ? 'rgba(255,255,255,0.4)' : '#f43f5e'), inPath ? 14 : 9);

            // Draw Edge Cost Label
            const midX = (u.x + v.x) / 2 + (v.y - u.y) * 0.08;
            const midY = (u.y + v.y) / 2 - (v.x - u.x) * 0.08;
            ctx.fillStyle = inPath ? '#38bdf8' : 'rgba(148, 163, 184, 0.7)';
            ctx.font = '10px JetBrains Mono';
            ctx.fillText(`c=${t.cost.toFixed(1)}`, midX, midY);
        }

        // 3. Draw States (Nodes)
        const pathStateSet = new Set(currentResult && currentResult.success ? currentResult.statePath : []);

        for (const [id, s] of engine.states) {
            const isStart = (id === engine.initialState);
            const isGoal = (id === engine.goalState);
            const isBad = engine.badStates.has(id);
            const inPath = pathStateSet.has(id);
            const isHovered = (hoveredNode === id);

            let nodeColor = '#334155';
            let glowColor = 'transparent';
            let ringColor = 'rgba(255,255,255,0.2)';
            let nodeRadius = 18;

            if (isBad) {
                nodeColor = '#e11d48';
                glowColor = 'rgba(244, 63, 94, 0.6)';
                ringColor = '#f43f5e';
            } else if (isStart) {
                nodeColor = '#059669';
                glowColor = 'rgba(16, 185, 129, 0.7)';
                ringColor = '#10b981';
                nodeRadius = 22;
            } else if (isGoal) {
                nodeColor = '#7c3aed';
                glowColor = 'rgba(168, 85, 247, 0.7)';
                ringColor = '#a855f7';
                nodeRadius = 22;
            } else if (inPath) {
                nodeColor = '#0284c7';
                glowColor = 'rgba(56, 189, 248, 0.8)';
                ringColor = '#38bdf8';
                nodeRadius = 20;
            }

            if (isHovered) {
                nodeRadius += 3;
            }

            // Outer Glow
            if (glowColor !== 'transparent') {
                ctx.shadowColor = glowColor;
                ctx.shadowBlur = 15;
            }

            // Node Circle
            ctx.fillStyle = nodeColor;
            ctx.beginPath();
            ctx.arc(s.x, s.y, nodeRadius, 0, Math.PI * 2);
            ctx.fill();
            ctx.shadowBlur = 0; // reset

            // Outer Ring
            ctx.strokeStyle = ringColor;
            ctx.lineWidth = inPath || isStart || isGoal ? 3 : 1.5;
            ctx.beginPath();
            ctx.arc(s.x, s.y, nodeRadius, 0, Math.PI * 2);
            ctx.stroke();

            // Node Label
            ctx.fillStyle = '#ffffff';
            ctx.font = 'bold 12px Outfit, sans-serif';
            ctx.textAlign = 'center';
            ctx.textBaseline = 'middle';
            ctx.fillText(s.name, s.x, s.y);

            // Cartesian Embedding Coordinates Subtitle
            ctx.fillStyle = 'rgba(255, 255, 255, 0.5)';
            ctx.font = '9px JetBrains Mono';
            ctx.fillText(`(${s.x.toFixed(0)},${s.y.toFixed(0)})`, s.x, s.y + nodeRadius + 12);
        }
    }

    function drawArrow(ctx, fromx, fromy, tox, toy, color, headlen = 10) {
        const dx = tox - fromx;
        const dy = toy - fromy;
        const angle = Math.atan2(dy, dx);
        const dist = Math.sqrt(dx * dx + dy * dy);
        
        // Offset arrow from target node radius
        const targetRadius = 22;
        const arrowX = tox - Math.cos(angle) * targetRadius;
        const arrowY = toy - Math.sin(angle) * targetRadius;

        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(arrowX, arrowY);
        ctx.lineTo(arrowX - headlen * Math.cos(angle - Math.PI / 6), arrowY - headlen * Math.sin(angle - Math.PI / 6));
        ctx.lineTo(arrowX - headlen * Math.cos(angle + Math.PI / 6), arrowY - headlen * Math.sin(angle + Math.PI / 6));
        ctx.closePath();
        ctx.fill();
    }

    // Canvas Interactions: Hover & Clicks
    canvas.addEventListener('mousemove', (e) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        let foundNode = null;
        for (const [id, s] of engine.states) {
            const dx = s.x - mouseX;
            const dy = s.y - mouseY;
            if (Math.sqrt(dx * dx + dy * dy) <= 24) {
                foundNode = id;
                break;
            }
        }

        hoveredNode = foundNode;
        canvas.style.cursor = hoveredNode !== null ? 'pointer' : 'crosshair';
        draw();
    });

    canvas.addEventListener('click', (e) => {
        const rect = canvas.getBoundingClientRect();
        const mouseX = e.clientX - rect.left;
        const mouseY = e.clientY - rect.top;

        // 1. Click Node -> Toggle Bad State
        for (const [id, s] of engine.states) {
            const dx = s.x - mouseX;
            const dy = s.y - mouseY;
            if (Math.sqrt(dx * dx + dy * dy) <= 24) {
                if (id !== engine.initialState && id !== engine.goalState) {
                    engine.toggleBadState(id);
                    runPlanner();
                    return;
                }
            }
        }

        // 2. Click Edge -> Toggle Availability
        for (const [tid, t] of engine.transitions) {
            const u = engine.states.get(t.from);
            const v = engine.states.get(t.to);
            if (!u || !v) continue;

            const distToSegment = pointToSegmentDistance(mouseX, mouseY, u.x, u.y, v.x, v.y);
            if (distToSegment < 10) {
                engine.toggleTransitionAvailability(tid);
                runPlanner();
                return;
            }
        }
    });

    function pointToSegmentDistance(px, py, x1, y1, x2, y2) {
        const l2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
        if (l2 === 0) return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
        let t = ((px - x1) * (x2 - x1) + (py - y1) * (y2 - y1)) / l2;
        t = Math.max(0, Math.min(1, t));
        return Math.sqrt((px - (x1 + t * (x2 - x1))) ** 2 + (py - (y1 + t * (y2 - y1))) ** 2);
    }

    // Sliders Event Handlers
    [sliderAlpha, sliderBeta, sliderGamma, sliderDelta, sliderSafetyRadius].forEach(slider => {
        slider.addEventListener('input', () => {
            runPlanner();
        });
    });

    scenarioSelect.addEventListener('change', (e) => {
        loadScenario(e.target.value);
    });

    if (algoSelect) {
        algoSelect.addEventListener('change', (e) => {
            engine.algoType = e.target.value;
            runPlanner();
        });
    }

    // Initialize Default Scenario (Test Case 1)
    loadScenario('testcase1');
});
