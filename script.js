/* =============================================
   SERP-X SIMULATION ENGINE
   Cave Rendering, Snake Robot, ML Integration
   ============================================= */

// ============ ML INSTANCES ============
const yolo = new YOLOv8Nano();
const rfClassifier = new RandomForestClassifier(50);
const rfRegressor = new RandomForestRegressor(100);
const astar = new AStarPathfinder();
const wedgeModel = new WedgePredictionModel();

// ============ STATE ============
const SIM = {
    running: false, paused: false, phase: 0,
    tick: 0, missionTime: 0, lastTime: 0, animFrame: null,
    // Robot (snake segments)
    snake: [], snakeLen: 14, robotX: 0, robotSpeed: 0, robotDepth: 0,
    // Battery
    battery: 100,
    // Cave data
    caveData: [], explored: 0, victimSeg: 150, victimFound: false,
    // Sonar
    sonarPulses: [],
    // Metrics
    metrics: { width: 0, height: 0, orientation: '--', airQuality: 0, wedgeRisk: 0, extraction: 0, slope: 0, co: 0, ch4: 0 },
    // ML results
    yoloResult: null, hazardResult: null, wedgeResult: null, extractResult: null,
    astarPath: [], astarExplored: [], astarGrid: null,
    // 3D map
    mapRotation: 0,
    // Humans (rescuers at entrance + forward teams)
    rescuers: [],
    forwardTeams: [],
    // Collapse debris
    debris: [],
    // YOLO bounding box flash
    yoloBboxFlash: 0,
    // Hurdle/obstacle state
    hurdleActive: false,
    hurdleType: null,
    hurdleSegment: -1,
    hurdlesPassed: [],
    speedBoost: 0,
    continueVisible: false,
    victimsRescued: 0,
};

// ============ CAVE GENERATION ============
function generateCave() {
    const cave = [];
    const segments = 400; // Extended for deeper exploration
    for (let i = 0; i < segments; i++) {
        const t = i / segments;
        let top, bottom, isGas = false, isWater = false, gasType = null, isDebris = false, isVictim = false;
        
        // Define cave geometry
        if (t < 0.06) {
            // Wide entrance with collapse rubble
            top = -70 - Math.sin(t * 12) * 10;
            bottom = 70 + Math.cos(t * 10) * 8;
        } else if (t < 0.125) {
            const s = (t - 0.06) / 0.065;
            top = -70 + s * 20 - Math.sin(t * 20) * 8;
            bottom = 70 - s * 15 + Math.cos(t * 16) * 6;
        } else if (t < 0.2) {
            // First tight passage
            const s = (t - 0.125) / 0.075;
            top = -50 + s * 15 - Math.sin(t * 28) * 6;
            bottom = 55 - s * 12 + Math.cos(t * 24) * 5;
        } else if (t < 0.25) {
            // Brief opening
            const s = (t - 0.2) / 0.05;
            top = -35 - Math.sin(s * Math.PI) * 18;
            bottom = 43 + Math.sin(s * Math.PI) * 12;
        } else if (t < 0.35) {
            // The Squeeze — birth canal (extremely narrow + steep decline)
            const s = (t - 0.25) / 0.1;
            top = -35 + s * 28 - Math.sin(t * 36) * 3;
            bottom = 43 - s * 30 + Math.cos(t * 32) * 2;
        } else if (t < 0.41) {
            // Victim pocket (around segment 150)
            const s = (t - 0.35) / 0.06;
            top = -7 - Math.sin(s * Math.PI) * 6;
            bottom = 13 + Math.sin(s * Math.PI) * 4;
            if (t > 0.36 && t < 0.38) isVictim = true;
        } else {
            // Post-rescue deep exploration zone
            const s = (t - 0.41) / 0.59;
            top = -10 - Math.sin(t * 50) * 15 + Math.cos(t * 15) * 10;
            bottom = 15 + Math.sin(t * 40) * 12 - Math.cos(t * 20) * 8;
            
            // Dense hazards and obstacles throughout exploration zone
            if (t > 0.42 && t < 0.44) {
                isDebris = true;
                top += 8; bottom -= 8; // Partial collapse
            } else if (t > 0.45 && t < 0.52) {
                isGas = true; gasType = 'co';
            } else if (t > 0.52 && t < 0.54) {
                isWater = true; bottom += 10; // Shallow pool
            } else if (t > 0.55 && t < 0.58) {
                isDebris = true;
                top += 14; bottom -= 14; // Major choke point
            } else if (t > 0.58 && t < 0.60) {
                isGas = true; gasType = 'ch4'; // Methane pocket
            } else if (t > 0.60 && t < 0.66) {
                isWater = true; bottom += 18; // Deep flooded chamber
            } else if (t > 0.67 && t < 0.68) {
                isVictim = true; // Second victim
            } else if (t > 0.69 && t < 0.72) {
                isDebris = true;
                top += 10; bottom -= 10; // Rockfall zone
            } else if (t > 0.72 && t < 0.74) {
                isGas = true; gasType = 'co';
                top -= 5; // CO seeping from above
            } else if (t > 0.75 && t < 0.82) {
                isGas = true; gasType = 'ch4';
                top -= 12; // Large methane chamber
            } else if (t > 0.82 && t < 0.84) {
                isVictim = true; // Third victim
            } else if (t > 0.84 && t < 0.87) {
                isWater = true; bottom += 12; // Underground stream
            } else if (t > 0.87 && t < 0.90) {
                isDebris = true;
                top += 12; bottom -= 12; // Severe constriction
            } else if (t > 0.90 && t < 0.92) {
                isGas = true; gasType = 'co';
            } else if (t > 0.92 && t < 0.96) {
                isWater = true; bottom += 22; // Terminal sump
            } else if (t > 0.96 && t < 0.97) {
                isVictim = true; // Fourth victim
            } else if (t > 0.97 && t < 0.99) {
                isDebris = true;
                top += 15; bottom -= 15; // Near-collapse
            }
        }

        const descent = t < 0.125 ? 0 : Math.sin((t - 0.125) * 5) * 65;
        let hazard = 0;
        if (t > 0.25 && t < 0.35) hazard = 0.65 + Math.random() * 0.3; // The Squeeze
        else if (t > 0.35 && t < 0.42) hazard = 0.85 + Math.random() * 0.1; // Victim area
        else if (t > 0.175 && t < 0.25) hazard = 0.2 + Math.random() * 0.2;
        else if (isGas || isWater || isDebris) hazard = 0.7 + Math.random() * 0.2; // New hazards

        const slope = t < 0.125 ? 5 : t < 0.25 ? 15 + (t - 0.125) * 160 : t < 0.35 ? 55 + (t - 0.25) * 80 : t < 0.5 ? 70 : 20 + Math.sin(t*20)*30;

        cave.push({ top, bottom, descent, hazard, width: bottom - top, explored: false, slope, isGas, gasType, isWater, isDebris, isVictim, victimFoundFlag: false });
    }
    return cave;
}

// Generate collapse debris
function generateDebris(cave) {
    const debris = [];
    // Rubble near entrance collapse
    for (let i = 0; i < 40; i++) {
        const seg = Math.floor(Math.random() * 25);
        const c = cave[seg];
        if (!c) continue;
        debris.push({
            seg, size: 2 + Math.random() * 8,
            yOff: c.top + Math.random() * (c.bottom - c.top) * 0.9,
            rot: Math.random() * Math.PI * 2,
            type: Math.random() > 0.6 ? 'rock' : 'rubble',
            color: `rgba(${80 + Math.random() * 40}, ${60 + Math.random() * 30}, ${40 + Math.random() * 20}, ${0.4 + Math.random() * 0.3})`
        });
    }
    // Debris in squeeze zone
    for (let i = 0; i < 20; i++) {
        const seg = 100 + Math.floor(Math.random() * 40);
        const c = cave[Math.min(seg, cave.length - 1)];
        debris.push({
            seg, size: 1 + Math.random() * 4,
            yOff: c.top + Math.random() * (c.bottom - c.top),
            rot: Math.random() * Math.PI * 2,
            type: 'rock',
            color: `rgba(${60 + Math.random() * 30}, ${45 + Math.random() * 20}, ${30 + Math.random() * 15}, ${0.3 + Math.random() * 0.3})`
        });
    }
    return debris;
}

// Generate rescuer positions at entrance + forward staging teams
function generateRescuers() {
    return [
        // Main entrance team
        { x: -10, baseY: 25, armPhase: 0, type: 'commander' },
        { x: -25, baseY: 30, armPhase: 1.2, type: 'medic' },
        { x: -40, baseY: 22, armPhase: 2.4, type: 'engineer' },
        { x: -55, baseY: 28, armPhase: 0.8, type: 'spotter' },
        { x: -70, baseY: 26, armPhase: 3.1, type: 'k9-handler' },
        { x: -85, baseY: 24, armPhase: 1.8, type: 'paramedic' },
        { x: -100, baseY: 32, armPhase: 0.4, type: 'logistics' },
        // Additional support team at entrance
        { x: -115, baseY: 26, armPhase: 2.0, type: 'medic' },
        { x: -130, baseY: 30, armPhase: 0.6, type: 'engineer' },
        { x: -145, baseY: 24, armPhase: 1.5, type: 'spotter' },
        { x: -160, baseY: 28, armPhase: 3.4, type: 'paramedic' },
        { x: -175, baseY: 22, armPhase: 0.9, type: 'logistics' },
    ];
}

// Generate forward-deployed teams deeper in the cave
function generateForwardTeams() {
    return [
        // Forward team Alpha — staged near first tight passage
        { seg: 35, yPos: 0.5, armPhase: 1.0, type: 'engineer', label: 'FWD-A' },
        { seg: 38, yPos: 0.4, armPhase: 2.3, type: 'medic', label: 'FWD-A' },
        { seg: 40, yPos: 0.6, armPhase: 0.7, type: 'spotter', label: 'FWD-A' },
        // Forward team Bravo — staged at opening before squeeze
        { seg: 78, yPos: 0.45, armPhase: 1.8, type: 'commander', label: 'FWD-B' },
        { seg: 80, yPos: 0.55, armPhase: 0.3, type: 'engineer', label: 'FWD-B' },
        { seg: 82, yPos: 0.5, armPhase: 2.9, type: 'paramedic', label: 'FWD-B' },
        { seg: 84, yPos: 0.4, armPhase: 1.1, type: 'k9-handler', label: 'FWD-B' },
    ];
}

// ============ CANVAS SETUP ============
let caveCanvas, caveCtx, mapCanvas, mapCtx, astarCanvas, astarCtx, gaugeCanvas, gaugeCtx, orientCanvas, orientCtx, wedgeGaugeCanvas, wedgeGaugeCtx;

function initCanvases() {
    caveCanvas = document.getElementById('cave-canvas');
    caveCtx = caveCanvas.getContext('2d');
    mapCanvas = document.getElementById('map-canvas');
    mapCtx = mapCanvas.getContext('2d');
    astarCanvas = document.getElementById('astar-canvas');
    astarCtx = astarCanvas.getContext('2d');
    gaugeCanvas = document.getElementById('gauge-canvas');
    gaugeCtx = gaugeCanvas.getContext('2d');
    orientCanvas = document.getElementById('orientation-canvas');
    orientCtx = orientCanvas.getContext('2d');
    wedgeGaugeCanvas = document.getElementById('wedge-gauge-canvas');
    wedgeGaugeCtx = wedgeGaugeCanvas.getContext('2d');
    resizeCanvases();
    window.addEventListener('resize', resizeCanvases);
}

function resizeCanvases() {
    const vp = document.querySelector('.cave-viewport');
    caveCanvas.width = vp.clientWidth;
    caveCanvas.height = vp.clientHeight;
    // Sub-panels
    [mapCanvas, astarCanvas].forEach(c => {
        const p = c.parentElement;
        c.width = p.clientWidth;
        c.height = p.clientHeight - 28;
    });
}

// ============ HELPER: Cave Y coordinates ============
function getCaveY(seg, H, centerY) {
    const c = SIM.caveData[Math.min(seg, SIM.caveData.length - 1)];
    if (!c) return { topY: 0, botY: H, midY: H / 2 };
    const topY = centerY + c.top * (H / 200) + c.descent * (H / 400);
    const botY = centerY + c.bottom * (H / 200) + c.descent * (H / 400);
    return { topY, botY, midY: (topY + botY) / 2 };
}

// ============ DRAW CAVE ============
function drawCave() {
    const W = caveCanvas.width, H = caveCanvas.height, ctx = caveCtx;
    const cave = SIM.caveData;
    if (!cave.length) return;

    ctx.fillStyle = '#050914';
    ctx.fillRect(0, 0, W, H);

    const centerY = H / 2;
    const scale = W / 80;
    const scrollX = Math.max(0, SIM.robotX - W * 0.3);

    // --- Rock walls with texture ---
    const rockGrad = ctx.createLinearGradient(0, 0, 0, H);
    rockGrad.addColorStop(0, '#1e150a');
    rockGrad.addColorStop(0.3, '#2a1d0f');
    rockGrad.addColorStop(0.5, '#221808');
    rockGrad.addColorStop(0.7, '#2a1d0f');
    rockGrad.addColorStop(1, '#1e150a');

    // Top wall
    ctx.fillStyle = rockGrad;
    ctx.beginPath();
    ctx.moveTo(0, 0);
    for (let px = 0; px < W; px++) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length) {
            const { topY } = getCaveY(seg, H, centerY);
            ctx.lineTo(px, topY);
        }
    }
    ctx.lineTo(W, 0);
    ctx.closePath();
    ctx.fill();

    // Bottom wall
    ctx.beginPath();
    ctx.moveTo(0, H);
    for (let px = 0; px < W; px++) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length) {
            const { botY } = getCaveY(seg, H, centerY);
            ctx.lineTo(px, botY);
        }
    }
    ctx.lineTo(W, H);
    ctx.closePath();
    ctx.fill();

    // Rock crack details
    ctx.save();
    ctx.strokeStyle = 'rgba(90, 65, 40, 0.25)';
    ctx.lineWidth = 0.5;
    for (let i = 0; i < 50; i++) {
        const sx = Math.random() * W;
        const sy = Math.random() * H;
        ctx.beginPath();
        ctx.moveTo(sx, sy);
        for (let j = 0; j < 3; j++) {
            ctx.lineTo(sx + (Math.random() - 0.5) * 30, sy + (Math.random() - 0.5) * 8);
        }
        ctx.stroke();
    }
    ctx.restore();

    // --- Collapse debris ---
    SIM.debris.forEach(d => {
        const dpx = d.seg * scale - scrollX;
        if (dpx < -20 || dpx > W + 20) return;
        const seg = Math.min(d.seg, cave.length - 1);
        const { topY, botY } = getCaveY(seg, H, centerY);
        const dy = topY + (botY - topY) * ((d.yOff - cave[seg].top) / (cave[seg].bottom - cave[seg].top));

        ctx.save();
        ctx.translate(dpx, dy);
        ctx.rotate(d.rot);
        ctx.fillStyle = d.color;
        if (d.type === 'rock') {
            ctx.beginPath();
            ctx.moveTo(-d.size, -d.size * 0.6);
            ctx.lineTo(d.size * 0.8, -d.size * 0.4);
            ctx.lineTo(d.size, d.size * 0.5);
            ctx.lineTo(-d.size * 0.3, d.size * 0.7);
            ctx.closePath();
            ctx.fill();
        } else {
            ctx.fillRect(-d.size / 2, -d.size / 3, d.size, d.size * 0.6);
        }
        ctx.restore();
    });

    // --- Hazard zones ---
    for (let px = 0; px < W; px += 3) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length && cave[seg].hazard > 0.5 && cave[seg].explored) {
            const { topY, botY } = getCaveY(seg, H, centerY);
            ctx.fillStyle = `rgba(255, 23, 68, ${cave[seg].hazard * 0.12})`;
            ctx.fillRect(px, topY, 4, botY - topY);
        }
    }

    // --- Fog of war ---
    if (SIM.running) {
        const rsx = SIM.robotX - scrollX;
        const fog = ctx.createLinearGradient(rsx + 80, 0, rsx + 220, 0);
        fog.addColorStop(0, 'rgba(5, 9, 20, 0)');
        fog.addColorStop(1, 'rgba(5, 9, 20, 0.88)');
        ctx.fillStyle = fog;
        ctx.fillRect(rsx + 80, 0, W, H);
    }

    // --- Water Hazards ---
    ctx.fillStyle = 'rgba(0, 191, 255, 0.3)';
    for (let px = 0; px < W; px++) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length && cave[seg].isWater && cave[seg].explored) {
            const { botY } = getCaveY(seg, H, centerY);
            ctx.fillRect(px, botY - 15 + Math.sin(SIM.tick * 0.05 + px * 0.1) * 2, 2, 15);
        }
    }

    // --- Gas Hazards (CO / CH4) ---
    for (let px = 0; px < W; px += 4) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length && cave[seg].isGas && cave[seg].explored) {
            const { topY, botY } = getCaveY(seg, H, centerY);
            const color = cave[seg].gasType === 'ch4' ? 'rgba(224, 64, 251, 0.15)' : 'rgba(138, 43, 226, 0.15)';
            ctx.fillStyle = color;
            ctx.beginPath();
            ctx.arc(px, topY + (botY - topY) * (0.2 + Math.random() * 0.6), 8 + Math.random() * 10, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    // --- Debris Hazards ---
    ctx.fillStyle = 'rgba(100, 80, 60, 0.9)';
    for (let px = 0; px < W; px += 8) {
        const seg = Math.floor((px + scrollX) / scale);
        if (seg >= 0 && seg < cave.length && cave[seg].isDebris && cave[seg].explored) {
            const { topY, botY } = getCaveY(seg, H, centerY);
            ctx.beginPath();
            ctx.moveTo(px, botY);
            ctx.lineTo(px + 4, botY - 6 - Math.random() * 8);
            ctx.lineTo(px + 8, botY);
            ctx.fill();
            
            ctx.beginPath();
            ctx.moveTo(px, topY);
            ctx.lineTo(px + 4, topY + 6 + Math.random() * 8);
            ctx.lineTo(px + 8, topY);
            ctx.fill();
        }
    }

    // --- Sonar pulses ---
    SIM.sonarPulses.forEach(p => {
        const px = p.x - scrollX;
        const alpha = 1 - p.age / p.maxAge;
        ctx.strokeStyle = `rgba(0, 229, 255, ${alpha * 0.25})`;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.arc(px, p.y, p.radius, 0, Math.PI * 2);
        ctx.stroke();
    });

    // --- LIDAR beams from robot ---
    if (SIM.running && !SIM.paused && SIM.snake.length > 0) {
        const head = SIM.snake[0];
        const hpx = head.x - scrollX;
        const seg = Math.floor(head.x / scale);
        const { midY } = getCaveY(seg, H, centerY);

        for (let b = 0; b < 10; b++) {
            const angle = (b / 10) * Math.PI * 2 + SIM.tick * 0.025;
            const len = 40 + Math.sin(SIM.tick * 0.04 + b) * 15;
            ctx.strokeStyle = `rgba(0, 229, 255, ${0.06 + Math.sin(SIM.tick * 0.03 + b) * 0.03})`;
            ctx.lineWidth = 0.5;
            ctx.beginPath();
            ctx.moveTo(hpx, midY);
            ctx.lineTo(hpx + Math.cos(angle) * len, midY + Math.sin(angle) * len);
            ctx.stroke();
        }
    }

    // --- A* path overlay on cave ---
    if (SIM.astarPath.length > 1) {
        ctx.strokeStyle = 'rgba(0, 230, 118, 0.35)';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        SIM.astarPath.forEach((p, i) => {
            const pathSeg = Math.floor((p.c / 60) * cave.length);
            const px = pathSeg * scale - scrollX;
            const { topY, botY } = getCaveY(pathSeg, H, centerY);
            const py = topY + (botY - topY) * (p.r / 40);
            if (i === 0) ctx.moveTo(px, py);
            else ctx.lineTo(px, py);
        });
        ctx.stroke();
        ctx.setLineDash([]);
    }

    // --- Rescuers at entrance ---
    if (scrollX < 200) {
        SIM.rescuers.forEach(r => {
            const rx = r.x * scale - scrollX;
            if (rx < -30 || rx > W + 30) return;
            const { topY, botY } = getCaveY(0, H, centerY);
            const ry = botY - 20;

            drawHuman(ctx, rx, ry, r.type, r.armPhase + SIM.tick * 0.01);
        });
    }

    // --- Forward-deployed teams inside cave ---
    if (SIM.forwardTeams) {
        SIM.forwardTeams.forEach(ft => {
            if (ft.seg > SIM.explored + 5) return; // Only show if explored
            const fpx = ft.seg * scale - scrollX;
            if (fpx < -30 || fpx > W + 30) return;
            const { topY, botY } = getCaveY(ft.seg, H, centerY);
            const fy = topY + (botY - topY) * ft.yPos;

            drawHuman(ctx, fpx, fy, ft.type, ft.armPhase + SIM.tick * 0.01);

            // Team label
            ctx.font = '6px "Share Tech Mono"';
            ctx.fillStyle = 'rgba(0, 229, 255, 0.35)';
            ctx.textAlign = 'center';
            ctx.fillText(ft.label, fpx, fy + 24);
            ctx.textAlign = 'left';
        });
    }

    // --- Victim (head-down inverted) ---
    if (SIM.explored > 135) {
        const vs = SIM.victimSeg;
        const vpx = vs * scale - scrollX;
        const { topY, botY, midY } = getCaveY(vs, H, centerY);

        // Thermal glow
        const glow = ctx.createRadialGradient(vpx, midY, 0, vpx, midY, 35);
        glow.addColorStop(0, 'rgba(255, 100, 0, 0.15)');
        glow.addColorStop(1, 'rgba(255, 60, 0, 0)');
        ctx.fillStyle = glow;
        ctx.fillRect(vpx - 35, midY - 35, 70, 70);

        // Draw victim human (inverted, stuck)
        ctx.save();
        ctx.translate(vpx, midY);
        ctx.rotate(Math.PI * 0.7); // Head-down ~126 degrees

        // Thermal aura
        ctx.fillStyle = 'rgba(255, 80, 0, 0.08)';
        ctx.beginPath();
        ctx.ellipse(0, 0, 16, 22, 0, 0, Math.PI * 2);
        ctx.fill();

        drawHumanBody(ctx, 0, 0, 'rgba(255, 145, 0, 0.85)', 0.9);
        ctx.restore();

        // YOLO bounding box overlay
        if (SIM.yoloBboxFlash > 0) {
            const bboxAlpha = Math.min(1, SIM.yoloBboxFlash);
            const bw = 36;
            const bh = 50;
            const conf = SIM.yoloResult ? (SIM.yoloResult.detections[0]?.confidence * 100).toFixed(1) : '0';
            ctx.font = '9px "Share Tech Mono"';
            ctx.fillStyle = `rgba(199, 125, 255, ${bboxAlpha})`;
            ctx.fillText(`person ${conf}%`, vpx - bw / 2, midY - bh / 2 - 4);

            // Corner brackets
            const cs = 6;
            ctx.strokeStyle = `rgba(199, 125, 255, ${bboxAlpha})`;
            ctx.lineWidth = 2;
            ctx.setLineDash([]);
            const x1 = vpx - bw / 2, y1 = midY - bh / 2, x2 = vpx + bw / 2, y2 = midY + bh / 2;
            [[x1, y1, 1, 1], [x2, y1, -1, 1], [x1, y2, 1, -1], [x2, y2, -1, -1]].forEach(([cx, cy, dx, dy]) => {
                ctx.beginPath();
                ctx.moveTo(cx, cy + cs * dy);
                ctx.lineTo(cx, cy);
                ctx.lineTo(cx + cs * dx, cy);
                ctx.stroke();
            });
        }

        // Label
        if (SIM.victimFound) {
            ctx.font = '8px "Share Tech Mono"';
            ctx.fillStyle = 'rgba(255, 145, 0, 0.7)';
            ctx.textAlign = 'center';
            ctx.fillText('VICTIM DETECTED', vpx, midY - 35);
            ctx.textAlign = 'left';
        }
    }

    // --- Snake Robot ---
    if (SIM.running && SIM.snake.length > 0) {
        drawSnakeRobot(ctx, scrollX, H, centerY, scale);
    }

    // --- Grid overlay ---
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.03)';
    ctx.lineWidth = 0.5;
    for (let gx = 0; gx < W; gx += 50) { ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, H); ctx.stroke(); }
    for (let gy = 0; gy < H; gy += 50) { ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(W, gy); ctx.stroke(); }

    // Depth scale
    ctx.font = '8px "Share Tech Mono"';
    ctx.fillStyle = 'rgba(0, 229, 255, 0.18)';
    ctx.textAlign = 'center';
    for (let dm = 0; dm < 200; dm += 10) {
        const dpx = dm * scale - scrollX;
        if (dpx > 0 && dpx < W) ctx.fillText(`${(dm * 0.5).toFixed(0)}m`, dpx, H - 6);
    }
}

// ============ DRAW HUMAN ============
function drawHumanBody(ctx, x, y, color, scl = 1) {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5 * scl;
    // Head
    ctx.beginPath();
    ctx.arc(x, y - 12 * scl, 4.5 * scl, 0, Math.PI * 2);
    ctx.stroke();
    // Torso
    ctx.beginPath();
    ctx.moveTo(x, y - 7.5 * scl);
    ctx.lineTo(x, y + 5 * scl);
    ctx.stroke();
    // Arms
    ctx.beginPath();
    ctx.moveTo(x - 6 * scl, y - 2 * scl);
    ctx.lineTo(x, y - 4 * scl);
    ctx.lineTo(x + 6 * scl, y - 3 * scl);
    ctx.stroke();
    // Legs
    ctx.beginPath();
    ctx.moveTo(x, y + 5 * scl);
    ctx.lineTo(x - 4.5 * scl, y + 15 * scl);
    ctx.moveTo(x, y + 5 * scl);
    ctx.lineTo(x + 4 * scl, y + 14 * scl);
    ctx.stroke();
}

function drawHuman(ctx, x, y, type, phase) {
    const colors = {
        commander: 'rgba(0, 229, 255, 0.7)',
        medic: 'rgba(255, 23, 68, 0.7)',
        engineer: 'rgba(255, 234, 0, 0.7)',
        spotter: 'rgba(0, 230, 118, 0.7)',
        'k9-handler': 'rgba(255, 145, 0, 0.7)',
        paramedic: 'rgba(224, 64, 251, 0.7)',
        logistics: 'rgba(100, 181, 246, 0.7)'
    };
    const color = colors[type] || 'rgba(200, 200, 200, 0.7)';

    ctx.save();
    ctx.translate(x, y);

    // Standing pose with slight arm movement
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    // Head
    ctx.beginPath();
    ctx.arc(0, -22, 5, 0, Math.PI * 2);
    ctx.stroke();
    // Helmet (for commander/engineer)
    if (type !== 'medic') {
        ctx.beginPath();
        ctx.arc(0, -23, 6.5, Math.PI, 0);
        ctx.stroke();
    }
    // Torso
    ctx.beginPath();
    ctx.moveTo(0, -17);
    ctx.lineTo(0, -2);
    ctx.stroke();
    // Arms (animated)
    const armSwing = Math.sin(phase) * 3;
    ctx.beginPath();
    ctx.moveTo(-8, -7 + armSwing);
    ctx.lineTo(0, -12);
    ctx.lineTo(8, -7 - armSwing);
    ctx.stroke();
    // Legs
    ctx.beginPath();
    ctx.moveTo(0, -2);
    ctx.lineTo(-5, 10);
    ctx.moveTo(0, -2);
    ctx.lineTo(5, 10);
    ctx.stroke();

    // Cross for medic
    if (type === 'medic') {
        ctx.strokeStyle = 'rgba(255, 23, 68, 0.5)';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.moveTo(-2, -12);
        ctx.lineTo(2, -12);
        ctx.moveTo(0, -14);
        ctx.lineTo(0, -10);
        ctx.stroke();
    }

    // Label
    ctx.font = '7px "Share Tech Mono"';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.fillText(type.toUpperCase(), 0, 18);

    ctx.restore();
}

// ============ DRAW SNAKE ROBOT ============
function drawSnakeRobot(ctx, scrollX, H, centerY, scale) {
    const snake = SIM.snake;
    if (snake.length < 2) return;

    // Draw trail
    ctx.strokeStyle = 'rgba(0, 229, 255, 0.06)';
    ctx.lineWidth = 8;
    ctx.lineCap = 'round';
    ctx.beginPath();
    snake.forEach((s, i) => {
        const sx = s.x - scrollX;
        const seg = Math.floor(s.x / scale);
        const { midY } = getCaveY(seg, H, centerY);
        const sy = midY + s.yOff;
        if (i === 0) ctx.moveTo(sx, sy);
        else ctx.lineTo(sx, sy);
    });
    ctx.stroke();

    // Draw body segments
    for (let i = snake.length - 1; i >= 0; i--) {
        const s = snake[i];
        const sx = s.x - scrollX;
        const seg = Math.floor(s.x / scale);
        const { midY } = getCaveY(seg, H, centerY);
        const sy = midY + s.yOff;

        const isHead = i === 0;
        const segSize = isHead ? 8 : 5 - (i / snake.length) * 2;
        const alpha = isHead ? 0.9 : 0.4 + (1 - i / snake.length) * 0.4;

        // Segment body
        ctx.fillStyle = `rgba(0, 229, 255, ${alpha * 0.2})`;
        ctx.strokeStyle = `rgba(0, 229, 255, ${alpha})`;
        ctx.lineWidth = 1.2;
        ctx.beginPath();
        ctx.roundRect(sx - segSize, sy - segSize * 0.6, segSize * 2, segSize * 1.2, isHead ? 4 : 2);
        ctx.fill();
        ctx.stroke();

        // Joints between segments
        if (i > 0) {
            const prev = snake[i - 1];
            const px = prev.x - scrollX;
            const pseg = Math.floor(prev.x / scale);
            const { midY: pmy } = getCaveY(pseg, H, centerY);
            const py = pmy + prev.yOff;
            ctx.strokeStyle = `rgba(0, 229, 255, ${alpha * 0.3})`;
            ctx.lineWidth = 2;
            ctx.beginPath();
            ctx.moveTo(sx, sy);
            ctx.lineTo(px, py);
            ctx.stroke();
        }

        // Head details
        if (isHead) {
            // Eye/sensor
            const eyeGlow = 0.5 + Math.sin(SIM.tick * 0.1) * 0.5;
            ctx.fillStyle = `rgba(0, 229, 255, ${eyeGlow})`;
            ctx.beginPath();
            ctx.arc(sx + 4, sy, 2.5, 0, Math.PI * 2);
            ctx.fill();

            // Antenna
            ctx.strokeStyle = 'rgba(0, 229, 255, 0.5)';
            ctx.lineWidth = 0.8;
            ctx.beginPath();
            ctx.moveTo(sx, sy - segSize * 0.6);
            ctx.lineTo(sx - 3, sy - segSize * 0.6 - 8);
            ctx.stroke();
            ctx.fillStyle = 'rgba(0, 229, 255, 0.7)';
            ctx.beginPath();
            ctx.arc(sx - 3, sy - segSize * 0.6 - 8, 1.5, 0, Math.PI * 2);
            ctx.fill();

            // Anti-gravity hover ring
            const hoverY = Math.sin(SIM.tick * 0.12) * 1.5;
            ctx.strokeStyle = 'rgba(0, 229, 255, 0.15)';
            ctx.setLineDash([2, 2]);
            ctx.beginPath();
            ctx.ellipse(sx, sy + segSize * 0.6 + 3 + hoverY, 10, 2.5, 0, 0, Math.PI * 2);
            ctx.stroke();
            ctx.setLineDash([]);

            // Label
            ctx.font = '8px "Share Tech Mono"';
            ctx.fillStyle = 'rgba(0, 229, 255, 0.6)';
            ctx.textAlign = 'center';
            ctx.fillText('SERP-X', sx, sy - segSize - 6);
            ctx.textAlign = 'left';
        }
    }
}

// ============ A* ROUTE VISUALIZATION ============
function drawAStarMap() {
    const W = astarCanvas.width, H = astarCanvas.height, ctx = astarCtx;
    ctx.fillStyle = '#030710';
    ctx.fillRect(0, 0, W, H);

    if (!SIM.astarGrid || SIM.astarGrid.length === 0) {
        ctx.font = '9px "Share Tech Mono"';
        ctx.fillStyle = 'rgba(0, 229, 255, 0.3)';
        ctx.textAlign = 'center';
        ctx.fillText('A* PATHFINDER — AWAITING GRID DATA', W / 2, H / 2);
        ctx.textAlign = 'left';
        return;
    }

    const cellW = W / 60;
    const cellH = H / 40;

    // Draw grid
    for (let r = 0; r < 40; r++) {
        for (let c = 0; c < 60; c++) {
            const cell = SIM.astarGrid[r]?.[c];
            if (!cell) continue;
            const x = c * cellW, y = r * cellH;

            if (!cell.walkable) {
                ctx.fillStyle = 'rgba(30, 21, 10, 0.8)';
                ctx.fillRect(x, y, cellW + 0.5, cellH + 0.5);
            } else if (cell.hazard > 0.7) {
                ctx.fillStyle = 'rgba(255, 23, 68, 0.12)';
                ctx.fillRect(x, y, cellW, cellH);
            } else if (cell.hazard > 0.3) {
                ctx.fillStyle = 'rgba(255, 145, 0, 0.06)';
                ctx.fillRect(x, y, cellW, cellH);
            }
        }
    }

    // Draw explored cells
    SIM.astarExplored.forEach(cell => {
        ctx.fillStyle = 'rgba(0, 229, 255, 0.04)';
        ctx.fillRect(cell.c * cellW, cell.r * cellH, cellW, cellH);
    });

    // Draw path
    if (SIM.astarPath.length > 1) {
        ctx.strokeStyle = 'rgba(0, 230, 118, 0.7)';
        ctx.lineWidth = 2;
        ctx.shadowColor = 'rgba(0, 230, 118, 0.3)';
        ctx.shadowBlur = 4;
        ctx.beginPath();
        SIM.astarPath.forEach((p, i) => {
            const x = p.c * cellW + cellW / 2;
            const y = p.r * cellH + cellH / 2;
            if (i === 0) ctx.moveTo(x, y);
            else ctx.lineTo(x, y);
        });
        ctx.stroke();
        ctx.shadowBlur = 0;

        // Start marker
        const sp = SIM.astarPath[0];
        ctx.fillStyle = 'rgba(0, 229, 255, 0.8)';
        ctx.beginPath();
        ctx.arc(sp.c * cellW + cellW / 2, sp.r * cellH + cellH / 2, 3, 0, Math.PI * 2);
        ctx.fill();

        // End marker
        const ep = SIM.astarPath[SIM.astarPath.length - 1];
        ctx.fillStyle = 'rgba(255, 145, 0, 0.8)';
        ctx.beginPath();
        ctx.arc(ep.c * cellW + cellW / 2, ep.r * cellH + cellH / 2, 3, 0, Math.PI * 2);
        ctx.fill();
    }

    ctx.font = '8px "Share Tech Mono"';
    ctx.fillStyle = 'rgba(0, 229, 255, 0.35)';
    ctx.textAlign = 'left';
    ctx.fillText(`NODES: ${SIM.astarExplored.length}`, 4, 10);
    ctx.fillText(`PATH: ${SIM.astarPath.length}`, 4, 20);
    ctx.textAlign = 'right';
    ctx.fillText('A* RESCUE ROUTE', W - 4, 10);
    ctx.textAlign = 'left';
}

// ============ 3D MAP ============
function draw3DMap() {
    const W = mapCanvas.width, H = mapCanvas.height, ctx = mapCtx;
    ctx.fillStyle = '#030710';
    ctx.fillRect(0, 0, W, H);
    const cave = SIM.caveData;
    if (!cave.length) return;

    const cx = W / 2, cy = H / 2 + 8;
    const rot = SIM.mapRotation;
    const maxSeg = Math.min(SIM.explored + 3, cave.length);

    for (let i = 0; i < maxSeg; i += 3) {
        const c = cave[i];
        const t = i / cave.length;
        const depth = i * 2;
        const px = cx + (depth - maxSeg) * Math.cos(rot) * 0.4;
        const py = cy + depth * 0.06 + c.descent * 0.15 - 15;
        const halfW = (c.bottom - c.top) * 0.2 * (0.5 + (1 - t) * 0.5);

        let color = c.hazard > 0.7 ? `rgba(255,23,68,${0.2 + c.hazard * 0.2})`
            : c.hazard > 0.3 ? `rgba(255,145,0,${0.15 + c.hazard * 0.2})`
                : `rgba(0,229,255,${0.1 + (1 - t) * 0.1})`;

        ctx.strokeStyle = color;
        ctx.lineWidth = 0.5;
        ctx.beginPath();
        ctx.ellipse(px, py, Math.max(1, halfW), Math.max(0.5, halfW * 0.4), Math.sin(rot) * 0.15, 0, Math.PI * 2);
        ctx.stroke();
    }

    // Robot on map
    if (SIM.running && SIM.snake.length > 0) {
        const seg = Math.floor(SIM.snake[0].x / (caveCanvas.width / 80));
        if (seg < maxSeg) {
            const c = cave[Math.min(seg, cave.length - 1)];
            const depth = seg * 2;
            const rpx = cx + (depth - maxSeg) * Math.cos(rot) * 0.4;
            const rpy = cy + depth * 0.06 + c.descent * 0.15 - 15;
            ctx.fillStyle = `rgba(0,229,255,${0.5 + Math.sin(SIM.tick * 0.06) * 0.3})`;
            ctx.beginPath();
            ctx.arc(rpx, rpy, 3, 0, Math.PI * 2);
            ctx.fill();
        }
    }

    ctx.font = '7px "Share Tech Mono"';
    ctx.fillStyle = 'rgba(0,229,255,0.3)';
    ctx.textAlign = 'right';
    ctx.fillText(`${maxSeg} SEGS`, W - 4, 10);
    ctx.textAlign = 'left';
}

// ============ GAUGES ============
function drawGauge(canvas, ctx, value, label) {
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0, 0, W, H);
    const cx = W / 2, cy = H - 8, radius = Math.min(W, H) * 0.58;

    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 7;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.arc(cx, cy, radius, Math.PI, 2 * Math.PI);
    ctx.stroke();

    const pct = Math.max(0, Math.min(100, value)) / 100;
    let color = pct < 0.3 ? '#ff1744' : pct < 0.6 ? '#ff9100' : '#00e676';

    const grad = ctx.createLinearGradient(cx - radius, cy, cx + radius, cy);
    grad.addColorStop(0, '#ff1744');
    grad.addColorStop(0.5, '#ff9100');
    grad.addColorStop(1, '#00e676');

    ctx.strokeStyle = grad;
    ctx.lineWidth = 7;
    ctx.beginPath();
    ctx.arc(cx, cy, radius, Math.PI, Math.PI + pct * Math.PI);
    ctx.stroke();

    ctx.font = '18px "Orbitron"';
    ctx.fillStyle = color;
    ctx.textAlign = 'center';
    ctx.fillText(`${Math.round(value)}%`, cx, cy - 10);
    ctx.font = '7px "Share Tech Mono"';
    ctx.fillStyle = 'rgba(255,255,255,0.25)';
    ctx.fillText(label || '', cx, cy + 4);
    ctx.textAlign = 'left';
}

function drawWedgeGauge(value) {
    const ctx = wedgeGaugeCtx, W = 140, H = 80;
    ctx.clearRect(0, 0, W, H);
    const cx = W / 2, cy = H - 6, r = 50;

    ctx.strokeStyle = 'rgba(255,255,255,0.05)';
    ctx.lineWidth = 6;
    ctx.lineCap = 'round';
    ctx.beginPath();
    ctx.arc(cx, cy, r, Math.PI, 2 * Math.PI);
    ctx.stroke();

    const pct = Math.max(0, Math.min(100, value)) / 100;
    // Inverse: high risk = red
    let color = pct > 0.7 ? '#ff1744' : pct > 0.4 ? '#ff9100' : '#00e676';

    ctx.strokeStyle = color;
    ctx.lineWidth = 6;
    ctx.shadowColor = color;
    ctx.shadowBlur = 8;
    ctx.beginPath();
    ctx.arc(cx, cy, r, Math.PI, Math.PI + pct * Math.PI);
    ctx.stroke();
    ctx.shadowBlur = 0;

    ctx.font = '7px "Share Tech Mono"';
    ctx.fillStyle = 'rgba(255,255,255,0.2)';
    ctx.textAlign = 'left';
    ctx.fillText('0', cx - r - 3, cy + 10);
    ctx.textAlign = 'right';
    ctx.fillText('100', cx + r + 3, cy + 10);
    ctx.textAlign = 'left';
}

function drawOrientation(orient) {
    const ctx = orientCtx, W = 80, H = 50;
    ctx.clearRect(0, 0, W, H);

    // Mini cave
    ctx.strokeStyle = 'rgba(224, 64, 251, 0.25)';
    ctx.lineWidth = 0.5;
    ctx.beginPath();
    ctx.moveTo(5, 12); ctx.lineTo(35, 8); ctx.lineTo(75, 10);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(5, 38); ctx.lineTo(35, 42); ctx.lineTo(75, 40);
    ctx.stroke();

    ctx.save();
    ctx.translate(52, 25);
    let angle = orient === 'HEAD-DOWN INVERTED' ? Math.PI * 0.7 : orient === 'PRONE' ? 0 : Math.PI / 2;
    ctx.rotate(angle);
    drawHumanBody(ctx, 0, 0, 'rgba(255, 145, 0, 0.75)', 0.6);
    ctx.restore();
}

// ============ SNAKE ROBOT PHYSICS ============
function updateSnake(dt) {
    const cave = SIM.caveData;
    const scale = caveCanvas.width / 80;
    const maxX = (cave.length - 12) * scale;

    // Head movement
    const seg = Math.floor(SIM.robotX / scale);
    const c = cave[Math.min(seg, cave.length - 1)];
    const passageWidth = c ? c.width : 100;

    let targetSpeed;
    if (passageWidth > 80) targetSpeed = 2.8;
    else if (passageWidth > 50) targetSpeed = 2.0;
    else if (passageWidth > 30) targetSpeed = 1.0;
    else targetSpeed = 0.35;

    const progress = SIM.robotX / maxX;
    
    // Stop completely only if the user has triggered a pause via the pause button
    if (SIM.paused) {
        targetSpeed = 0;
    } else if (SIM.hurdleActive) {
        // Hurdle encountered — robot stops until user clicks CONTINUE
        targetSpeed = 0;
    } else {
        // Auto-advance phases based on progress (no pausing)
        if (SIM.phase < 7 && progress > 0.43) {
            advanceToPhase(7);
        }

        // Check for new hurdles (only in deep exploration, phase >= 9)
        if (SIM.phase >= 9 && c && !SIM.hurdleActive) {
            const hurdleKey = `seg-${seg}`;
            if (!SIM.hurdlesPassed.includes(hurdleKey)) {
                if (c.isDebris && seg > 160) {
                    triggerHurdle('DEBRIS', seg, hurdleKey);
                    targetSpeed = 0;
                } else if (c.isWater && seg > 200) {
                    triggerHurdle('WATER', seg, hurdleKey);
                    targetSpeed = 0;
                } else if (c.isGas && seg > 170) {
                    triggerHurdle('GAS', seg, hurdleKey);
                    targetSpeed = 0;
                }
            }
        }

        // Speed boost from CONTINUE button
        if (SIM.speedBoost > 0) {
            targetSpeed *= 2.5;
            SIM.speedBoost -= dt;
        }

        // Slow down in hazardous zones but never fully stop (unless hurdle triggered)
        if (c && c.isWater && !SIM.speedBoost) targetSpeed *= 0.3;
        if (c && c.isDebris && !SIM.speedBoost) targetSpeed *= 0.2;
        if (c && c.isGas && !SIM.speedBoost) targetSpeed *= 0.6;
    }

    SIM.robotSpeed += (targetSpeed - SIM.robotSpeed) * 0.025;
    SIM.robotX += SIM.robotSpeed * dt * 45;
    SIM.robotX = Math.min(SIM.robotX, maxX);
    SIM.robotDepth = SIM.robotX / scale * 0.5;

    // Snake undulation
    const headYOff = Math.sin(SIM.tick * 0.06) * 3 + Math.sin(SIM.tick * 0.02) * 2;

    // Update snake segments
    if (SIM.snake.length === 0) {
        for (let i = 0; i < SIM.snakeLen; i++) {
            SIM.snake.push({ x: SIM.robotX - i * 8, yOff: 0 });
        }
    }

    // Head follows robot position
    SIM.snake[0].x = SIM.robotX;
    SIM.snake[0].yOff = headYOff;

    // Body follows head with delay
    for (let i = 1; i < SIM.snake.length; i++) {
        const prev = SIM.snake[i - 1];
        const curr = SIM.snake[i];
        const dx = prev.x - curr.x;
        const dy = prev.yOff - curr.yOff;
        const dist = Math.sqrt(dx * dx + dy * dy);
        const spacing = 8;

        if (dist > spacing) {
            const ratio = (dist - spacing) / dist;
            curr.x += dx * ratio * 0.4;
            curr.yOff += dy * ratio * 0.4;
        }

        // Undulation
        curr.yOff += Math.sin(SIM.tick * 0.06 + i * 0.6) * 0.15;
    }

    // Mark explored
    const currentSeg = Math.floor(SIM.robotX / scale);
    for (let i = 0; i <= currentSeg && i < cave.length; i++) cave[i].explored = true;
    SIM.explored = currentSeg;

    SIM.battery = Math.max(0, 100 - SIM.missionTime * 0.25);
}

// ============ HURDLE SYSTEM ============
function triggerHurdle(type, seg, key) {
    SIM.hurdleActive = true;
    SIM.hurdleType = type;
    SIM.hurdleSegment = seg;
    SIM.hurdlesPassed.push(key);
    
    // Mark a range of segments as "passed" so we don't re-trigger
    for (let i = -5; i <= 15; i++) {
        SIM.hurdlesPassed.push(`seg-${seg + i}`);
    }
    
    // Show CONTINUE button
    showContinueButton();
    
    // Log the hurdle
    const hurdleMessages = {
        'DEBRIS': '⚠ ROCKFALL DEBRIS BLOCKING PATH — Manual override required to proceed.',
        'WATER': '⚠ FLOODED PASSAGE DETECTED — Submersion protocol required. Press CONTINUE.',
        'GAS': '⚠ TOXIC GAS POCKET — CO/CH₄ levels dangerous. Ventilation bypass needed.'
    };
    addLog('alert', 'WARN', hurdleMessages[type] || '⚠ OBSTACLE ENCOUNTERED');
    
    // Update AI recommendation
    const aiMessages = {
        'DEBRIS': { text: '⛏️ CLEAR DEBRIS', type: 'warning', conf: 78, reason: 'Rockfall detected ahead. SERP-X articulated body can navigate through gaps. Press CONTINUE to engage debris-clearing mode.' },
        'WATER': { text: '🌊 SUBMERSION MODE', type: 'warning', conf: 72, reason: 'Flooded passage ahead. SERP-X is waterproof rated IP68. Press CONTINUE to engage amphibious traversal.' },
        'GAS': { text: '💨 GAS BYPASS', type: 'danger', conf: 65, reason: 'Toxic gas pocket detected. MQ sensors reading high. Press CONTINUE to engage sealed traversal with active filtration.' },
    };
    const msg = aiMessages[type];
    if (msg) updateAI(msg.text, msg.type, msg.conf, msg.reason);
    
    document.getElementById('status-message').textContent = `⚠ HURDLE: ${type} — PRESS CONTINUE TO PROCEED`;
    flashPanel('cave-panel');
}

function showContinueButton() {
    const btn = document.getElementById('btn-continue');
    btn.style.display = 'inline-block';
    SIM.continueVisible = true;
    // Pulse animation on the button
    btn.style.animation = 'none';
    btn.offsetHeight; // trigger reflow
    btn.style.animation = 'continue-pulse 1.2s ease-in-out infinite';
}

function hideContinueButton() {
    const btn = document.getElementById('btn-continue');
    btn.style.display = 'none';
    btn.style.animation = 'none';
    SIM.continueVisible = false;
}

// ============ ML INTEGRATION ============
function runYOLO() {
    const scale = caveCanvas.width / 80;
    const seg = Math.floor(SIM.robotX / scale);
    const cave = SIM.caveData;
    
    // Find closest victim segment
    let closestVictimDist = 1000;
    for (let i = 0; i < cave.length; i++) {
        if (cave[i].isVictim) {
            let dist = Math.abs(seg - i) * 0.5;
            if (dist < closestVictimDist) closestVictimDist = dist;
        }
    }

    const result = yolo.detect({
        thermalSignature: closestVictimDist < 15 ? 0.9 - closestVictimDist * 0.02 : 0.1,
        distance: closestVictimDist,
        passageWidth: cave[Math.min(seg, cave.length - 1)]?.width || 50,
        victimPresent: closestVictimDist < 15,
        victimX: 100,
        victimY: 60
    });

    SIM.yoloResult = result;

    const best = yolo.getBestPersonDetection();
    if (best) {
        document.getElementById('yolo-class').textContent = 'PERSON';
        const confPct = (best.confidence * 100).toFixed(1);
        document.getElementById('yolo-confidence').textContent = `${confPct}%`;
        document.getElementById('yolo-conf-fill').style.width = `${confPct}%`;
        document.getElementById('yolo-bbox').textContent = `[${Math.round(best.bbox.x)},${Math.round(best.bbox.y)},${Math.round(best.bbox.w)},${Math.round(best.bbox.h)}]`;
        document.getElementById('yolo-ml-status').textContent = 'DETECTED';
        document.getElementById('yolo-ml-status').className = 'ml-status complete';
        document.getElementById('yolo-status').textContent = `PERSON ${confPct}%`;
        SIM.yoloBboxFlash = 3;
    } else {
        document.getElementById('yolo-class').textContent = 'SCANNING';
        document.getElementById('yolo-confidence').textContent = '--%';
        document.getElementById('yolo-conf-fill').style.width = '0%';
        document.getElementById('yolo-bbox').textContent = '--';
        document.getElementById('yolo-ml-status').textContent = 'SCANNING';
        document.getElementById('yolo-ml-status').className = 'ml-status processing';
        document.getElementById('yolo-status').textContent = 'SCANNING...';
    }

    document.getElementById('yolo-inference').textContent = `${result.inferenceTime.toFixed(1)} ms`;
}

function runHazardDetection() {
    const m = SIM.metrics;
    const result = rfClassifier.predict({
        width: m.width,
        height: m.height,
        slope: m.slope,
        airQuality: m.airQuality,
        rockStability: Math.max(0.1, 1 - m.slope / 90),
        waterLevel: 0.05 + Math.random() * 0.1
    });

    SIM.hazardResult = result;

    const levelEl = document.getElementById('hazard-level');
    levelEl.textContent = result.predicted;
    levelEl.className = `hazard-level ${result.predicted.toLowerCase()}`;

    const p = result.probabilities;
    document.getElementById('hp-safe').style.width = `${p.safe * 100}%`;
    document.getElementById('hp-caution').style.width = `${p.caution * 100}%`;
    document.getElementById('hp-danger').style.width = `${p.danger * 100}%`;
    document.getElementById('hp-safe-val').textContent = `${(p.safe * 100).toFixed(1)}%`;
    document.getElementById('hp-caution-val').textContent = `${(p.caution * 100).toFixed(1)}%`;
    document.getElementById('hp-danger-val').textContent = `${(p.danger * 100).toFixed(1)}%`;

    document.getElementById('hazard-ml-status').textContent = 'ACTIVE';
    document.getElementById('hazard-ml-status').className = 'ml-status active';

    // Update tree visualization
    const treesEl = document.getElementById('rf-trees');
    if (treesEl.children.length === 0) {
        treesEl.innerHTML = '';
        result.treeVotes.forEach(vote => {
            const dot = document.createElement('div');
            dot.className = `rf-tree-dot ${vote}`;
            treesEl.appendChild(dot);
        });
    } else {
        result.treeVotes.forEach((vote, i) => {
            if (treesEl.children[i]) treesEl.children[i].className = `rf-tree-dot ${vote}`;
        });
    }
}

function runWedgePrediction() {
    const m = SIM.metrics;
    const result = wedgeModel.predict({
        tunnelWidth: m.width,
        tunnelHeight: m.height,
        slope: m.slope
    });

    SIM.wedgeResult = result;

    document.getElementById('wp-twidth').textContent = `${m.width.toFixed(1)} cm`;
    document.getElementById('wp-theight').textContent = `${m.height.toFixed(1)} cm`;
    document.getElementById('wp-slope').textContent = `${m.slope.toFixed(1)}°`;
    document.getElementById('wedge-score').textContent = result.score;

    const scoreEl = document.getElementById('wedge-score');
    if (result.score > 70) scoreEl.style.color = 'var(--accent-red)';
    else if (result.score > 40) scoreEl.style.color = 'var(--accent-orange)';
    else scoreEl.style.color = 'var(--accent-green)';

    drawWedgeGauge(result.score);

    document.getElementById('wedge-ml-status').textContent = 'ACTIVE';
    document.getElementById('wedge-ml-status').className = 'ml-status active';
}

function runExtractionFeasibility() {
    const m = SIM.metrics;
    const result = rfRegressor.predict({
        width: m.width,
        height: m.height,
        wedgeRisk: SIM.wedgeResult ? SIM.wedgeResult.score : 50,
        slope: m.slope
    });

    SIM.extractResult = result;
    SIM.metrics.extraction = result.prediction;

    document.getElementById('val-extraction').textContent = `${result.prediction.toFixed(1)}%`;
    document.getElementById('val-est-time').textContent = `${(12 - result.prediction * 0.1).toFixed(1)} hrs`;

    drawGauge(gaugeCanvas, gaugeCtx, result.prediction, 'FEASIBILITY');

    document.getElementById('extract-ml-status').textContent = 'ACTIVE';
    document.getElementById('extract-ml-status').className = 'ml-status active';
}

function runAStar() {
    const cave = SIM.caveData;
    SIM.astarGrid = astar.buildGrid(cave, 40, 60);

    // Find entrance center
    let startR = 20; // Middle row
    for (let r = 0; r < 40; r++) {
        if (SIM.astarGrid[r][2]?.walkable) { startR = r; break; }
    }

    // Find victim column/row
    const victimCol = Math.floor((SIM.victimSeg / cave.length) * 60);
    let endR = 20;
    for (let r = 0; r < 40; r++) {
        if (SIM.astarGrid[r][victimCol]?.walkable) { endR = r; break; }
    }

    const path = astar.findPath(startR, 2, endR, victimCol);
    SIM.astarPath = path;
    SIM.astarExplored = astar.explored;

    addLog('ml', 'A*', `Pathfinding complete. ${path.length} waypoints, ${astar.explored.length} nodes explored.`);
}

// ============ METRICS UPDATE ============
function updateMetrics() {
    const cave = SIM.caveData;
    const scale = caveCanvas.width / 80;
    const seg = Math.floor(SIM.robotX / scale);
    const c = cave[Math.min(seg, cave.length - 1)];
    if (!c || !SIM.running) return;

    const width = c.width * 1.2;
    const height = width * 0.65 + Math.sin(seg * 0.08) * 4;
    SIM.metrics.width = Math.max(10, width);
    SIM.metrics.height = Math.max(8, height);
    SIM.metrics.slope = c.slope;

    // Orientation metric based on if near a victim
    const activeV = cave.findIndex((c, i) => c.isVictim && Math.abs(i - seg) < 15);
    if (activeV >= 0) SIM.metrics.orientation = 'HEAD-DOWN INVERTED';
    else if (SIM.explored > 50) SIM.metrics.orientation = 'SCANNING...';

    const t = seg / cave.length;
    
    // Gas/Air logic
    let baseAir = Math.max(11, 20.9 - t * 9 + Math.sin(SIM.tick * 0.008) * 0.2);
    let targetCo = 0;
    let targetCh4 = 0;

    if (c.isGas) {
        baseAir -= 5; // Drops heavily in gas
        if (c.gasType === 'co') {
            targetCo = 150 + Math.random() * 50;
        } else if (c.gasType === 'ch4') {
            targetCh4 = 15 + Math.random() * 10;
        }
    }

    SIM.metrics.airQuality += (baseAir - SIM.metrics.airQuality) * 0.1;
    SIM.metrics.co += (targetCo - SIM.metrics.co) * 0.1;
    SIM.metrics.ch4 += (targetCh4 - SIM.metrics.ch4) * 0.1;
}

function renderMetrics() {
    const m = SIM.metrics;
    document.getElementById('val-width').innerHTML = m.width > 0 ? `${m.width.toFixed(1)}<small>cm</small>` : '--';
    document.getElementById('bar-width').style.width = `${Math.min(100, m.width / 1.4)}%`;
    document.getElementById('val-height').innerHTML = m.height > 0 ? `${m.height.toFixed(1)}<small>cm</small>` : '--';
    document.getElementById('bar-height').style.width = `${Math.min(100, m.height / 1.0)}%`;
    document.getElementById('val-orientation').textContent = m.orientation;
    document.getElementById('val-air').innerHTML = m.airQuality > 0 ? `${m.airQuality.toFixed(1)}<small>%</small>` : '--';
    document.getElementById('bar-air').style.width = `${(m.airQuality / 21) * 100}%`;

    document.getElementById('val-co').innerHTML = m.co > 0 ? `${Math.round(m.co)}<small>ppm</small>` : '--';
    document.getElementById('bar-co').style.width = `${Math.min(100, m.co / 2.5)}%`;

    document.getElementById('val-ch4').innerHTML = m.ch4 > 0 ? `${m.ch4.toFixed(1)}<small>LEL%</small>` : '--';
    document.getElementById('bar-ch4').style.width = `${Math.min(100, m.ch4 / 0.5)}%`;

    // Card alerts
    const wc = document.getElementById('metric-width');
    const ac = document.getElementById('metric-air');
    const coc = document.getElementById('metric-co');
    const ch4c = document.getElementById('metric-ch4');
    
    wc.classList.toggle('alert', m.width > 0 && m.width < 30);
    wc.classList.toggle('warning', m.width >= 30 && m.width < 50);
    ac.classList.toggle('alert', m.airQuality > 0 && m.airQuality < 15);
    ac.classList.toggle('warning', m.airQuality >= 15 && m.airQuality < 18);
    
    coc.classList.toggle('alert', m.co > 100);
    coc.classList.toggle('warning', m.co >= 35 && m.co <= 100);
    
    ch4c.classList.toggle('alert', m.ch4 > 10);
    ch4c.classList.toggle('warning', m.ch4 >= 5 && m.ch4 <= 10);

    drawOrientation(m.orientation);

    document.getElementById('robot-depth').textContent = `DEPTH: ${SIM.robotDepth.toFixed(1)}m`;
    document.getElementById('robot-speed').textContent = `SPEED: ${SIM.robotSpeed.toFixed(2)} m/s`;

    const battPct = Math.round(SIM.battery);
    document.getElementById('sb-battery').innerHTML = `<span class="sb-dot ${battPct > 30 ? 'sb-green' : battPct > 15 ? 'sb-yellow' : 'sb-red'}"></span> BATTERY: ${battPct}%`;
}

// ============ PHASE MANAGEMENT ============
const phaseInfo = {
    1: { status: 'DEPLOYING', log: ['SYS', 'SERP-X snake-bot deploying. Anti-gravity drive engaged. 14 segments online.'], msg: 'DEPLOYING SERP-X SNAKE ROBOT' },
    2: { status: 'EXPLORING', log: ['BOT', 'Cave system entered. LIDAR + thermal scanning active. Mapping debris field.'], msg: 'EXPLORING — MAPPING COLLAPSE ZONE' },
    3: { status: 'DETECTING', log: ['ML', 'YOLOv8 Nano inference pipeline active. Scanning for human thermal signatures.'], msg: 'YOLOv8n VICTIM DETECTION ACTIVE' },
    4: { status: 'MAPPING', log: ['ML', '3D tunnel map generation from LIDAR point cloud. Processing geometry.'], msg: '3D TUNNEL MAP GENERATION' },
    5: { status: 'RF HAZARD', log: ['ML', 'Random Forest classifier (50 trees) — hazard analysis executing.'], msg: 'RANDOM FOREST HAZARD CLASSIFICATION' },
    6: { status: 'KNULL ZONE', log: ['ML', 'Knull Zone prediction: tunnel_w, tunnel_h, slope, shoulder_w → risk score.'], msg: 'KNULL ZONE RISK PREDICTION MODEL' },
    7: { status: 'A* PATH', log: ['ML', 'A* pathfinding on 40×60 grid. Finding optimal rescue route.'], msg: 'A* PATHFINDING — COMPUTING RESCUE ROUTE' },
    8: { status: 'EXTRACTION', log: ['ML', 'RF Regressor (100 trees) — extraction feasibility computed.'], msg: 'RF REGRESSOR EXTRACTION ANALYSIS' },
    9: { status: 'EXPLORING', log: ['SYS', 'Resuming exploration. MQ Gas Sensors active. Proceeding deeper.'], msg: 'DEEP EXPLORATION — MQ SENSORS ACTIVE' }
};

function advanceToPhase(n) {
    if (n <= SIM.phase) return;
    for (let i = 1; i < n; i++) {
        document.getElementById(`phase-${i}`).classList.remove('active');
        document.getElementById(`phase-${i}`).classList.add('complete');
        document.getElementById(`phase-${i}-status`).textContent = 'DONE';
    }
    document.getElementById(`phase-${n}`).classList.add('active');
    document.getElementById(`phase-${n}-status`).textContent = 'ACTIVE';
    SIM.phase = n;

    const info = phaseInfo[n];
    if (info) {
        const tagMap = { SYS: 'system', BOT: 'robot', ML: 'ml', AI: 'ai' };
        addLog(tagMap[info.log[0]] || 'system', info.log[0], info.log[1]);
        document.getElementById('status-message').textContent = info.msg;
        const sd = document.querySelector('.status-dot');
        const st = document.querySelector('.status-text');
        st.textContent = info.status;
        if (n >= 5) { sd.className = 'status-dot alert'; st.style.color = 'var(--accent-red)'; }
        else { sd.className = 'status-dot active'; st.style.color = 'var(--accent-green)'; }
    }

    // Phase-specific actions
    if (n === 3) {
        document.getElementById('yolo-overlay').style.display = 'flex';
        updateAI('PROCEED WITH CAUTION', 'warning', 72, 'YOLOv8 Nano scanning for human thermal signatures. Passage narrowing detected ahead.');
    }
    if (n === 5) {
        updateAI('⛔ DO NOT PULL', 'danger', 94, 'CRITICAL: Victim head-down inverted at ~70° decline. Random Forest hazard classifier: DANGER. Body wedge geometry creates one-way valve effect. Direct pull causes fatal compression.');
        addLog('alert', 'WARN', '⚠ DIRECT EXTRACTION IS NOT VIABLE — FATAL RISK');
    }
    if (n === 6) {
        updateAI('EXPAND PASSAGE', 'warning', 87, 'Knull Zone Algorithm: shoulder width (45.7cm) exceeds passage clearance. Risk score critical. Recommend hydraulic expansion: +12cm lateral, +8cm vertical.');
    }
    if (n === 7) {
        runAStar();
        updateAI('A* ROUTE COMPUTED', 'warning', 82, `A* pathfinding found ${SIM.astarPath.length}-waypoint rescue route avoiding hazard zones. Estimated extraction path length: ${(SIM.astarPath.length * 0.5).toFixed(1)}m.`);
        setTimeout(() => advanceToPhase(8), 2500);
    }
    if (n === 8) {
        const recEl = document.getElementById('ai-recommendation');
        recEl.className = 'ai-recommendation rec-success';
        document.getElementById('ai-rec-value').textContent = 'CREATE SECONDARY ACCESS';
        document.getElementById('ai-confidence').textContent = 'CONFIDENCE: 91%';
        document.getElementById('ai-rec-reason').textContent = `PRIMARY: Secondary access tunnel from adjacent chamber (4.2m). SECONDARY: Hydraulic expansion + pulley system. Extraction probability: ${SIM.metrics.extraction.toFixed(1)}%. Time: 6-12 hours.`;

        generateStrategies();
        addLog('ai', 'AI', '✓ All ML analyses complete. Rescue strategies generated.');
        // Show CONTINUE button for user to proceed to deep exploration
        showContinueButton();
        addLog('system', 'SYS', '▶ Press CONTINUE to begin deep exploration phase.');
    }
}

function continueExploration() {
    document.getElementById('btn-pause').disabled = false;
    SIM.paused = false;
    
    // Clear hurdle state
    if (SIM.hurdleActive) {
        const type = SIM.hurdleType;
        SIM.hurdleActive = false;
        SIM.hurdleType = null;
        SIM.speedBoost = 3; // 3 seconds of boosted speed to push through
        
        addLog('robot', 'BOT', `✓ ${type} obstacle cleared. Speed boost engaged — pushing through.`);
        updateAI('CONTINUING EXPLORATION', 'success', 88, `${type} obstacle cleared. SERP-X proceeding with boosted drive. ML sensors re-engaging.`);
        document.getElementById('status-message').textContent = `HURDLE CLEARED — CONTINUING DEEP EXPLORATION`;
        
        // Keep the button visible but don't hide — it stays for future hurdles
        hideContinueButton();
    } else {
        // First-time continue from initial analysis
        hideContinueButton();
    }
    
    // Reset ML statuses for new exploration
    ['yolo-ml-status', 'hazard-ml-status', 'wedge-ml-status', 'extract-ml-status'].forEach(id => {
        document.getElementById(id).textContent = 'SCANNING';
        document.getElementById(id).className = 'ml-status processing';
    });
    document.getElementById('yolo-overlay').style.display = 'none';
    
    if (SIM.phase < 9) advanceToPhase(9);
}

// ============ VICTIM DETECTION EVENT ============
function onVictimDetected() {
    const conf = SIM.yoloResult?.detections[0]?.confidence;
    const confStr = conf ? (conf * 100).toFixed(1) : 'N/A';
    addLog('alert', 'YOLO', `⚠ HUMAN DETECTED — YOLOv8n confidence: ${confStr}%`);
    addLog('robot', 'BOT', 'Thermal signature confirmed. Distance: 2.1m. Initiating detailed scan.');
    flashPanel('cave-panel');
    SIM.yoloBboxFlash = 3;
    // Robot continues moving — only manual pause stops it
}

function updateAI(text, type, confidence, reason) {
    const el = document.getElementById('ai-recommendation');
    el.className = `ai-recommendation rec-${type}`;
    document.getElementById('ai-rec-value').textContent = text;
    document.getElementById('ai-confidence').textContent = `CONFIDENCE: ${confidence}%`;
    document.getElementById('ai-rec-reason').textContent = reason;
}

function generateStrategies() {
    const list = document.getElementById('strategies-list');
    list.innerHTML = '';
    const wedgeScore = SIM.wedgeResult?.score || 80;
    const extractPct = SIM.metrics.extraction?.toFixed(1) || '0.0';
    const astarLen = SIM.astarPath?.length || 0;
    const items = [
        { cls: 'danger', icon: '⛔', name: 'DIRECT PULL EXTRACTION', desc: 'Fatal risk: body compression. RF Hazard: DANGER. Wedge geometry creates one-way valve.', badge: 'NOT VIABLE' },
        { cls: 'recommended', icon: '🔧', name: 'HYDRAULIC PASSAGE EXPANSION', desc: `Widen passage +12cm lateral, +8cm vertical. Knull Zone risk drops from ${wedgeScore} to ~${Math.max(15, wedgeScore - 45)}.`, badge: 'RECOMMENDED' },
        { cls: 'recommended', icon: '⛏️', name: 'SECONDARY ACCESS TUNNEL', desc: `Drill from adjacent chamber (4.2m). A* route: ${astarLen} waypoints. Bypass squeeze entirely.`, badge: 'RECOMMENDED' },
        { cls: 'recommended', icon: '🏗️', name: 'REINFORCED SHORING', desc: 'Install hydraulic props at 3 critical points to prevent further collapse during extraction.', badge: 'RECOMMENDED' },
        { cls: 'caution', icon: '🔄', name: 'PULLEY EXTRACTION SYSTEM', desc: `After expansion. RF Regressor: ${extractPct}% feasibility. Requires 4-point anchor rig.`, badge: 'CONDITIONAL' },
        { cls: 'caution', icon: '💨', name: 'VENTILATION + GAS PURGE', desc: 'Deploy forced-air ventilation to clear CO and CH₄ pockets before human entry. MQ sensors monitoring.', badge: 'CONDITIONAL' },
        { cls: 'caution', icon: '🤖', name: 'SERP-X SUPPLY DELIVERY', desc: 'Use snake robot to deliver O₂ mask, water, and thermal blanket to victim before extraction.', badge: 'CONDITIONAL' },
        { cls: 'danger', icon: '⚠️', name: 'CONTROLLED BLASTING', desc: 'Micro-charges to widen passage. Risk of secondary collapse. Last resort only.', badge: 'HIGH RISK' },
    ];
    items.forEach((s, i) => {
        setTimeout(() => {
            const el = document.createElement('div');
            el.className = `strategy-item ${s.cls}`;
            el.innerHTML = `<span class="strategy-icon">${s.icon}</span><div class="strategy-text"><div class="strategy-name">${s.name}</div><div class="strategy-desc">${s.desc}</div></div><span class="strategy-badge">${s.badge}</span>`;
            list.appendChild(el);
        }, i * 300);
    });
}

// ============ LOGGING ============
function addLog(type, tag, msg) {
    const log = document.getElementById('event-log');
    const entry = document.createElement('div');
    entry.className = `log-entry log-${type}`;
    entry.innerHTML = `<span class="log-time">${fmtTime(SIM.missionTime)}</span><span class="log-tag">${tag}</span><span class="log-msg">${msg}</span>`;
    log.appendChild(entry);
    log.scrollTop = log.scrollHeight;
}

function fmtTime(s) {
    return `${Math.floor(s / 3600).toString().padStart(2, '0')}:${Math.floor((s % 3600) / 60).toString().padStart(2, '0')}:${Math.floor(s % 60).toString().padStart(2, '0')}`;
}

function flashPanel(id) {
    const el = document.getElementById(id);
    el.style.boxShadow = '0 0 25px rgba(255,145,0,0.25)';
    el.style.borderColor = 'rgba(255,145,0,0.4)';
    setTimeout(() => { el.style.boxShadow = ''; el.style.borderColor = ''; }, 2000);
}

// ============ SIMULATION LOOP ============
function updateSim(dt) {
    if (!SIM.running || SIM.paused) return;
    SIM.tick++;
    SIM.missionTime += dt;
    SIM.mapRotation += 0.004;

    if (SIM.phase >= 1) updateSnake(dt);

    // Sonar
    if (SIM.tick % 25 === 0) {
        const scale = caveCanvas.width / 80;
        const seg = Math.floor(SIM.robotX / scale);
        const { midY } = getCaveY(seg, caveCanvas.height, caveCanvas.height / 2);
        SIM.sonarPulses.push({ x: SIM.robotX, y: midY, radius: 0, age: 0, maxAge: 50 });
    }
    SIM.sonarPulses = SIM.sonarPulses.filter(p => { p.radius += 2.5; p.age++; return p.age < p.maxAge; });

    if (SIM.yoloBboxFlash > 0) SIM.yoloBboxFlash -= dt;

    updateMetrics();

    // ML pipelines running periodically
    if (SIM.tick % 15 === 0 && SIM.phase >= 3) runYOLO();
    if (SIM.tick % 30 === 0 && SIM.phase >= 5) runHazardDetection();
    if (SIM.tick % 30 === 0 && SIM.phase >= 6) runWedgePrediction();
    if (SIM.tick % 30 === 0 && SIM.phase >= 8) runExtractionFeasibility();

    // Phase progression
    const progress = SIM.robotX / ((SIM.caveData.length - 12) * (caveCanvas.width / 80));
    if (progress > 0.02 && SIM.phase < 2) advanceToPhase(2);
    if (progress > 0.125 && SIM.phase < 3) advanceToPhase(3);
    if (progress > 0.225 && SIM.phase < 4) advanceToPhase(4);
    if (progress > 0.275 && SIM.phase < 5) advanceToPhase(5);
    if (progress > 0.31 && SIM.phase < 6) advanceToPhase(6);

    // Victim detection (dynamic for multiple victims)
    const currentSeg = Math.floor(SIM.robotX / ((caveCanvas.width / 80)));
    const activeVictim = SIM.caveData.find((c, i) => c.isVictim && !c.victimFoundFlag && Math.abs(i - currentSeg) < 8);
    if (activeVictim) {
        activeVictim.victimFoundFlag = true;
        onVictimDetected();
    }
}

function mainLoop(ts) {
    if (!SIM.lastTime) SIM.lastTime = ts;
    const dt = Math.min((ts - SIM.lastTime) / 1000, 0.05);
    SIM.lastTime = ts;

    updateSim(dt);
    drawCave();
    draw3DMap();
    drawAStarMap();
    renderMetrics();

    // Clocks
    const now = new Date();
    document.getElementById('real-clock').textContent = now.toLocaleTimeString('en-US', { hour12: false });
    if (SIM.running) document.getElementById('mission-clock').textContent = fmtTime(SIM.missionTime);

    SIM.animFrame = requestAnimationFrame(mainLoop);
}

// ============ CONTROLS ============
function startSimulation() {
    if (SIM.running && SIM.paused) {
        SIM.paused = false;
        document.getElementById('btn-pause').textContent = '⏸ HOLD';
        addLog('system', 'SYS', 'Simulation resumed.');
        return;
    }
    if (SIM.running) return;

    SIM.running = true;
    SIM.paused = false;
    SIM.caveData = generateCave();
    SIM.debris = generateDebris(SIM.caveData);
    SIM.rescuers = generateRescuers();
    SIM.forwardTeams = generateForwardTeams();
    SIM.snake = [];

    yolo.loadModel();
    addLog('system', 'SYS', 'ML Models loaded: YOLOv8n (3.2M params), RF-50, RF-Reg-100, A*, Knull Zone.');
    addLog('system', 'SYS', `Rescue teams deployed: ${SIM.rescuers.length} at entrance, ${SIM.forwardTeams.length} forward-staged.`);

    document.getElementById('btn-start').disabled = true;
    document.getElementById('btn-pause').disabled = false;

    advanceToPhase(1);

    SIM.lastTime = 0;
    SIM.animFrame = requestAnimationFrame(mainLoop);
}

function pauseSimulation() {
    if (!SIM.running) return;
    SIM.paused = !SIM.paused;
    document.getElementById('btn-pause').textContent = SIM.paused ? '▶ RESUME' : '⏸ HOLD';
    addLog('system', 'SYS', SIM.paused ? 'Simulation paused.' : 'Simulation resumed.');
}

function resetSimulation() {
    SIM.running = false; SIM.paused = false; SIM.phase = 0; SIM.tick = 0;
    SIM.missionTime = 0; SIM.robotX = 0; SIM.robotSpeed = 0; SIM.robotDepth = 0;
    SIM.victimFound = false; SIM.explored = 0; SIM.battery = 100;
    SIM.sonarPulses = []; SIM.snake = []; SIM.mapRotation = 0;
    SIM.yoloResult = null; SIM.hazardResult = null; SIM.wedgeResult = null; SIM.extractResult = null;
    SIM.astarPath = []; SIM.astarExplored = []; SIM.astarGrid = null;
    SIM.yoloBboxFlash = 0;
    SIM.hurdleActive = false; SIM.hurdleType = null; SIM.hurdleSegment = -1;
    SIM.hurdlesPassed = []; SIM.speedBoost = 0; SIM.continueVisible = false;
    SIM.victimsRescued = 0; SIM.forwardTeams = [];
    SIM.metrics = { width: 0, height: 0, orientation: '--', airQuality: 0, wedgeRisk: 0, extraction: 0, slope: 0, co: 0, ch4: 0 };

    if (SIM.animFrame) cancelAnimationFrame(SIM.animFrame);

    document.getElementById('btn-start').disabled = false;
    document.getElementById('btn-pause').disabled = true;
    document.getElementById('btn-pause').textContent = '⏸ HOLD';
    hideContinueButton();
    document.getElementById('yolo-overlay').style.display = 'none';

    // Reset phases
    for (let i = 1; i <= 9; i++) {
        document.getElementById(`phase-${i}`).classList.remove('active', 'complete');
        if (i === 1) document.getElementById(`phase-${i}`).classList.add('active');
        document.getElementById(`phase-${i}-status`).textContent = 'PENDING';
    }

    // Reset AI
    document.getElementById('ai-recommendation').className = 'ai-recommendation';
    document.getElementById('ai-rec-value').textContent = 'AWAITING DATA...';
    document.getElementById('ai-confidence').textContent = 'CONFIDENCE: --';
    document.getElementById('ai-rec-reason').textContent = 'Deploy SERP-X to begin environmental assessment and ML pipeline initialization.';

    // Reset metrics
    ['val-width', 'val-height', 'val-air', 'val-co', 'val-ch4'].forEach(id => document.getElementById(id).innerHTML = '--');
    document.getElementById('val-orientation').textContent = '--';
    ['bar-width', 'bar-height', 'bar-air', 'bar-co', 'bar-ch4'].forEach(id => document.getElementById(id).style.width = '0%');
    ['metric-width', 'metric-air', 'metric-co', 'metric-ch4'].forEach(id => document.getElementById(id).classList.remove('alert', 'warning'));

    // Reset ML panels
    document.getElementById('yolo-class').textContent = '--';
    document.getElementById('yolo-confidence').textContent = '--%';
    document.getElementById('yolo-conf-fill').style.width = '0%';
    document.getElementById('yolo-bbox').textContent = '--';
    document.getElementById('yolo-inference').textContent = '-- ms';
    document.getElementById('hazard-level').textContent = '--';
    document.getElementById('hazard-level').className = 'hazard-level';
    ['hp-safe', 'hp-caution', 'hp-danger'].forEach(id => document.getElementById(id).style.width = '0%');
    ['hp-safe-val', 'hp-caution-val', 'hp-danger-val'].forEach(id => document.getElementById(id).textContent = '--%');
    document.getElementById('rf-trees').innerHTML = '';
    document.getElementById('wedge-score').textContent = '--';
    document.getElementById('wedge-score').style.color = '';
    ['wp-twidth', 'wp-theight', 'wp-slope'].forEach(id => document.getElementById(id).textContent = '-- ');
    document.getElementById('val-extraction').textContent = '--%';
    document.getElementById('val-est-time').textContent = '-- hrs';
    ['yolo-ml-status', 'hazard-ml-status', 'wedge-ml-status', 'extract-ml-status'].forEach(id => {
        document.getElementById(id).textContent = 'STANDBY';
        document.getElementById(id).className = 'ml-status';
    });

    document.getElementById('strategies-list').innerHTML = '<div class="strategy-placeholder">Strategies generated after all ML analyses complete.</div>';
    document.getElementById('event-log').innerHTML = '<div class="log-entry log-system"><span class="log-time">00:00:00</span><span class="log-tag">SYS</span><span class="log-msg">SERP-X initialized. ML pipelines loaded: YOLOv8n, RandomForest, A*.</span></div>';
    document.getElementById('status-message').textContent = 'SYSTEM READY — AWAITING DEPLOYMENT';
    document.getElementById('mission-clock').textContent = '00:00:00';
    document.querySelector('.status-text').textContent = 'STANDBY';
    document.querySelector('.status-text').style.color = '';
    document.querySelector('.status-dot').className = 'status-dot';

    SIM.caveData = generateCave();
    SIM.debris = generateDebris(SIM.caveData);
    drawCave(); draw3DMap(); drawAStarMap();
    drawGauge(gaugeCanvas, gaugeCtx, 0, 'FEASIBILITY');
    drawWedgeGauge(0);
    drawOrientation('--');
}

// ============ INIT ============
window.addEventListener('DOMContentLoaded', () => {
    initCanvases();
    SIM.caveData = generateCave();
    SIM.debris = generateDebris(SIM.caveData);
    SIM.rescuers = generateRescuers();
    SIM.forwardTeams = generateForwardTeams();
    drawCave(); draw3DMap(); drawAStarMap();
    drawGauge(gaugeCanvas, gaugeCtx, 0, 'FEASIBILITY');
    drawWedgeGauge(0);
    drawOrientation('--');

    setInterval(() => {
        const now = new Date();
        document.getElementById('real-clock').textContent = now.toLocaleTimeString('en-US', { hour12: false });
    }, 1000);

    // Idle animation
    (function idle() {
        if (!SIM.running) {
            SIM.tick++;
            SIM.mapRotation += 0.006;
            drawCave(); draw3DMap(); drawAStarMap();
        }
        requestAnimationFrame(idle);
    })();
});
