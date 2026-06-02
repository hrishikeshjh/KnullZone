/* =============================================
   ML ALGORITHMS — Pure JavaScript Implementations
   YOLOv8 Nano, Random Forest, A*, Wedge Prediction
   ============================================= */

// ============================================================
// 1. YOLOv8 NANO — Simulated Object Detection
//    Real YOLOv8n architecture: CSPDarknet backbone, C2f modules,
//    SPPF, PAN-FPN neck, decoupled head. We simulate the
//    inference pipeline with realistic confidence scoring.
// ============================================================

class YOLOv8Nano {
    constructor() {
        this.classes = ['person', 'helmet', 'body_part', 'equipment', 'debris'];
        this.inputSize = 640;
        this.confThreshold = 0.25;
        this.iouThreshold = 0.45;
        this.modelLoaded = false;
        this.inferenceTime = 0;
        this.detections = [];

        // Simulated weights — anchor sensitivities per class
        this.classWeights = {
            person: { thermal: 0.85, motion: 0.3, shape: 0.75 },
            helmet: { thermal: 0.2, motion: 0.05, shape: 0.9 },
            body_part: { thermal: 0.7, motion: 0.15, shape: 0.6 },
            equipment: { thermal: 0.1, motion: 0.0, shape: 0.8 },
            debris: { thermal: 0.05, motion: 0.0, shape: 0.5 }
        };
    }

    loadModel() {
        this.modelLoaded = true;
        return { params: '3.2M', gflops: 8.7, layers: 225 };
    }

    // Simulate Non-Maximum Suppression
    nms(boxes, scores, iouThreshold) {
        const indices = scores.map((s, i) => i).sort((a, b) => scores[b] - scores[a]);
        const keep = [];

        while (indices.length > 0) {
            const current = indices.shift();
            keep.push(current);

            const remaining = [];
            for (const idx of indices) {
                const iou = this.computeIoU(boxes[current], boxes[idx]);
                if (iou < iouThreshold) remaining.push(idx);
            }
            indices.length = 0;
            indices.push(...remaining);
        }
        return keep;
    }

    computeIoU(boxA, boxB) {
        const x1 = Math.max(boxA.x, boxB.x);
        const y1 = Math.max(boxA.y, boxB.y);
        const x2 = Math.min(boxA.x + boxA.w, boxB.x + boxB.w);
        const y2 = Math.min(boxA.y + boxA.h, boxB.y + boxB.h);

        const intersection = Math.max(0, x2 - x1) * Math.max(0, y2 - y1);
        const areaA = boxA.w * boxA.h;
        const areaB = boxB.w * boxB.h;

        return intersection / (areaA + areaB - intersection + 1e-6);
    }

    // Run detection on simulated frame data
    detect(frameData) {
        const startTime = performance.now();

        const { thermalSignature, distance, passageWidth, victimPresent, victimX, victimY } = frameData;

        const rawDetections = [];

        if (victimPresent) {
            // Simulated backbone feature extraction
            const thermalScore = Math.min(1, thermalSignature * this.classWeights.person.thermal);
            const shapeScore = this.classWeights.person.shape * (1 - distance / 100);
            const noiseFactor = (Math.random() - 0.5) * 0.08;

            // C2f module confidence aggregation
            const rawConf = (thermalScore * 0.5 + shapeScore * 0.5) + noiseFactor;
            const confidence = Math.max(0, Math.min(0.98, rawConf));

            if (confidence > this.confThreshold) {
                // Bounding box prediction (scaled to input size)
                const bboxNoise = () => (Math.random() - 0.5) * 8;
                rawDetections.push({
                    class: 'person',
                    confidence: confidence,
                    bbox: {
                        x: victimX - 18 + bboxNoise(),
                        y: victimY - 25 + bboxNoise(),
                        w: 36 + bboxNoise() * 0.5,
                        h: 50 + bboxNoise() * 0.5
                    }
                });

                // Also detect body parts at closer range
                if (distance < 30) {
                    rawDetections.push({
                        class: 'body_part',
                        confidence: confidence * 0.7 + Math.random() * 0.1,
                        bbox: {
                            x: victimX - 10 + bboxNoise(),
                            y: victimY - 15 + bboxNoise(),
                            w: 20, h: 22
                        }
                    });
                }
            }
        }

        // Random debris detections
        if (Math.random() < 0.15) {
            rawDetections.push({
                class: 'debris',
                confidence: 0.3 + Math.random() * 0.25,
                bbox: {
                    x: Math.random() * 200,
                    y: Math.random() * 100,
                    w: 15 + Math.random() * 20,
                    h: 10 + Math.random() * 15
                }
            });
        }

        // Apply NMS
        if (rawDetections.length > 1) {
            const boxes = rawDetections.map(d => d.bbox);
            const scores = rawDetections.map(d => d.confidence);
            const keepIndices = this.nms(boxes, scores, this.iouThreshold);
            this.detections = keepIndices.map(i => rawDetections[i]);
        } else {
            this.detections = rawDetections;
        }

        // Simulated inference time (realistic for YOLOv8n)
        this.inferenceTime = 12 + Math.random() * 8; // 12-20ms typical for nano
        const endTime = performance.now();

        return {
            detections: this.detections,
            inferenceTime: this.inferenceTime,
            frameTime: endTime - startTime
        };
    }

    getBestPersonDetection() {
        const persons = this.detections.filter(d => d.class === 'person');
        if (persons.length === 0) return null;
        return persons.reduce((best, d) => d.confidence > best.confidence ? d : best);
    }
}


// ============================================================
// 2. RANDOM FOREST — Hazard Classification
//    Ensemble of decision trees for classifying cave conditions
//    into SAFE / CAUTION / DANGER
// ============================================================

class DecisionTree {
    constructor(id, maxDepth = 5) {
        this.id = id;
        this.maxDepth = maxDepth;
        this.tree = null;
    }

    // Build a simple random decision tree based on feature splits
    buildTree(depth = 0) {
        if (depth >= this.maxDepth) {
            // Leaf: random class distribution
            const probs = this.randomProbs();
            return { type: 'leaf', probs };
        }

        const features = ['width', 'height', 'slope', 'airQuality', 'rockStability', 'waterLevel'];
        const feature = features[Math.floor(Math.random() * features.length)];
        const threshold = Math.random();

        return {
            type: 'node',
            feature,
            threshold,
            left: this.buildTree(depth + 1),
            right: this.buildTree(depth + 1)
        };
    }

    randomProbs() {
        let s = Math.random(), c = Math.random(), d = Math.random();
        const total = s + c + d;
        return { safe: s / total, caution: c / total, danger: d / total };
    }

    // Predict class probabilities for given features
    predict(features) {
        if (!this.tree) this.tree = this.buildTree();
        return this.traverse(this.tree, features);
    }

    traverse(node, features) {
        if (node.type === 'leaf') return node.probs;
        const val = features[node.feature] || 0;
        if (val <= node.threshold) return this.traverse(node.left, features);
        else return this.traverse(node.right, features);
    }
}

class RandomForestClassifier {
    constructor(nTrees = 50) {
        this.nTrees = nTrees;
        this.trees = [];
        this.treeVotes = [];

        for (let i = 0; i < nTrees; i++) {
            this.trees.push(new DecisionTree(i, 4 + Math.floor(Math.random() * 3)));
        }
    }

    // Predict with full ensemble — returns class probabilities and per-tree votes
    predict(features) {
        // Normalize features to [0, 1]
        const norm = {
            width: Math.min(1, features.width / 120),
            height: Math.min(1, features.height / 100),
            slope: Math.min(1, Math.abs(features.slope) / 90),
            airQuality: Math.min(1, features.airQuality / 21),
            rockStability: features.rockStability || 0.5,
            waterLevel: features.waterLevel || 0.1
        };

        // Apply domain knowledge as feature engineering
        // Narrow passages are more dangerous
        const widthDanger = 1 - norm.width;
        const slopeDanger = norm.slope;
        const airDanger = 1 - norm.airQuality;

        // Augmented features for decision trees
        const augFeatures = {
            ...norm,
            widthDanger,
            slopeDanger,
            airDanger,
            compositeRisk: (widthDanger * 0.4 + slopeDanger * 0.3 + airDanger * 0.3)
        };

        let totalSafe = 0, totalCaution = 0, totalDanger = 0;
        this.treeVotes = [];

        for (const tree of this.trees) {
            const probs = tree.predict(augFeatures);

            // Weight by domain-knowledge composite risk
            const riskBias = augFeatures.compositeRisk;
            const adjusted = {
                safe: probs.safe * (1 - riskBias * 0.7),
                caution: probs.caution * (1 + riskBias * 0.2),
                danger: probs.danger * (1 + riskBias * 0.8)
            };

            const tTotal = adjusted.safe + adjusted.caution + adjusted.danger;
            const normAdj = {
                safe: adjusted.safe / tTotal,
                caution: adjusted.caution / tTotal,
                danger: adjusted.danger / tTotal
            };

            // Each tree votes
            let vote;
            if (normAdj.safe >= normAdj.caution && normAdj.safe >= normAdj.danger) vote = 'safe';
            else if (normAdj.danger >= normAdj.caution) vote = 'danger';
            else vote = 'caution';

            this.treeVotes.push(vote);
            totalSafe += normAdj.safe;
            totalCaution += normAdj.caution;
            totalDanger += normAdj.danger;
        }

        const n = this.nTrees;
        const result = {
            safe: totalSafe / n,
            caution: totalCaution / n,
            danger: totalDanger / n
        };

        // Determine final class
        let predicted;
        if (result.danger >= result.safe && result.danger >= result.caution) predicted = 'DANGER';
        else if (result.caution >= result.safe) predicted = 'CAUTION';
        else predicted = 'SAFE';

        return { probabilities: result, predicted, treeVotes: this.treeVotes };
    }
}


// ============================================================
// 3. RANDOM FOREST REGRESSOR — Extraction Feasibility
//    Predicts extraction probability (0-100%) using ensemble regression
// ============================================================

class RFRegressorTree {
    constructor(id) {
        this.id = id;
        this.splits = this.generateSplits();
    }

    generateSplits() {
        // Random split points for regression tree
        return {
            widthSplit: 0.2 + Math.random() * 0.6,
            heightSplit: 0.2 + Math.random() * 0.6,
            riskSplit: 0.3 + Math.random() * 0.4,
            slopeSplit: 0.2 + Math.random() * 0.5,
            leafValues: Array.from({ length: 8 }, () => Math.random() * 100)
        };
    }

    predict(features) {
        const { width, height, wedgeRisk, slope } = features;
        // Binary decision path through 3 splits → 8 leaf nodes
        let idx = 0;
        if (width > this.splits.widthSplit) idx += 4;
        if (height > this.splits.heightSplit) idx += 2;
        if (wedgeRisk > this.splits.riskSplit) idx += 1;

        // Base prediction from leaf, adjusted by domain knowledge
        let pred = this.splits.leafValues[idx];

        // Domain adjustments
        pred *= (width * 0.6 + 0.4);       // Wider passages → higher feasibility
        pred *= (1 - wedgeRisk * 0.5);       // Higher risk → lower feasibility
        pred *= (1 - slope * 0.3);           // Steeper slope → harder extraction

        return Math.max(0, Math.min(100, pred));
    }
}

class RandomForestRegressor {
    constructor(nTrees = 100) {
        this.nTrees = nTrees;
        this.trees = [];
        for (let i = 0; i < nTrees; i++) {
            this.trees.push(new RFRegressorTree(i));
        }
    }

    predict(features) {
        // Normalize inputs
        const norm = {
            width: Math.min(1, features.width / 120),
            height: Math.min(1, features.height / 100),
            wedgeRisk: Math.min(1, features.wedgeRisk / 100),
            slope: Math.min(1, Math.abs(features.slope) / 90)
        };

        // Get predictions from all trees
        const predictions = this.trees.map(tree => tree.predict(norm));

        // Ensemble: average prediction
        const mean = predictions.reduce((s, v) => s + v, 0) / predictions.length;

        // Standard deviation for confidence interval
        const variance = predictions.reduce((s, v) => s + (v - mean) ** 2, 0) / predictions.length;
        const std = Math.sqrt(variance);

        return {
            prediction: Math.max(2, Math.min(95, mean)),
            std: std,
            lowerBound: Math.max(0, mean - 1.96 * std),
            upperBound: Math.min(100, mean + 1.96 * std),
            treePredictions: predictions
        };
    }
}


// ============================================================
// 4. A* PATHFINDING — Rescue Route Optimization
//    Finds shortest/safest path through cave grid
// ============================================================

class AStarPathfinder {
    constructor() {
        this.grid = [];
        this.rows = 0;
        this.cols = 0;
        this.path = [];
        this.explored = [];
        this.openSet = [];
    }

    // Generate navigable grid from cave data
    buildGrid(caveData, gridRows = 40, gridCols = 60) {
        this.rows = gridRows;
        this.cols = gridCols;
        this.grid = [];

        for (let r = 0; r < gridRows; r++) {
            this.grid[r] = [];
            for (let c = 0; c < gridCols; c++) {
                const segIdx = Math.floor((c / gridCols) * caveData.length);
                const seg = caveData[Math.min(segIdx, caveData.length - 1)];

                // Map row to vertical position in cave
                const normalizedY = (r / gridRows - 0.5) * 200; // -100 to +100

                // Check if this cell is inside the passage
                const isInside = normalizedY > seg.top && normalizedY < seg.bottom;
                const distToWall = isInside
                    ? Math.min(normalizedY - seg.top, seg.bottom - normalizedY)
                    : 0;

                // Cost: closer to walls = higher cost (more dangerous)
                let cost = 1;
                if (!isInside) {
                    cost = Infinity; // Wall
                } else {
                    // Higher cost near walls and in hazardous zones
                    cost = 1 + (1 / (distToWall + 0.5)) * 3;
                    if (seg.hazard > 0.5) cost += seg.hazard * 10;
                    if (seg.hazard > 0.8) cost += 20;
                }

                this.grid[r][c] = {
                    r, c, cost,
                    walkable: isInside,
                    hazard: seg.hazard,
                    g: Infinity, h: 0, f: Infinity,
                    parent: null,
                    inOpen: false, inClosed: false
                };
            }
        }

        return this.grid;
    }

    heuristic(a, b) {
        // Weighted Euclidean — prefer horizontal movement
        const dx = Math.abs(a.c - b.c);
        const dy = Math.abs(a.r - b.r);
        return Math.sqrt(dx * dx * 1.0 + dy * dy * 1.5);
    }

    findPath(startR, startC, endR, endC) {
        // Reset
        for (let r = 0; r < this.rows; r++) {
            for (let c = 0; c < this.cols; c++) {
                const cell = this.grid[r][c];
                cell.g = Infinity;
                cell.f = Infinity;
                cell.parent = null;
                cell.inOpen = false;
                cell.inClosed = false;
            }
        }

        this.path = [];
        this.explored = [];
        this.openSet = [];

        const start = this.grid[startR]?.[startC];
        const end = this.grid[endR]?.[endC];
        if (!start || !end || !start.walkable || !end.walkable) return [];

        start.g = 0;
        start.h = this.heuristic(start, end);
        start.f = start.h;
        start.inOpen = true;
        this.openSet.push(start);

        let iterations = 0;
        const maxIterations = 5000;

        while (this.openSet.length > 0 && iterations < maxIterations) {
            iterations++;

            // Find node with lowest f
            let lowestIdx = 0;
            for (let i = 1; i < this.openSet.length; i++) {
                if (this.openSet[i].f < this.openSet[lowestIdx].f) lowestIdx = i;
            }

            const current = this.openSet.splice(lowestIdx, 1)[0];
            current.inOpen = false;
            current.inClosed = true;
            this.explored.push(current);

            // Goal reached
            if (current.r === end.r && current.c === end.c) {
                let node = current;
                while (node) {
                    this.path.unshift({ r: node.r, c: node.c });
                    node = node.parent;
                }
                return this.path;
            }

            // Expand neighbors (8-directional)
            const neighbors = this.getNeighbors(current);
            for (const neighbor of neighbors) {
                if (neighbor.inClosed || !neighbor.walkable) continue;

                const tentativeG = current.g + neighbor.cost * this.heuristic(current, neighbor);

                if (tentativeG < neighbor.g) {
                    neighbor.parent = current;
                    neighbor.g = tentativeG;
                    neighbor.h = this.heuristic(neighbor, end);
                    neighbor.f = neighbor.g + neighbor.h;

                    if (!neighbor.inOpen) {
                        neighbor.inOpen = true;
                        this.openSet.push(neighbor);
                    }
                }
            }
        }

        return this.path; // May be empty if no path found
    }

    getNeighbors(node) {
        const neighbors = [];
        const dirs = [[-1,-1],[-1,0],[-1,1],[0,-1],[0,1],[1,-1],[1,0],[1,1]];

        for (const [dr, dc] of dirs) {
            const nr = node.r + dr;
            const nc = node.c + dc;
            if (nr >= 0 && nr < this.rows && nc >= 0 && nc < this.cols) {
                neighbors.push(this.grid[nr][nc]);
            }
        }
        return neighbors;
    }
}


// ============================================================
// 5. WEDGE PREDICTION MODEL
//    Uses tunnel width, height, slope, human shoulder width
//    to predict wedge risk score
// ============================================================

class WedgePredictionModel {
    constructor() {
        this.shoulderWidth = 45.7; // Average human shoulder width in cm
        this.chestDepth = 25.4;    // Average chest depth in cm
        this.hipWidth = 36.0;      // Average hip width in cm
    }

    predict(params) {
        const { tunnelWidth, tunnelHeight, slope } = params;

        // Width clearance ratio: how much space relative to shoulder
        const widthClearance = (tunnelWidth - this.shoulderWidth) / this.shoulderWidth;
        const heightClearance = (tunnelHeight - this.chestDepth) / this.chestDepth;

        // Slope factor: steeper slope increases wedge risk exponentially
        const slopeRad = Math.abs(slope) * Math.PI / 180;
        const slopeFactor = Math.pow(Math.sin(slopeRad), 1.5) * 2;

        // Convergence analysis: if walls converge, risk increases dramatically
        const convergenceRisk = widthClearance < 0 ? Math.abs(widthClearance) * 3 : 0;

        // Compute individual risk components (0-1)
        const widthRisk = Math.max(0, Math.min(1, 1 - (widthClearance + 0.3) / 0.8));
        const heightRisk = Math.max(0, Math.min(1, 1 - (heightClearance + 0.2) / 0.6));
        const slopeRisk = Math.min(1, slopeFactor);
        const convRisk = Math.min(1, convergenceRisk);

        // Weighted combination
        const compositeRisk = (
            widthRisk * 0.35 +
            heightRisk * 0.20 +
            slopeRisk * 0.25 +
            convRisk * 0.20
        );

        // Non-linear scaling — makes extreme cases more extreme
        const finalScore = Math.round(
            Math.max(0, Math.min(100, Math.pow(compositeRisk, 0.8) * 100))
        );

        return {
            score: finalScore,
            components: {
                widthRisk: Math.round(widthRisk * 100),
                heightRisk: Math.round(heightRisk * 100),
                slopeRisk: Math.round(slopeRisk * 100),
                convergenceRisk: Math.round(convRisk * 100)
            },
            clearance: {
                widthClearance: (widthClearance * 100).toFixed(1),
                heightClearance: (heightClearance * 100).toFixed(1)
            },
            passable: finalScore < 60,
            slopeAngle: Math.abs(slope)
        };
    }
}


// ============================================================
// EXPORTS (global scope for browser)
// ============================================================
window.YOLOv8Nano = YOLOv8Nano;
window.RandomForestClassifier = RandomForestClassifier;
window.RandomForestRegressor = RandomForestRegressor;
window.AStarPathfinder = AStarPathfinder;
window.WedgePredictionModel = WedgePredictionModel;
