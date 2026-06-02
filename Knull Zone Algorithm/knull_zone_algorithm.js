/**
 * Knull Zone Algorithm (Wedge Prediction Model)
 * 
 * This algorithm calculates the structural wedge risk (0 - 100) for a human extractor
 * or victim inside a tight cave constriction (like the Birth Canal or Squeeze in Nutty Putty Cave).
 * It evaluates physical dimensions relative to human body metrics (shoulder width, chest depth),
 * slope gravity vectors, and wall convergence.
 */

class KnullZoneAlgorithm {
    constructor() {
        // Average human body dimensions (in centimeters)
        this.shoulderWidth = 45.7; // Standard adult shoulder width
        this.chestDepth = 25.4;    // Standard adult chest depth
        this.hipWidth = 36.0;      // Standard adult hip width
    }

    /**
     * Predicts the wedging/stuck probability and risk components.
     * @param {Object} params 
     * @param {number} params.tunnelWidth - Width of the tunnel passage in cm
     * @param {number} params.tunnelHeight - Height of the tunnel passage in cm
     * @param {number} params.slope - Slope angle in degrees (-90 to +90)
     * @returns {Object} Risk analysis report
     */
    predict(params) {
        const { tunnelWidth, tunnelHeight, slope } = params;

        // 1. Width Clearance Ratio (horizontal clearance relative to shoulder width)
        const widthClearance = (tunnelWidth - this.shoulderWidth) / this.shoulderWidth;
        
        // 2. Height Clearance Ratio (vertical clearance relative to chest depth)
        const heightClearance = (tunnelHeight - this.chestDepth) / this.chestDepth;

        // 3. Slope Gravitational Vector Factor
        // Steeper decline/incline forces body weight into the pinch point under gravity.
        // Math: sin^1.5 of the slope angle, amplified.
        const slopeRad = Math.abs(slope) * Math.PI / 180;
        const slopeFactor = Math.pow(Math.sin(slopeRad), 1.5) * 2;

        // 4. Convergence Risk
        // If the passage width is less than the shoulder width, risk increases rapidly (compression zone).
        const convergenceRisk = widthClearance < 0 ? Math.abs(widthClearance) * 3 : 0;

        // Compute individual risk components normalized to [0, 1]
        // Width risk increases as width clearance drops below 50% extra space
        const widthRisk = Math.max(0, Math.min(1, 1 - (widthClearance + 0.3) / 0.8));
        
        // Height risk increases as height clearance drops below 40% extra space
        const heightRisk = Math.max(0, Math.min(1, 1 - (heightClearance + 0.2) / 0.6));
        
        // Slope risk capped at 1.0
        const slopeRisk = Math.min(1, slopeFactor);
        
        // Convergence risk capped at 1.0
        const convRisk = Math.min(1, convergenceRisk);

        // Weighted combination of risk factors
        // Width: 35%, Height: 20%, Slope: 25%, Convergence: 20%
        const compositeRisk = (
            widthRisk * 0.35 +
            heightRisk * 0.20 +
            slopeRisk * 0.25 +
            convRisk * 0.20
        );

        // Non-linear scaling (power curves) to make extreme tight squeezes exponentially more dangerous
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
            passable: finalScore < 60, // Verdict boundary: >= 60 is deemed a high wedge risk
            slopeAngle: Math.abs(slope)
        };
    }
}

// Export for browser compatibility (global scope) or CommonJS/ES6 environments
if (typeof module !== 'undefined' && module.exports) {
    module.exports = { KnullZoneAlgorithm };
} else {
    window.KnullZoneAlgorithm = KnullZoneAlgorithm;
}
