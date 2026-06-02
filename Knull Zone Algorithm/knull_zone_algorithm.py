#!/usr/bin/env python3
"""
Knull Zone Algorithm (Wedge Prediction Model)

This module implements the Knull Zone Algorithm, which calculates the structural 
wedge risk (0 - 100) for a human operator or victim stuck in narrow, sloping 
subterranean passages. 

The algorithm models the physical clearances of a human body (shoulder width, 
chest depth) relative to passage dimensions, factoring in gravity vectors from 
slope angles and wall convergence.
"""

import math

class KnullZoneAlgorithm:
    def __init__(self):
        # Average adult human dimensions (in centimeters)
        self.shoulder_width = 45.7  # Average shoulder width
        self.chest_depth = 25.4     # Average chest depth
        self.hip_width = 36.0       # Average hip width

    def predict(self, tunnel_width: float, tunnel_height: float, slope: float) -> dict:
        """
        Predicts wedging/stuck probability and returns a detailed risk analysis.
        
        :param tunnel_width: Width of the passage in centimeters
        :param tunnel_height: Height of the passage in centimeters
        :param slope: Slope angle of the passage in degrees (-90 to +90)
        :returns: Dictionary with composite score, component risks, clearances, and passability
        """
        # 1. Width clearance ratio (horizontal space relative to shoulder width)
        width_clearance = (tunnel_width - self.shoulder_width) / self.shoulder_width

        # 2. Height clearance ratio (vertical space relative to chest depth)
        height_clearance = (tunnel_height - self.chest_depth) / self.chest_depth

        # 3. Slope gravitational vector factor
        # Gravity pulls the body deeper into the wedge. Evaluated via sin^1.5 of the slope.
        slope_rad = math.radians(abs(slope))
        slope_factor = math.pow(math.sin(slope_rad), 1.5) * 2

        # 4. Convergence Risk
        # Negative clearance represents physical body compression, accelerating the wedge risk
        convergence_risk = abs(width_clearance) * 3 if width_clearance < 0 else 0.0

        # Normalize component risks to [0, 1]
        width_risk = max(0.0, min(1.0, 1.0 - (width_clearance + 0.3) / 0.8))
        height_risk = max(0.0, min(1.0, 1.0 - (height_clearance + 0.2) / 0.6))
        slope_risk = min(1.0, slope_factor)
        conv_risk = min(1.0, convergence_risk)

        # Composite risk: Width (35%), Height (20%), Slope (25%), Convergence (20%)
        composite_risk = (
            width_risk * 0.35 +
            height_risk * 0.20 +
            slope_risk * 0.25 +
            conv_risk * 0.20
        )

        # Non-linear scaling to match physical reality where narrow limits become exponentially worse
        final_score = round(max(0.0, min(100.0, math.pow(composite_risk, 0.8) * 100.0)))

        return {
            "score": final_score,
            "components": {
                "width_risk_pct": round(width_risk * 100),
                "height_risk_pct": round(height_risk * 100),
                "slope_risk_pct": round(slope_risk * 100),
                "convergence_risk_pct": round(conv_risk * 100)
            },
            "clearance": {
                "width_clearance_pct": round(width_clearance * 100, 1),
                "height_clearance_pct": round(height_clearance * 100, 1)
            },
            "passable": final_score < 60,
            "slope_angle": abs(slope)
        }

if __name__ == "__main__":
    # Test suite demonstrating algorithm behavior under different scenarios
    algo = KnullZoneAlgorithm()
    
    scenarios = [
        {"name": "Safe & Wide Entrance", "width": 100.0, "height": 80.0, "slope": 5.0},
        {"name": "Moderately Tight Decline", "width": 55.0, "height": 40.0, "slope": 25.0},
        {"name": "Nutty Putty Birth Canal (The Squeeze)", "width": 42.0, "height": 24.0, "slope": 70.0},
        {"name": "Sub-shoulder Compression Zone", "width": 38.0, "height": 22.0, "slope": 85.0}
    ]

    print("=" * 60)
    print("KNULL ZONE ALGORITHM — TEST EXECUTION")
    print("=" * 60)

    for i, s in enumerate(scenarios, 1):
        res = algo.predict(s["width"], s["height"], s["slope"])
        print(f"\nScenario {i}: {s['name']}")
        print(f"  Inputs: Width={s['width']}cm, Height={s['height']}cm, Slope={s['slope']}°")
        print(f"  Result Score: {res['score']}/100")
        print(f"  Passable: {'YES' if res['passable'] else 'NO (HIGH WEDGE RISK)'}")
        print(f"  Breakdown: Width Risk={res['components']['width_risk_pct']}%, "
              f"Height Risk={res['components']['height_risk_pct']}%, "
              f"Slope Risk={res['components']['slope_risk_pct']}%, "
              f"Convergence Risk={res['components']['convergence_risk_pct']}%")
    print("-" * 60)
