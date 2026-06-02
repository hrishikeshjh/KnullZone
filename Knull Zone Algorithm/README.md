# Knull Zone Algorithm (Wedge Prediction Model)

The **Knull Zone Algorithm** (commercially referred to as the *Wedge Prediction Model* in the SERP-X dashboard) is a mathematical heuristic model designed to analyze and predict the risk of a human being or robotic explorer becoming wedged or stuck in a narrow, sloping subterranean passage. 

This algorithm was inspired by structural constraints encountered in real-world cave rescue scenarios, most notably the 2009 Nutty Putty Cave incident (e.g., the *Birth Canal*, *Ed's Push*, and similar constrictions).

---

## 📐 Mathematical Model & Physics

The algorithm evaluates risk across four primary vectors, combining linear clearance ratios with trigonometric gravity vectors to calculate a composite score from **0 (safe, fully passable)** to **100 (complete physical entrapment / high wedge risk)**.

### 1. Width Clearance ($C_w$)
Measures the horizontal clearance of the passage relative to the average adult human shoulder width ($W_{shoulder} = 45.7\text{ cm}$):
$$C_w = \frac{W_{tunnel} - W_{shoulder}}{W_{shoulder}}$$

### 2. Height Clearance ($C_h$)
Measures the vertical clearance of the passage relative to the average adult human chest depth ($D_{chest} = 25.4\text{ cm}$):
$$C_h = \frac{H_{tunnel} - D_{chest}}{D_{chest}}$$

### 3. Slope Gravity Vector ($F_g$)
As the slope of a passage increases, gravity pulls the body down into the V-shaped constriction, transforming forward momentum into compressive wedging force. The algorithm calculates this using a non-linear trigonometric projection:
$$F_g = 2 \times \sin^{1.5}(\theta_{slope})$$
*Where $\theta_{slope}$ is the absolute slope angle in radians.*

### 4. Convergence Risk ($R_{conv}$)
If the tunnel width is smaller than the human shoulder width ($C_w < 0$), the body undergoes lateral compression. This represents a "one-way valve" geometry where backing out is physically impossible.
$$R_{conv} = \begin{cases} 3 \times |C_w| & \text{if } C_w < 0 \\ 0 & \text{if } C_w \ge 0 \end{cases}$$

---

## 🧮 Composite Scoring

The individual risk components are normalized to $[0, 1]$ and aggregated using domain-weighted coefficients:

| Risk Component | Symbol | Normalized Formula | Weight | Description |
| :--- | :--- | :--- | :--- | :--- |
| **Width Risk** | $R_w$ | $\max\left(0, \min\left(1, 1 - \frac{C_w + 0.3}{0.8}\right)\right)$ | **35%** | Measures lateral pinch probability. |
| **Height Risk** | $R_h$ | $\max\left(0, \min\left(1, 1 - \frac{C_h + 0.2}{0.6}\right)\right)$ | **20%** | Measures vertical compression probability. |
| **Slope Risk** | $R_s$ | $\min(1.0, F_g)$ | **25%** | Measures gravity-assisted wedging. |
| **Convergence Risk**| $R_c$ | $\min(1.0, R_{conv})$ | **20%** | Multiplier for sub-body compression. |

The composite risk score is calculated as:
$$\text{Composite Risk} = 0.35 R_w + 0.20 R_h + 0.25 R_s + 0.20 R_c$$

### Non-Linear Scaling
To account for the critical physical threshold where a passage goes from "difficult squeeze" to "absolute block," a power curve is applied:
$$\text{Wedge Score} = \text{round}\left( \max\left(0, \min\left(100, \text{Composite Risk}^{0.8} \times 100\right)\right) \right)$$

---

## 🚦 Risk Categories & Passability

* **Score < 60:** **Passable**. The passage provides enough clearance for an operator or robot to wiggle through or reverse.
* **Score $\ge$ 60:** **High Wedge Risk (Non-Passable)**. There is a high probability of mechanical lock or chest compression, preventing self-extraction.

---

## 💻 Multi-Language Support

We provide official implementations in two languages:
1. **JavaScript ([knull_zone_algorithm.js](file:///c:/Users/Hrishikesh%20Jha/Desktop/serp/Knull%20Zone%20Algorithm/knull_zone_algorithm.js))**: Used for real-time visualization and rendering inside the dashboard client-side application.
2. **Python ([knull_zone_algorithm.py](file:///c:/Users/Hrishikesh%20Jha/Desktop/serp/Knull%20Zone%20Algorithm/knull_zone_algorithm.py))**: Used for scripting, standalone command-line testing, and integration into backend service APIs.
