#What is MPC?

### Cost Function

Before designing the controller, it is essential to define an appropriate **cost function**.  
In this project, our goal is to control the vehicle to **follow a race track** as accurately and smoothly as possible.

Therefore, the cost function should balance two key objectives:

1. **Path tracking accuracy:**  
   Minimize the deviation from the reference trajectory — smaller deviation means better tracking.

2. **Control smoothness:**  
   Minimize the magnitude and rate of control commands to ensure ride comfort — smaller steering effort means smoother driving.

This setup mirrors the **optimal control problem**, where we trade off between **control performance** and **input aggressiveness**.

Hence, the general form of the cost function can be expressed as:

\[
J = \sum_{k=0}^{N} \left( \delta x_k^2 + \lambda u_k^2 \right)
\]

where:
- \( \delta x_k \): Lateral deviation (distance between predicted and reference positions)  
- \( u_k \): Steering input  
- \( \lambda \): Weighting coefficient balancing tracking accuracy and control effort  

To compute \( \delta x_k \), we require a **predictive model** of the vehicle — this forms the foundation of **Model Predictive Control (MPC)**.

---

### Predictive Model - Kinematic Bicycle Model

The core principle of **MPC** is to use a model of the system (the "plant") to **predict its future evolution** over a finite horizon.

In this case, we employ a **kinematic bicycle model**, which captures the vehicle’s motion geometry without considering tire forces.

The model equations are as follows:

\[
\begin{aligned}
\dot{x} &= v \cos(\delta + \theta) \\
\dot{y} &= v \sin(\delta + \theta) \\
\dot{\theta} &= \frac{v}{R} = \frac{v}{L / \sin(\delta)} = \frac{v \sin(\delta)}{L} \\
\dot{\delta} &= \phi
\end{aligned}
\]

Here:
- \( x, y \): Vehicle position  
- \( \theta \): Heading angle  
- \( \delta \): Steering angle  
- \( v \): Linear velocity  
- \( L \): Wheelbase  
- \( \phi \): Steering rate (control input)

The discrete-time form, using a sampling time \( \Delta t \), is:

\[
\begin{aligned}
x_{t+1} &= x_t + \dot{x} \, \Delta t \\
y_{t+1} &= y_t + \dot{y} \, \Delta t \\
\theta_{t+1} &= \theta_t + \dot{\theta} \, \Delta t \\
\delta_{t+1} &= \delta_t + \dot{\delta} \, \Delta t
\end{aligned}
\]

Thus, the **state vector** and **control inputs** are defined as:

\[
X = [x, y, \theta, \delta]^T, \quad U = [v, \phi]^T
\]

where:
- \( X \): System states — position, heading, and steering  
- \( U \): Control inputs — velocity and steering rate  

This predictive model enables the MPC to simulate the future trajectory of the vehicle and compute optimal control inputs that minimize the cost function.



### IMPLEMENTATION



## Aim

Implement **Model Predictive Control (MPC)** along with the following objectives:

- Ensure the car follows the **ideal raceline**, even at low speeds.  
- Use the **map collected by SLAM (after the first lap)** and the **vehicle model** — without relying on any predetermined logic.  
- Incorporate **actuator dynamics**, **limits**, and **friction constraints**.  
- Avoid relying on simple convex formulations (like LQR) — solve for a **nonlinear solution**.  
- Model the race car using a **dynamic bicycle model** with **nonlinear tire force laws**.

---

## Vehicle Model

### Assumptions
- No load transfer.  
- Driving on a flat surface.  
- Combined slip is neglected — lateral and longitudinal slips are treated independently.  
- Forces on the drivetrain are applied at the **center of gravity (COG)**, not at the tire contact patches.

---

### Dynamic Bicycle Model

In this model:
- \( F_{R,y} \) and \( F_{F,y} \) represent the lateral tire forces at the rear and front wheels respectively.  

**State vector:**
\[
\tilde{x} = [X, Y, \phi, v_x, v_y, r]^T
\]

**Control inputs:**
\[
\tilde{u} = [\delta, D]^T
\]

Where:
- \( \delta \): Steering control  
- \( D \): Drive command (–1 = full brake, 1 = full throttle)

---

### Limitations

At **low speeds**, this model performs poorly because the **slip angle** (α) becomes very high even when the car barely moves.

Slip angle definition:
\[
\alpha = \text{angle between wheel heading and actual velocity direction}
\]

When \( v_x \) is very small, α increases significantly, causing instability. Hence, forces alone cannot accurately describe vehicle motion at low speeds.

---

## Combining Both Models

A **blended model** is formed using a weighted mean of the **dynamic** and **kinematic** models.

Define speed regions:
- **Slow region:** 0–2 m/s → Only kinematic model  
- **Gray region:** 3–5 m/s → Blended model  
- **Fast region:** >5 m/s → Only dynamic model  

This ensures stability and continuity across all speed ranges.

---

## Tire Constraints

A tire cannot simultaneously produce its maximum **longitudinal** and **lateral** forces — it has a limited **friction budget**, modeled as an **ellipse**:

\[
F_x^2 + F_y^2 \leq F_{\max}^2
\]

Interpretation:
- More \( F_x \) → less \( F_y \) available  
- Full lateral force → zero longitudinal force, and vice versa  

Tuning:
- Smaller \( p_{\text{long}} \): allows more lateral force 
- Larger \( p_{\text{long}} \): allows more longitudinal force 

---

#### Formulating Errors:

Instead of targeting discrete points, the vehicle minimizes:
1. **Contouring error (E_c)** — lateral deviation from the path.  
2. **Lag error (E_l)** — distance behind or ahead of the target position along the path.


## Contouring Formulation

The **center line** of the track is treated as the **contour**.  
The goal: **follow this contour as fast as possible** while respecting vehicle dynamics.

We parameterize the center line by the **arc length θ**:

\[
\text{Contour: } (X_{\text{ref}}(\theta), Y_{\text{ref}}(\theta))
\]

---

## Lag Error Formulation

The **lag error** represents the **longitudinal displacement** along the track.  
The goal: **minimize this lag** to stay on schedule along the reference contour.

We define lag error along the **arc length θ**:

\[
e_\ell = \theta_{\text{vehicle}} - \theta_{\text{ref}}
\]

- \(\theta_{\text{ref}}\): arc length of the reference point on the center line  
- \(\theta_{\text{vehicle}}\): arc length of the vehicle projected onto the center line

---

### Projection

Given the current car position \((X, Y)\), find the closest point on the contour:
\[
(X_{\text{ref}}(\theta), Y_{\text{ref}}(\theta))
\]

θ represents arc-length along the track:
- \( \theta = 0 \): start of track  
- \( \theta = L \): end of track (L = total length)

Although computing true arc length is expensive, θ serves as a **computationally efficient approximation** that correlates with real distance.

---

### Cost Function

We define the total cost as a weighted combination of contouring, lag, and velocity terms:

\[
J = q_c E_c^2 + q_l E_l^2 - \gamma v_{\theta,k}
\]

where:
- \( v_{\theta,k} \): speed along the path at timestep *k*  
- \( \gamma, q_c, q_l \ge 0 \): weights for tuning  

Interpretation:
- High path speed \( v_{\theta,k} \) reduces the overall cost (encourages faster tracking).  
- Lag and velocity terms are treated separately since they serve different purposes.

---

## Track Constraints

The track is simplified as a **circular region** around \((X(\theta), Y(\theta))\), ensuring the vehicle stays within the boundaries.  
This circular constraint is computationally efficient and easier than modeling exact racetrack borders.

---

## Double Integrator Model

To track θ, velocity, and acceleration relative to the path, a **double integrator model** is introduced.

This model helps the controller predict motion along the reference path, maintaining continuity in both position and velocity.

---

## MPC Problem Formulation

We use tilde (~) to denote the **vehicle’s internal state**.

**Vehicle state:**
\[
\tilde{x} = [X, Y, \psi, v_x, v_y, \omega]^T
\]

**Vehicle control:**
\[
\tilde{u} = [\delta, D]^T
\]

Note: \( v_\theta \) is not part of the vehicle state — it affects θ but acts as a **control input** to the car.

**Full system state:**
\[
x = [\tilde{x}, v_\theta, u]
\]

Δu (change in control) is **not** included in the current state, as it is a **predicted variable** in MPC optimization.

To convert from continuous to discrete time, we apply **Runge-Kutta 4 (RK4)** integration.

---

### Control Input Penalty

Two penalties are added:
1. **Control magnitude penalty:** prevents excessive steering or throttle inputs.  
2. **Rate of change penalty:** penalizes sudden variations in control inputs (\( \Delta u_k \)), ensuring smooth actuation.

---

### Side-Slip Penalty

Let:
- \( \beta_{\text{kin}} \): safe “non-sliding” reference slip  
- \( \beta_{\text{dyn}} \): actual slip during dynamic motion  

The MPC minimizes their difference to reduce sliding and maintain stable handling.

---

### Constraint Cost (Slack Variable)

When constraints are violated (e.g., due to sharp maneuvers or dynamics), introduce a **slack variable \( S_c \ge 0 \)**.

This allows small temporary violations without making the optimization infeasible.  
Slack is penalized both **linearly** and **quadratically** in the cost function.

---

### Final MPC Problem

The final cost function combines:
- Contouring and lag errors  
- Control penalties  
- Slip penalties  
- Slack penalties  