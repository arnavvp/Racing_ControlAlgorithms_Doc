# PID Control

## 1. Introduction
A **PID controller** (Proportional–Integral–Derivative) is one of the most widely used feedback controllers in control systems.  
It continuously calculates an **error value** as the difference between a desired setpoint and a measured process variable, and applies a correction based on proportional, integral, and derivative terms.

---

## 2. PID Equation

The control signal \( u(t) \) is defined as:

\[
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{d e(t)}{dt}
\]

Where:
- \( e(t) = r(t) - y(t) \) → error (difference between reference \( r(t) \) and output \( y(t) \))
- \( K_p \) → proportional gain  
- \( K_i \) → integral gain  
- \( K_d \) → derivative gain  

---

## 3. Roles of Each Term

### (a) Proportional (P)
- Output is proportional to the current error.
- Large \( K_p \) → faster response, but can overshoot.

\[
u_P(t) = K_p \, e(t)
\]

---

### (b) Integral (I)
- Accumulates past errors.
- Eliminates **steady-state error**.
- Too large \( K_i \) may cause instability.

\[
u_I(t) = K_i \int_0^t e(\tau) \, d\tau
\]

---

### (c) Derivative (D)
- Reacts to the **rate of change** of error.
- Provides damping → reduces overshoot.
- Sensitive to noise.

\[
u_D(t) = K_d \frac{d e(t)}{dt}
\]

---

## 4. Discrete-Time PID

In digital implementations (e.g., in robotics or embedded systems), PID is often computed at discrete timesteps \( k \):

\[
u[k] = K_p e[k] + K_i \sum_{j=0}^k e[j] \Delta t + K_d \frac{e[k] - e[k-1]}{\Delta t}
\]

---

## 5. Tuning Methods

- **Trial and Error**
- **Ziegler–Nichols method**
- **Cohen–Coon method**
- **Software optimization (auto-tuning)**

---

## 6. Applications
- Cruise control in cars  
- Drone stabilization  
- Industrial process control  
- Autonomous racing cars (steering & throttle)  

---

## 7. Summary
- PID = **Proportional + Integral + Derivative**  
- Each term plays a role in balancing speed, accuracy, and stability.  
- Proper tuning of \( K_p, K_i, K_d \) is crucial for performance.
