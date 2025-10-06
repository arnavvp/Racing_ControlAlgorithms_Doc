### Stanley Controller

Next, we discuss the **Stanley Controller**, a path tracking algorithm that uses the vehicle’s **front axle** as the reference point. Unlike the Pure Pursuit method, it corrects both the **heading error** (difference in orientation between the vehicle and path) and the **cross-track error** (lateral distance between the front axle and the nearest way point).


Image 1 and 2


### Stanley Steering Law

As shown, 𝜓(t) represents the **angle between the vehicle heading and the path heading**, while 𝛿 denotes the **steering angle**.  
The **Stanley Method** defines three intuitive steering control laws:

1. **Eliminate Heading Error**  
   The first step corrects the orientation difference between the vehicle and the path:  
   \[
   \delta(t) = \psi(t)
   \]

2. **Eliminate Cross-Track Error**  
   Next, the controller finds the closest point between the vehicle and the path, denoted as *e(t)*.  
   The steering correction is expressed as:  
   \[
   \delta(t) = \psi(t) + \tan^{-1}\left(\frac{k \cdot e(t)}{v(t)}\right)
   \]
   where *k* is a tuning gain and *v(t)* is the vehicle speed.

3. **Apply Steering Angle Limits**  
   To respect physical steering constraints:  
   \[
   \delta(t) \in [\delta_{\min}, \delta_{\max}]
   \]

To avoid division by zero at low speeds, a **softening constant** is added to the denominator, improving numerical stability and ensuring smooth control.


### Why the Stanley Controller Is Effective and Stable

Let’s summarize the reasoning behind the effectiveness and stability of the **Stanley Controller**.

The controller simultaneously accounts for both **heading error** and **cross-track error**, allowing it to handle various driving scenarios smoothly and accurately.

---

#### **Scenario 1: Large Heading Error, Small Cross-Track Error**

When the **heading error (ψ)** is large but the **cross-track error (e)** is small, the vehicle’s steering angle **δ** becomes large in the opposite direction to correct the heading difference.  
This action aligns the vehicle’s orientation with the desired path.

#### **Scenario 2: Large Cross-Track Error, Small Heading Error**

When the **cross-track error** is large but the **heading error** is small, the controller commands a steering angle that directs the vehicle sharply toward the path.  
If the heading error ψ(t) = 0, then δ(t) can reach π/2, meaning the vehicle turns directly toward the path.
 

As the vehicle moves closer to the trajectory, the **cross-track error decreases**, causing the steering angle to gradually reduce. The heading correction term then aligns the vehicle smoothly along the path.



### Why Stanley Method is Superior to Pure Pursuit

The **Stanley Method** provides more stable and accurate path tracking, especially at higher speeds, compared to the **Pure Pursuit** approach.

In **Pure Pursuit**, the controller focuses only on a single look-ahead point and computes a curvature to reach it. This method does not consider the alignment with the overall path direction, often leading to oscillations or instability at higher speeds.

In contrast, the **Stanley Method** accounts for both the **path curvature** and the **heading of the target trajectory**. By comparing the vehicle’s heading with the desired path heading, it effectively reduces lateral error and ensures smoother, more stable tracking.


### Important Implementation Notes

When implementing the **Stanley Controller**, it is crucial to maintain numerical stability and realistic steering behavior by enforcing the following conditions:

1. **Normalize Angles**  
   Always normalize all angular values (such as heading and steering angles) to stay within the range:  
   \[
   [-\pi, +\pi]
   \]  
   This prevents discontinuities when angles wrap around.

2. **Limit Steering Angles**  
   Constrain the computed steering command within the physical steering range of the vehicle:  
   \[
   \delta(t) \in [\delta_{\min}, \delta_{\max}]
   \]  
   This makes sure that the steering output remains achievable by the actual vehicle hardware.
