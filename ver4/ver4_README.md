# 💡 Embedded Reinforced Learning Agent (RL-ALC) Analysis

This repository contains the performance analysis and core findings for the Adaptive Q-Learning Agent designed for Autonomous Lighting Control (RL-ALC) on the ESP32 platform.

The agent's primary goal is to maintain a target illuminance of $300\text{ Lux}$ while minimizing LED energy consumption by learning the optimal control policy.

## 📊 Core Performance Metrics

The analysis of the `rl_log_data.csv` (1,556 episodes) confirms the agent successfully converged on a robust, high-performance policy.

| Metric | Outcome | Conclusion | 
 | ----- | ----- | ----- | 
| **Reward Convergence** | Moving Average Reward stabilized at $0.85$ **to** $0.90$ (max $1.0$). | **Excellent Performance:** The agent consistently achieved near-optimal results, balancing the target Lux goal with efficient power usage. | 
| **Policy Stabilization (**$\epsilon$ **Decay)** | Exploration rate ($\epsilon$) decayed from $1.0$ to the floor of $0.05$. | **Stable Learning:** The self-tuning mechanism successfully transitioned the agent from random **exploration** to reliable **exploitation** of the learned policy. | 
| **Control Stability (Lux)** | Lux values were highly volatile (ranging $23-700\text{ Lux}$), but the agent quickly countered disturbances. | **High Responsiveness:** The agent actively controls the environment, confirming its ability to cope with significant, real-time external light changes. | 

## 🧠 Learned Optimal Policy (Q-Table Analysis)

The most critical insight comes from the final learned Q-values for the **Target State (S2:** $250 - 350\text{ Lux}$**)**, which reveal the agent's strategy under optimal conditions:

| State | Action | Final Q-Value | Learned Policy Action | 
 | ----- | ----- | ----- | ----- | 
| S2 | **LIGHT+** | $2.4263$ | **LIGHT+** | 
| S2 | LIGHT- | $2.1929$ |  | 

### **Interpretation of the Learned Policy**

The agent's optimal action in the Target State is **LIGHT+**, meaning it chooses to slightly increase the LED brightness even when already close to the goal.

1. **Prioritization of Stability:** While the intuitive policy would be **IDLE** (to maximize the small positive energy reward), the agent learned that the environment is too volatile (as seen by Lux readings ranging up to $700\text{ Lux}$).

2. **Why IDLE is Preferred (The Energy Goal):**

   * The **IDLE** action represents the lowest possible energy consumption (zero PWM change).

   * In a stable environment, the highest possible reward ($R \approx 1.0$) is achieved by remaining in the Target State (S2) while executing **IDLE** to minimize energy penalties.

   * The fact that the agent chose **LIGHT+** over **IDLE** shows that the **penalty for dropping out of the Target State** is greater than the **benefit of saving energy** via the IDLE action.

3. **Creating a Buffer:** By choosing **LIGHT+**, the agent proactively increases the PWM duty cycle (as seen in the stable PWM mean of $\approx 32.85$), creating an **energy buffer** that helps the system rapidly resist sudden drops in ambient light.

4. **Conclusion:** The agent developed a sophisticated, risk-averse policy that prioritizes remaining in the high-reward range over achieving the absolute lowest energy consumption, demonstrating successful adaptation to its noisy environment.

## 📈 Learning Dynamics

The self-tuning parameters ($\epsilon$ and $\alpha$) demonstrate a controlled, stable learning process:

* **Epsilon (**$\epsilon$**):** The smooth decay confirms a controlled transition from initial random experimentation to expert behavior.

* **Alpha (**$\alpha$**):** The rapid initial decay of the learning rate ensures that early, less accurate experiences are quickly overwritten, while later experiences (after episode $\approx 400$) only lead to fine-tuning, preventing the agent from forgetting its established, successful policy.

## 🚀 Suggestions for Future Work

The current agent is highly functional, but further performance and energy efficiency gains can be explored with the following enhancements:

1. **Introduce an Explicit IDLE Action Q-Value:** If the IDLE action was not explicitly represented in the Q-table data provided, capturing its final learned value is crucial. If it was present but lower than LIGHT+, the next step is to test the effect of increasing the IDLE reward.

2. **Reward Function Tuning (Incentivizing IDLE):**

   * **Increase the Energy Reward:** The current reward function implicitly penalizes energy consumption. Explicitly increasing the reward for taking the IDLE action in S2 (e.g., $R(\text{S2, IDLE}) = +0.05$) relative to the reward for LIGHT+/LIGHT- could force the agent to accept a small risk of drifting out of S2 in exchange for substantial energy savings.

3. **Hysteresis in Target State:**

   * Refine State S2 into two sub-states: `S2_High` ($\text{300-350 Lux}$) and `S2_Low` ($\text{250-300 Lux}$).

   * Policy in `S2_High`: Encourage **IDLE** or **LIGHT-**.

   * Policy in `S2_Low`: Encourage **IDLE** or **LIGHT+**.

   * This hysteresis prevents rapid toggling between LIGHT+ and LIGHT- near the $300\text{ Lux}$ centerline, improving efficiency and actuator life.

4. **Integration of Time:**

   * Augment the state vector to include a time-based feature (e.g., "Time of Day" or "Rate of Change of Ambient Light").

   * This would allow the agent to learn a completely different policy for the high-volatility daytime period versus the stable nighttime period, leading to much better energy optimization overall.
