# 💡 REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU)

## Project Overview

This project implements a Tabular Q-Learning agent designed to manage
the climate (Illuminance and Temperature) within a confined space. The
primary objective is to learn an energy-efficient policy that maintains
user comfort while minimizing actuator activity.

The solution is divided into a **Python Simulation** for rapid training
and policy development, and an **Embedded C++ Sketch** for real-time
execution of the learned policy on constrained hardware (e.g.,
Arduino/ESP32).

------------------------------------------------------------------------

## 🧠 Reinforcement Learning Space

The State and Action spaces are strictly defined and mirrored between
the simulation and embedded code to ensure policy portability.

------------------------------------------------------------------------

### 1. **State Space (𝑆): 9 Discrete States**

The environment is discretized into a 3×3 grid based on sensor readings
from a **BH1750 (Lux)** and a **BMP180 (Temperature)**.

  ------------------------------------------------------------------------
  Environment       Bin **0** (Low)  Bin **1** (Target)  Bin **2** (High)
  Factor                                                 
  ----------------- ---------------- ------------------- -----------------
  Illuminance (Lux) ≤ 100 lx         101--499 lx         ≥ 500 lx

  Temperature (°C)  ≤ 18.0 °C        18.1--23.9 °C       ≥ 24.0 °C
  ------------------------------------------------------------------------

The **Target Comfort Zone** is **State 4 (S4)**: Medium Lux (Bin 1) and
Comfort Temp (Bin 1).

------------------------------------------------------------------------

### 2. **Action Space (𝐴): 5 Discrete Actions**

Actions correspond directly to actuator controls, with an explicit
**energy cost** integrated into the reward function.

  Action ID   Action Name   Energy Cost / Reward
  ----------- ------------- ----------------------------
  0           LIGHT+        Negative Penalty (-0.05)
  1           LIGHT-        Negative Penalty (-0.05)
  2           TEMP+         Negative Penalty (-0.05)
  3           TEMP-         Negative Penalty (-0.05)
  4           IDLE          Small Reward Bonus (+0.01)

------------------------------------------------------------------------

## 🛠️ Implementation Details

### Embedded C++ Sketch (`lux_rl_framework.ino`)

The sketch runs the Q-Learning loop, interacts with sensors (BH1750,
BMP180), and outputs actuator commands via a serial proxy.

#### Crucial Moment Logging

To support debugging on constrained hardware, the sketch logs:

-   **Exploration**: Random actions taken under ε-Greedy\
-   **Crucial Updates**: TD Error ≥ 1.0\
-   **Policy Change**: Best action for a state changes

------------------------------------------------------------------------

## 🚀 Policy Transfer

The trained Q-Table from the Python simulation is **exported and
hardcoded** into the C++ sketch.\
This allows the embedded system to immediately execute an optimized
policy **without on-chip training**.

------------------------------------------------------------------------

## 📊 Simulation Results Summary

Simulation ran for **200 episodes** with:

-   α = 0.1\
-   γ = 0.9\
-   ε = 0.3

### Key Findings

  Metric                       Value
  ---------------------------- -------
  Total Episodes               200
  Crucial Updates (TD ≥ 1.0)   20
  Policy Changes               10

------------------------------------------------------------------------

### Final Policy Analysis (State S4)

Goal: In the target comfort zone (S4), the agent should choose **IDLE**
to save energy.

  State ID   Description                 Best Action (Learned)
  ---------- --------------------------- -----------------------
  S4         Medium Lux + Comfort Temp   **LIGHT+**

### ❌ Conclusion: Energy Efficiency Failure

The agent still chooses **LIGHT+** instead of **IDLE**.\
The reward bonus for IDLE (**+0.01**) was not strong enough to outweigh
environmental drift effects.

------------------------------------------------------------------------

## 🔧 Proposed Next Steps

-   Increase IDLE reward bonus (e.g., **+0.05** or more)
-   Increase penalties for active actions
-   Run more episodes for better convergence
-   Re-evaluate environmental drift model to match real-world conditions
-   

Embedded Reinforcement Learning: Why Being "IDLE" Is So Hard to Learn! 💡

Just finished a deep dive into an embedded Reinforcement Learning project: building an Adaptive Climate Control Unit (ACCU) using Q-Learning on constrained hardware. The goal? Optimize user comfort (Lux and Temp) while relentlessly minimizing energy consumption.

The core challenge was translating this priority into a learning signal. We heavily penalized any active action (LIGHT+, TEMP+) and gave a small bonus to the IDLE (do nothing) action when the system was already in the comfort zone (State S4).

🧠 The Unexpected Result

After 200 simulation episodes, the agent learned to control everything perfectly except the most important part: energy saving.

For the optimal target state (S4: Comfort Zone), the learned policy was LIGHT+, not IDLE! The agent preferred to actively spend energy rather than sit still.

This highlights a crucial principle in #ReinforcementLearning:

The long-term cost of environmental drift, combined with the low bonus for saving energy, often outweighs the immediate reward of doing nothing. The agent learns that continuous, small interventions are safer than waiting for a potentially large, costly drift outside the target zone.

🛠️ Next Steps

This is where the engineering and data science collide. We failed to fully converge on an energy-efficient policy, necessitating a tuning phase:

Increase IDLE Reward: Make the bonus for saving energy more significant.

Increase Active Penalty: Make the cost of running actuators steeper.

It’s a powerful lesson in how reward function design dictates behavior, especially in resource-constrained environments like #IoT and #EmbeddedSystems.

Have you faced similar policy convergence issues where the "safe" action was unexpectedly the energy-consuming one? Let's discuss!

#Qlearning #AI #EmbeddedAI #EnergyEfficiency #IoTDevelopment #DataScience #MachineLearning #Tech
