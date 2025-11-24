# 💡 REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU)

## ❗ Embedded Reinforcement Learning: Why Being **IDLE** Is So Hard to Learn!

One of the most important discoveries in this project is how
**surprisingly difficult** it is for a reinforcement learning
agent---especially on embedded hardware---to learn that sometimes the
best action is to **do nothing**.

In this ACCU system, the goal was energy efficiency:\
- Penalize all active actions (LIGHT+, TEMP+)\
- Reward the IDLE action when in the comfort zone (State S4)

### 🧠 The Unexpected Result

After 200 episodes of training, the agent mastered climate
control---**except for the most important part: energy saving.**

In the optimal comfort zone (**State S4**), the learned policy was:

👉 **LIGHT+**\
❌ Not the desired\
👉 **IDLE**

This is a powerful reinforcement learning insight:

> The long-term cost of environmental drift, combined with too small of
> a reward for saving energy, makes the agent prefer constant
> micro‑adjustments over sitting still---even though IDLE should be
> "best" in theory.

This happens frequently in resource‑limited environments like IoT and
embedded systems.

### 🛠️ Next Steps to Fix the Policy

To converge on true energy‑efficient behavior:

-   **Increase IDLE reward** (e.g., +0.05 or more)\
-   **Increase penalties** for actuator actions\
-   Re-run training with more episodes\
-   Improve the environmental drift model

If you've struggled with agents preferring "safe but costly" actions
over efficient ones---this is exactly that phenomenon.

------------------------------------------------------------------------

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

The sketch logs for debugging:

-   **Exploration** under ε-Greedy\
-   **Crucial Updates** (TD Error ≥ 1.0)\
-   **Policy Changes** when best action changes

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

The reward for IDLE (+0.01) wasn't strong enough to overcome long-term
drift and risk avoidance.\
Thus, the learned behavior violates the intended energy-saving design.

------------------------------------------------------------------------

## 🔧 Proposed Next Steps

-   Increase IDLE reward bonus (e.g., **+0.05** or more)\
-   Increase penalties for active actions\
-   Run more episodes for better convergence\
-   Improve environment drift model to match real conditions

------------------------------------------------------------------------

#Qlearning #AI #EmbeddedAI #EnergyEfficiency #IoTDevelopment
#DataScience #MachineLearning #Tech
