# 💡 REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU)

## Embedded Reinforcement Learning: Why Being *IDLE* Is So Difficult to Learn

One of the most revealing findings in this project was discovering how
challenging it is for a reinforcement learning (RL) agent---particularly
one running on constrained embedded hardware---to learn that the most
efficient action is sometimes to **do nothing at all**.

In this Adaptive Climate Control Unit (ACCU), the design intention was
clear:

-   **Penalise** every active intervention (LIGHT+, LIGHT--, TEMP+,
    TEMP--)\
-   **Reward** the *IDLE* action whenever the environment was already
    within the comfort zone (State S4)

This should, in theory, encourage the agent to conserve energy by
remaining idle whenever the environment is ideal.

------------------------------------------------------------------------

## 🧠 The Unexpected Outcome

After 200 training episodes, the agent learned to manage temperature and
lighting effectively---yet failed to learn the key behaviour: **energy
efficiency**.

Instead of selecting **IDLE** in the comfort zone (State S4), the agent
consistently selected:

➡️ **LIGHT+**\
❌ Not the intended action.

This demonstrates an important RL principle:

> When the long-term cost of environmental drift outweighs the immediate
> reward of staying idle, the agent will prefer continuous, small
> corrective actions---even if they consume more energy.

This behaviour is common in RL systems deployed in real-world or
resource-constrained environments such as IoT devices and embedded
controllers.

------------------------------------------------------------------------

# 🔧 Recommended Next Steps

To align the learned policy with the desired energy‑efficient behaviour,
several adjustments are recommended:

### **1. Increase the Reward for IDLE**

Make energy-saving behaviour significantly more valuable. - Suggested
increase: **from +0.01 to +0.05** or higher\
- This helps the agent recognise that "doing nothing" is the optimal
choice in stable conditions.

### **2. Increase Penalties for Active Actions**

Reflect real-world energy costs more clearly. - Suggested penalties:
**--0.1 or stronger**\
- Encourages the agent to avoid unnecessary adjustments.

### **3. Extend Training Duration**

200 episodes may not be sufficient for full convergence. - Consider
**500--2000 episodes**

### **4. Refine Environmental Drift Modelling**

Excessive drift pressure forces the agent into constant
micro‑adjustments. - Reduce drift rate\
- Add realistic noise patterns\
- Match sensor characteristics more closely

### **5. Consider Reward Shaping or Potential-Based Methods**

Encourage long-term efficient behaviour, not just per-step efficiency. -
Example: reward for remaining in the comfort zone over time

### **6. Targeted Training for S4**

Train the agent specifically on the comfort zone with a static
environment to lock in the correct IDLE behaviour before full dynamic
training.

------------------------------------------------------------------------

## Project Overview

This project implements a Tabular Q-Learning agent designed to manage
illuminance and temperature within a confined environment. The goal is
to learn an energy-efficient policy that maintains user comfort while
minimising actuator activity.

The solution consists of a **Python simulation** for training and a
corresponding **Embedded C++ sketch** for execution on hardware such as
Arduino or ESP32 boards.

------------------------------------------------------------------------

## Reinforcement Learning Space

### 1. **State Space: 9 Discrete States**

The environment is discretised into a 3×3 grid based on:

-   **BH1750** lux readings\
-   **BMP180** temperature readings

  Environment Factor   Bin 0 (Low)   Bin 1 (Target)   Bin 2 (High)
  -------------------- ------------- ---------------- --------------
  Illuminance (Lux)    ≤100 lx       101--499 lx      ≥500 lx
  Temperature (°C)     ≤18.0 °C      18.1--23.9 °C    ≥24.0 °C

**State 4 (S4)** is the Target Comfort Zone (Medium Lux + Comfort Temp).

------------------------------------------------------------------------

### 2. **Action Space: 5 Discrete Actions**

  Action ID   Action Name   Energy Cost / Reward
  ----------- ------------- ----------------------
  0           LIGHT+        --0.05
  1           LIGHT--       --0.05
  2           TEMP+         --0.05
  3           TEMP--        --0.05
  4           IDLE          +0.01

------------------------------------------------------------------------

## Implementation Details

### Embedded C++ Sketch (`lux_rl_framework.ino`)

The sketch runs the Q-Learning loop, reads sensors, and issues actuator
commands.

It logs: - Exploration events\
- Crucial Q-table updates (TD error ≥ 1.0)\
- Policy changes when the best action for a state shifts

------------------------------------------------------------------------

## Policy Transfer

The trained Q-table from the Python simulation is exported and embedded
directly into the C++ firmware, enabling immediate optimal behaviour
without the need for on-device training.

------------------------------------------------------------------------

## Simulation Results

Training ran for 200 episodes with: - α = 0.1\
- γ = 0.9\
- ε = 0.3

  Metric                       Value
  ---------------------------- -------
  Total Episodes               200
  Crucial Updates (TD ≥ 1.0)   20
  Policy Changes               10

### Final Policy for S4

  State   Description           Learned Best Action
  ------- --------------------- ---------------------
  S4      Target comfort zone   **LIGHT+**

This indicates the need for reward restructuring.

