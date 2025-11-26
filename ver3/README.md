# Embedded Reinforcement Learning Framework (RALU) - V3.0

## 💡 Project Overview: The RALU Agent

The **Real-time Adaptive Lighting Unit (RALU)** is an embedded Reinforcement Learning (RL) project **utilising** an **ESP32-S3** microcontroller to implement a self-tuning **Q-Learning Agent**.

The agent's objective is to continuously learn and **optimise** the **LED PWM duty cycle** (the actuator) to maintain a specific ambient light level (**TARGET\_LUX: 300.0 Lux**) **whilst** minimising energy usage.

This framework is designed to provide real-time, explainable, and autonomous control, adapting immediately to changes in the environment (e.g., changes in natural light).

***

## 🚀 What's New in V3.0: Tighter Control & Real-Time Visualisation

This version represents a major stability and **optimisation** update, focusing on achieving tighter control around the target and introducing a critical debugging tool.

| Feature | Description | Benefit |
| :--- | :--- | :--- |
| **5-Bin State Space** | Expanded the state space from 3 to **5 distinct bins** (Very Low, Low, Target, High, Very High). The Target state is now a narrow band of **250-350 Lux**. | Forces the agent to learn finer distinctions, leading to **tighter, more accurate control** around the 300 Lux target. |
| **High-Sensitivity Reward** | The reward function constant `REWARD_MAX_DISTANCE` was lowered to **1.0f**. | Harshly penalises any Lux deviation outside the target band, **accelerating convergence** towards the desired 300 Lux. |
| **Real-Time Learning Graph** | Integrated **Chart.js** into the web dashboard to plot the **Moving Average Reward** in real-time. | Provides immediate, visual feedback on the learning process, allowing the user to confirm the agent is **converging and stable** (a reward line near 1.0). |
| **Faster Episode Time** | The RL episode cycle remains at **500ms** for maximum responsiveness. | Ensures the agent can react quickly to sudden changes in ambient light conditions. |

***

## 🧠 RL Agent Core: Q-Learning Implementation

The RALU agent uses a **Model-Free Q-Learning** algorithm. The core components are:

### 1. State Space ($S$)

The environment is **discretised** into **5 Lux-based states** to ensure high-resolution control:

| State Index | State Name | Lux Range (BH1750) |
| :--- | :--- | :--- |
| **S0** | Very Low | `< 100 Lux` |
| **S1** | Low | `100 - 250 Lux` |
| **S2** | **Target** | `250 - 350 Lux` |
| **S3** | High | `350 - 500 Lux` |
| **S4** | Very High | `> 500 Lux` |

### 2. Action Space ($A$)

| Action Index | Action Name | PWM Change |
| :--- | :--- | :--- |
| **A0** | LIGHT+ | `+10` to PWM Duty Cycle |
| **A1** | LIGHT- | `-10` to PWM Duty Cycle |
| **A2** | IDLE | `No Change` (Saves Energy) |

### 3. Q-Value Update Rule

The agent updates its policy by calculating the Temporal Difference (TD) error:

$$Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \left[ r_{t+1} + \gamma \max_{a} Q(s_{t+1}, a) - Q(s_t, a_t) \right]$$

***

## ⚙️ Self-Tuning Mechanism

To ensure both rapid initial learning and stable long-term control, the agent employs two adaptive parameters:

### 1. Adaptive Exploration ($\epsilon$-Greedy Policy)

The $\epsilon$ (Epsilon) value starts at $\epsilon=1.0$ (100% random exploration) and decays multiplicatively after every episode by $\epsilon_{\text{decay}}=0.999$, until it reaches a minimum of $\epsilon_{\text{end}}=0.05$.

This ensures the agent:
1.  **Explores** the environment thoroughly at the start to fill the Q-table.
2.  **Exploits** the learned policy (takes the best action) most of the time after convergence, only taking a random action 5% of the time.

### 2. Dynamic Learning Rate ($\alpha$)

The learning rate ($\alpha$) is calculated dynamically using the visitation count **$N(s, a)$** for each state-action pair, stored in the `N_table`.

$$\alpha_{\text{dynamic}} \leftarrow \frac{\text{ALPHA\_START}}{\text{ALPHA\_START} + N(s, a)}$$

* **Low $N$ (New States):** $\alpha$ remains high, allowing fast updates and quick adaptation to new conditions.
* **High $N$ (Learned States):** $\alpha$ drops near zero, making the Q-values highly stable and preventing overshooting due to noise.

***

## 📈 Performance Analysis of the RALU Agent (Post-Convergence)

The provided serial log data (Episodes 6517 to 6546) confirms that the RALU agent is operating in a **highly converged and stable state**, demonstrating the effectiveness of the V3.0 self-tuning and state-space refinements.

### 1. High Stability and Optimal Policy

The agent has successfully completed its primary exploration phase and transitioned into the exploitation phase, as evidenced by:

* **Policy:** The agent consistently chooses the **`IDLE`** action in the **S2 (Target)** state. This is the optimal, energy-efficient policy, as it maintains the target Lux without unnecessary PWM changes.
* **Lux Stability:** The Lux reading is extremely stable, varying only between **274.17 Lux and 277.50 Lux**. **Whilst** slightly below the 300 Lux target, this narrow variance confirms **tight control** well within the required S2 state band (250-350 Lux).
* **Reward:** The reward is consistently high, ranging from **0.752 to 0.785**. The reward is high but not 1.0 (perfect) because the agent is slightly below the 300 Lux **centre**, demonstrating the high sensitivity of the reward function.

### 2. Self-Tuning Confirmation

The log data confirms the automatic parameter adjustments have **stabilised** the learning process:

* **Epsilon ($\epsilon$):** Fixed at **0.050**. The agent is in **high-exploitation mode** (95% reliance on the Q-table).
* **Alpha ($\alpha$):** Fixed at **0.000** (or near zero). This indicates the visitation count **$N(s, a)$** is very high, causing the dynamic learning rate to drop. The Q-values are effectively **frozen and stable**, preventing noise from **destabilising** the learned policy.

### 3. Policy Correction (Episode 6530)

A small sequence demonstrates the agent's ability to correct deviations:

* After a momentary excursion (likely an $\epsilon$-Greedy exploration or a sensor fluctuation) resulted in **337.50 Lux** (still S2, but high) at the start of Episode 6530, the agent immediately responded with a **`LIGHT-`** action. This correction successfully brought the PWM back to the stable value of **40**, demonstrating fast and effective corrective control.

***

## 💻 Hardware and Setup

### Requirements

* **Microcontroller:** ESP32-S3 (or compatible ESP32 module with I2C).
* **Sensor:** BH1750 I2C Digital Light Sensor (Lux).
* **Actuator:** LED connected to a PWM pin (e.g., GPIO 8 / D9).
* **Libraries:** Standard ESP32 WiFi and Wire libraries.

### Pin Configuration

| Component | Pin (ESP32-S3) | Notes |
| :--- | :--- | :--- |
| **BH1750 SDA** | `GPIO 43` | I2C Data Pin |
| **BH1750 SCL** | `GPIO 44` | I2C Clock Pin |
| **LED PWM** | `GPIO 8` | Configured for `LEDC_TIMER_8_BIT` (0-255 range) |

***

## 📊 Real-Time Dashboard & Visualisation

The agent hosts a responsive web dashboard on the ESP32's local IP address. The dashboard provides comprehensive monitoring, including the newly added **Learning Progress Graph**.

### 1. Learning Progress Graph (New!)

* **Type:** Line Chart using Chart.js.
* **Y-Axis:** **Moving Average Reward** (tracks the average reward over the last 100 episodes).
* **X-Axis:** **Episode Count**.
* **Interpretation:** A stable line nearing **1.0** indicates that the agent has successfully converged on a high-performing, optimal control policy.

### 2. Q-Table Heatmap

* The Q-Table is rendered as a **colour**-coded heatmap, showing the estimated expected future reward for taking any action in any state.
* **Green:** High Q-Value (Good action).
* **Red:** Low Q-Value (Bad action).
* A **★** marks the agent's current best action (the learned policy) for each state.

### 3. Live Metrics

The dashboard constantly updates the following metrics every 500ms:

* **Illuminance (Lux):** Current reading from the BH1750 sensor.
* **LED PWM Output:** The current duty cycle (0-255).
* **Reward:** The immediate reward ($r_{t+1}$) for the last action taken.
* **$\epsilon$ (Epsilon):** The current exploration rate.
* **$\alpha$ (Alpha):** The last calculated dynamic learning rate.
* **RL Log:** A message detailing the last action (EXPLORE or EXPLOIT) and the result of the `updateQTable` function.
