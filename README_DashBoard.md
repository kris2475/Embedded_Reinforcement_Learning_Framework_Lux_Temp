# 💡 ESP32 Adaptive Climate Control Unit (ACCU) with Q-Learning

## Project Summary

This project implements a complete **Q-Learning** framework on an **ESP32 microcontroller** to create an Adaptive Climate Control Unit (ACCU). The primary goal is to train an autonomous agent to maintain optimal environmental conditions—specifically **Target Lux (Illuminance)** and **Target Temperature**—while strategically minimizing the energy cost associated with taking actions.

The system features a **real-time, single-page web dashboard** embedded directly on the ESP32, providing crucial visualization of the Q-Table, state changes, and learning dynamics.

---

## 1. System Overview and Core Concept

The system utilizes **Reinforcement Learning (RL)** to solve a classic control problem.

| Component | Role | Details |
| :--- | :--- | :--- |
| **Goal** | Optimal Environment | Maintain **300 Lux** and **21°C**. |
| **Algorithm** | Q-Learning | A model-free RL algorithm used to learn the **optimal policy** (best action per state). |
| **Hardware** | ESP32 | Runs the entire RL algorithm, manages Wi-Fi connectivity, and hosts the web server. |
| **Sensors** | Simulated (BH1750/BMP180) | Provide continuous measurements of Lux and Temperature, defining the environment's state. |
| **Actuators** | Simulated Actions | A small set of predefined actions (e.g., `LIGHT+`, `TEMP-`) used to influence the environment. |

### Q-Learning Cycle (Every 5 Seconds)

The agent operates in a continuous loop, sensing the environment, calculating a reward, updating its knowledge, and executing a new action.

| Step | Detail |
| :--- | :--- |
| **Sense (S)** | Read continuous Lux and Temp. **Discretize** these values into a single **State Index** ($S_0$ to $S_8$). |
| **Reward (R)** | Calculate the reward based on proximity to targets, applying a penalty for the action's simulated **energy cost**. |
| **Learn (Q-Update)** | Update the Q-Table entry for the previous $(S, A)$ pair using the **Temporal Difference (TD) error** equation. |
| **Action (A')** | Select the next action $A'$ using the **Epsilon-Greedy** strategy (mostly exploit, occasionally explore). |
| **Execute** | Apply the chosen action (simulated actuator control). |

### The Q-Learning Equation

The core of the learning process is the Q-Value update equation:

$$Q(S, A) \leftarrow Q(S, A) + \alpha [R + \gamma \max_{A'} Q(S', A') - Q(S, A)]$$

---

## 2. Dashboard and Streaming Fix

The dashboard is a critical visualization tool, delivered as a large, single-file HTML/CSS/JavaScript application embedded in the Arduino sketch's **Flash Memory (`PROGMEM`)**.

### Crucial Technical Fix: Robust HTML Streaming

A significant challenge in hosting large files from an ESP32 is memory allocation. Attempting to buffer the entire HTML file in RAM before sending often leads to **memory allocation failures** or **watchdog timer resets** (the "still blank" issue).

The final sketch implements a robust, low-level solution:

* **`sendLargeHtmlFromProgmem()`:** This custom function directly reads the HTML content from Flash memory.
* **Chunked Sending:** It sends the file in safe, small **512-byte chunks** sequentially. This bypasses the memory-heavy internal buffers of the high-level web server functions, ensuring stability.

> **Expected Behavior:** When a browser connects, the Serial Monitor should confirm the successful transmission with the message: `HTML stream complete. Connection closed.`

---

## 3. Interpreting the Dashboard

The web dashboard is designed to provide immediate, actionable insight into the agent's real-time performance and learning.

### A. Sensor Data & Status

| Element | Interpretation |
| :--- | :--- |
| **Lux / Temp Value** | The current, raw sensor readings (state input). |
| **Gauges** | Visual representation against the target. The center line is the target (300 Lux / 21°C). |
| **Reward Value** | The immediate reward received in the last step. Range: -1.0 to +1.0. Higher values indicate better performance relative to energy cost. |
| **State Index (S0-S8)** | The discrete environmental state the system currently recognizes. |
| **Action Taken** | The command chosen by the RL agent in the last step (e.g., `LIGHT+`, `TEMP-`). |

### B. State Space Mapping (S0 to S8)

The continuous sensor readings are **discretized** into 9 distinct states, formed by combining three bins for Lux and three bins for Temperature.

| State Index (S) | Lux Bin | Temp Bin | Description |
| :---: | :--- | :--- | :--- |
| **S0** | Low (< 100) | Cold (< 18°C) | Environment is too dark and too cold. |
| **S4** | Medium/Target (100-500) | Comfort/Target (18-24°C) | **Ideal State (Target)**. |
| **S8** | High (> 500) | Hot (> 24°C) | Environment is too bright and too hot. |

### C. Q-Table Policy Heatmap (The Core of the Learning)

This visualization reveals the agent's learned knowledge (the Policy).

| Feature | Interpretation |
| :--- | :--- |
| **Rows** | Represent the 9 possible **States** ($S_0$ to $S_8$). |
| **Columns** | Represent the 5 available **Actions** (`LIGHT+`, `LIGHT-`, `TEMP+`, `TEMP-`, `IDLE`). |
| **Cell Value (Color)** | The **Q-Value**. This is the expected cumulative future reward if the agent takes the specific Action (column) while in the specific State (row). |
| **Color Scale** | **Green** = High Positive Q-Value (Action is consistently good). **Red** = High Negative Q-Value (Action is consistently bad). |
| **Blue Star (★)** | Marks the **Greedy Action**—the best-known action for that state, defining the current policy. |
| **Active State** | The entire row corresponding to $S_{current}$ is highlighted with a pulsing blue outline. |

### D. RL Log

The log provides a detailed narrative of the agent's decision-making and learning process.

* `"EXPLORE:"`: The agent deliberately chose a **random action** despite having a known best action (based on the Epsilon-Greedy strategy).
* `"EXPLOIT:"`: The agent chose the action with the **highest known Q-value** (the Greedy action).
* `"LEARN: ... [CRUCIAL UPDATE]"`: Indicates a large **TD Error**, meaning the observed reward was very surprising and led to a substantial update in the Q-Table.
* `"LEARN: ... [POLICY CHANGE: XXX]"`: The most significant event. This means the update was large enough to change the **Greedy Action (★)** for a state, indicating the agent has truly learned a new, better policy.
