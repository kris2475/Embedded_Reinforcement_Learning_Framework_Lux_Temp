# 💡 REINFORCEMENT LEARNING ADAPTIVE CLIMATE CONTROL UNIT (ACCU) Q-LEARNING TRAINER

An embedded framework for training a **Tabular Q-Learning** agent to manage the climate (Illumination and Temperature) within a confined space, prioritizing **energy efficiency** and user comfort.

---

## 🚀 Overview

This Arduino/ESP32 sketch implements a core **Q-Learning** algorithm designed for resource-constrained **Embedded Reinforcement Learning (ERL)** systems. The agent learns an energy-efficient policy to maintain target comfort conditions (Lux and Temperature) by penalizing active control actions (`LIGHT+`, `TEMP+`, etc.) and rewarding the `IDLE` (no energy) action when comfort is met.

The framework features **Crucial Moment Logging**, which helps track and debug the learning process by reporting only the most significant events:

1.  **Exploration:** Random actions taken under the $\epsilon$-Greedy strategy.
2.  **Crucial Update:** Large updates to the Q-Table when the **Temporal Difference (TD) Error** magnitude exceeds a threshold ($\ge 1.0$), indicating a surprising or highly rewarding outcome.
3.  **Policy Change:** When an update causes the "best" action for a given state to switch.

## 🛠️ Hardware & Prerequisites

This sketch is written for typical embedded microcontrollers (tested on **ESP32/Arduino** with I2C capabilities).

| Component | Function | I2C Address | Notes |
| :--- | :--- | :--- | :--- |
| **Microcontroller** | ESP32/ESP8266/Arduino | N/A | Must support `Wire.h` (I2C) |
| **Illuminance Sensor** | **BH1750 (GY-30)** | `0x23` | Measures **Lux** |
| **Temperature Sensor** | **BMP180** | `0x77` | Measures **Temperature ($\text{}^\circ\text{C}$)** |
| **Actuators** | Serial/Console Output | N/A | **Simulated** (No physical relays or PWM implemented) |

### Pin Configuration
* $\text{I2C}$ **SDA**: Pin `43`
* $\text{I2C}$ **SCL**: Pin `44`

## 🧠 Reinforcement Learning Space

The Q-Learning algorithm operates over a small, defined state and action space to ensure fast computation and low memory footprint.

### 1. State Space ($\mathbf{S}$): 9 Discrete States

The environment is discretized into $\mathbf{3}$ **Lux Bins** $\times$ $\mathbf{3}$ **Temp Bins**, totaling **9 states** (S0 to S8).

| Environment Factor | Bin $\mathbf{0}$ | Bin $\mathbf{1}$ | Bin $\mathbf{2}$ |
| :--- | :--- | :--- | :--- |
| **Illuminance (Lux)** | **LOW** ($\le 100 \text{ lx}$) | **MEDIUM** ($101 - 499 \text{ lx}$) | **HIGH** ($\ge 500 \text{ lx}$) |
| **Temperature ($\text{}^\circ\text{C}$)** | **COLD** ($\le 18.0 \text{}^\circ\text{C}$) | **COMFORT** ($18.1 - 23.9 \text{}^\circ\text{C}$) | **HOT** ($\ge 24.0 \text{}^\circ\text{C}$) |

### 2. Action Space ($\mathbf{A}$): 5 Discrete Actions

The agent can select one of five actions to control the simulated actuators.

| Action ID | Action Name | ACCU Control Interpretation | Energy Cost |
| :---: | :--- | :--- | :--- |
| 0 | **LIGHT+** | Increase Illumination | **Negative Reward Penalty** |
| 1 | **LIGHT-** | Decrease Illumination | **Negative Reward Penalty** |
| 2 | **TEMP+** | Activate Heater/Fan (Raise Temp) | **Negative Reward Penalty** |
| 3 | **TEMP-** | Activate Cooler/Fan (Lower Temp) | **Negative Reward Penalty** |
| 4 | **IDLE** | No Control Action / Zero Energy | **Small Reward Bonus** |

### 3. Q-Learning Hyperparameters

| Parameter | Symbol | Value | Description |
| :--- | :---: | :---: | :--- |
| Learning Rate | $\alpha$ | `0.1` | Moderate rate; prioritizes new experience. |
| Discount Factor | $\gamma$ | `0.9` | High value; values future rewards heavily. |
| Exploration Rate | $\epsilon$ | `0.3` | High rate; ensures active exploration over exploitation. |
| TD Error Threshold | $\text{N/A}$ | `1.0` | Used for logging **Crucial Updates**. |

---

## 📘 Q-Learning Algorithm

The learning process is driven by the **Bellman Equation** for Q-Learning, applied iteratively in the `loop()` function.

$$
Q(S, A) \leftarrow Q(S, A) + \alpha \times [R + \gamma \times \max_{A'} Q(S', A') - Q(S, A)]
$$

### Reward Function $\mathbf{R}$ (Energy-Centric Policy)

The immediate reward $R$ is calculated based on comfort deviation and an energy cost/bonus:

$$
R = R_{\text{Comfort}} + R_{\text{Energy\_Cost}}
$$

* **$R_{\text{Comfort}}$**: Inversely proportional to the $\text{L}2$-norm distance from the **Target Comfort Zone** ($\text{Target\_Lux} = 300.0\text{ lx}$, $\text{Target\_Temp} = 21.0\text{}^\circ\text{C}$).
* **$R_{\text{Energy\_Cost}}$**:
    * Active actions (`LIGHT+`, `TEMP-`, etc.): **-0.05** penalty.
    * `IDLE` action: **+0.01** bonus.

This structure forces the agent to learn that the long-term optimal action is **IDLE** when the environment is in the comfort zone, effectively achieving **energy conservation**.

### Graceful Sensor Failure

The system implements a **failover policy**: if the BH1750 Lux sensor returns an invalid reading ($< 0$), the system automatically uses the `TARGET_LUX` value ($\mathbf{300.0 \text{ lx}}$) for that iteration's state calculation. This prevents state errors and allows training to continue without interruption.

---

## 📊 Serial Output Monitoring

The serial output is verbose, designed to trace the agent's decision-making and learning in real-time.

* **`[EPISODE]`**: Reports the current sensor readings and the derived **State ($\mathbf{S'}$)**.
* **`[REWARD]`**: Reports the immediate reward **($\mathbf{R}$)** received.
* **`[RL-LOG] EXPLORE/EXPLOIT`**: Reports the chosen action **($\mathbf{A'}$)** and strategy.
* **`[CRUCIAL UPDATE]`**: Alerts when a large $\text{TD}$ error drives a significant Q-value change.
* **`[POLICY CHANGE]`**: Alerts when the best action for a state changes.
* **Q-TABLE**: Printed periodically (`~10s`) showing the current learned policy (best action for each state is marked with `*`).
