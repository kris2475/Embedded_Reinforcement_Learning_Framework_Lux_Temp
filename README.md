# Embedded Reinforcement Learning Framework for Lux and Temperature Control

This project presents a comprehensive **Embedded Reinforcement Learning (RL) Framework** designed to create energy-efficient and highly adaptive **Smart Building Automation Systems (SBAS)**. The framework’s core function is to deploy resource-optimised RL algorithms onto **constrained embedded systems** (for example, low-power microcontrollers) to autonomously manage and optimise settings for key environmental variables: **Illuminance (Lux)** and **Temperature**.

The aim is to move beyond static control systems, which rely on pre-programmed heuristics, by enabling the embedded node to learn optimal operational policies directly from its environment. This dynamic approach significantly reduces energy waste and system tuning time while ensuring robust maintenance of occupant comfort levels.

## Development Progress and Repository Structure

The repository is logically separated into a high-level **training environment** (`simulations`) and multiple, iterative **embedded deployment versions** (`ver1`, `ver2`, `ver3`, `ver4`). This structure clearly maps the progression from conceptual testing to highly optimised hardware implementation.

### 1. `simulations`

This folder represents the **pre-deployment phase**, where the Reinforcement Learning algorithms are rigorously developed and validated against modeled and historical data.

* **Purpose:** To provide a controlled, high-level environment (typically Python) for training robust RL agents before porting the resultant policy or inference code to the embedded C++ environment. The simulation environment acts as a crucial testbed for hyperparameter tuning.

* **Content Focus (Future Work):**
    * **Environment Model:** Contains a Python implementation of the building physics, accurately simulating thermal and lighting dynamics.
    * **Agent Training Scripts:** Scripts for a variety of RL algorithms (e.g., Q-Learning, SARSA) to explore and converge on optimal control policies.
    * **Validation Tools:** Includes plotting scripts, logging handlers, and metrics calculations to assess agent performance against baseline control systems.

* **Key Outcome:** A proven, stable RL policy ready for translation and efficient integration into the embedded C++ environment.

---

## Current Implemented Status: Adaptive Q-Learning for Lux Control (`ver2`)

The currently implemented code is the **Adaptive Q-Learning Agent**, successfully addressing the single-variable Lux control problem on constrained hardware. This functionality is entirely contained within the `ver2` scope.

### Implementation Details (Verifiable from Sketch):
* **Target Hardware:** ESP32-S3 (or similar low-power MCU).
* **Algorithm:** Tabular Q-Learning (implemented with the standard update rule: $Q(s_t, a_t) \leftarrow Q(s_t, a_t) + \alpha \left[ r_{t+1} + \gamma \max_{a} Q(s_{t+1}, a) - Q(s_t, a_t) \right]$).
* **State Space:** Highly discrete 5-state space defined by Lux ranges, centered around the target zone.
* **Action Space (3 Actions):** `LIGHT+` (Increase PWM), `LIGHT-` (Decrease PWM), `IDLE` (Maintain current PWM).
* **Key Feature:** Implements **Adaptive Exploration** ($\epsilon$-Decay) and **Adaptive Learning Rate** ($\alpha$-Decay) to accelerate convergence and stabilize the final policy (this is the key difference between `ver1` and `ver2`).

### Performance Summary (Verifiable from Log Data):
Analysis of the provided log data confirms that the agent successfully converged on a stable policy, achieving near-optimal rewards ($\approx 0.90$) by effectively balancing Lux target maintenance against energy cost. The adaptive parameters ensured a rapid transition from exploration ($\epsilon = 1.0$) to exploitation ($\epsilon = 0.05$).

---

### 2. `ver1` (Initial Embedded Proof-of-Concept - Non-Adaptive Baseline)

This version represents the necessary conceptual and implementation baseline **before** adaptive optimization.

* **Focus:** Establishing the basic embedded Q-Learning infrastructure using **fixed** parameters.

* **Content Focus (Conceptual Baseline):**
    * **Basic RL Implementation:** Houses the C/C++ code for a simple tabular RL algorithm (Q-Learning) using a highly discretised state–action space and **fixed, non-decaying** $\epsilon$ and $\alpha$ values.
    * **Hardware Abstraction Layer (HAL):** Minimal code for interfacing directly with physical sensors (Lux sensor) and low-level actuators (PWM dimmer controls).

* **Progress Highlight:** Demonstrating the foundational ability to compute rewards and update the Q-table in real-time, providing the un-optimized comparison for `ver2`.

### 3. `ver2` (Optimisation and Algorithm Refinement - Currently Implemented)

This folder contains the complete, current working code for the embedded RL agent.

* **Focus:** **Computational and memory optimisation** through the implementation of **Adaptive $\epsilon$ and $\alpha$ decay mechanisms**, specifically addressing the slow convergence and instability of the `ver1` baseline.

* **Content Focus (Implemented Sketch):**
    * **Adaptive RL Agent:** Implementation of the **Adaptive Q-Learning** algorithm with dynamic, self-tuning $\epsilon$ and $\alpha$ decay.
    * **Optimisation Techniques:** Focus on efficient table lookups and parameter management suitable for constrained environments.
    * **Enhanced Measurement Logic:** Refinement of sensor reading and data-processing pipelines for state determination.

* **Progress Highlight:** Demonstrates significantly **faster convergence** and a highly stable final policy compared to `ver1`, validating the use of adaptive parameters for embedded systems.

### 4. `ver3` (Future Work: Multi-Variable Control)

* **Focus:** Expansion of the framework to incorporate the **Temperature control variable** alongside Lux control, achieving **multi-variable control (Lux + Temp)**.

### 5. `ver4` (Future Work: Distributed RL and Scaling)

* **Focus:** Scaling the framework to manage complex, multi-zone building environments using distributed intelligence and **Decentralized control, inter-agent communication, and handling coupled environmental dynamics**.

## Technology & Licensing

* **Core Languages:** C++ (Embedded), Python (Simulation)
* **Licence:** The project is distributed under the **Apache-2.0 Licence**.




