# Embedded Reinforcement Learning Framework for Lux and Temperature Control

This project presents a comprehensive **Embedded Reinforcement Learning (RL) Framework** designed to create energy-efficient and highly adaptive **Smart Building Automation Systems (SBAS)**. The framework’s core function is to deploy resource-optimised RL algorithms onto **constrained embedded systems** (for example, low-power microcontrollers) to autonomously manage and optimise settings for key environmental variables: **Illuminance (Lux)** and **Temperature**.

The aim is to move beyond static control systems, which rely on pre-programmed heuristics, by enabling the embedded node to learn optimal operational policies directly from its environment. This dynamic approach significantly reduces energy waste and system tuning time while ensuring robust maintenance of occupant comfort levels.

## Development Progress and Repository Structure

The repository is logically separated into a high-level **training environment** (`simulations`) and multiple, iterative **embedded deployment versions** (`ver1`, `ver2`, `ver3`, `ver4`). This structure clearly maps the progression from conceptual testing to highly optimised hardware implementation.

### 1. `simulations`

This folder represents the **pre-deployment phase**, where the Reinforcement Learning algorithms are rigorously developed and validated against modeled and historical data.

* **Purpose:** To provide a controlled, high-level environment (typically Python) for training robust RL agents before porting the resultant policy or inference code to the embedded C++ environment. The simulation environment acts as a crucial testbed for hyperparameter tuning.

* **Content Focus:**
  * **Environment Model:** Contains a Python implementation of the building physics, accurately simulating thermal and lighting dynamics, occupant behaviour (e.g., presence detection), and actuator responses (e.g., LED dimming curves, HVAC efficiency).
  * **Agent Training Scripts:** Scripts for a variety of RL algorithms (for instance, tabular Q-Learning, SARSA, or scalable Deep Q-Network (DQN) implementations) used to explore and converge on optimal control policies under various seasonal and occupancy scenarios.
  * **Validation Tools:** Includes plotting scripts, extensive logging handlers, and metrics calculations to quantitatively assess agent performance against baseline control systems (such as fixed-point PID controllers or heuristic-based methods), focusing on convergence speed, total energy consumption, and comfort compliance error.

* **Key Outcome:** A proven, stable RL policy (either a Q-table or a set of learned network weights) ready for translation and efficient integration into the embedded C++ environment.

---

## Current Status: Adaptive Q-Learning for Lux Control (The Core Implementation)

The current working code, found in `Adaptive_Q_Learning_Agent_RL_Lighting_Control_Trainer_5_states.ino`, represents the successful completion of the basic single-variable control problem (Lux) and demonstrates the viability of the embedded RL approach.

### Implementation Details:
* **Target Hardware:** ESP32-S3 (or similar low-power MCU).
* **Algorithm:** Tabular Q-Learning.
* **State Space:** Highly discrete 5-state space defined by Lux ranges (e.g., S2 is the $250 - 350\text{ Lux}$ Target State).
* **Action Space (3 Actions):** `LIGHT+` (Increase PWM), `LIGHT-` (Decrease PWM), `IDLE` (Maintain current PWM).
* **Key Feature:** Implements **Adaptive Exploration** ($\epsilon$-Decay) and **Adaptive Learning Rate** ($\alpha$-Decay) to accelerate convergence and stabilize the final policy.

### Performance Summary (Based on `rl_log_data.csv`):
Analysis of the logs confirms that the agent successfully converged on a stable policy, achieving near-optimal rewards ($\approx 0.90$) by effectively balancing Lux target maintenance against energy cost. The adaptive parameters ensured a rapid transition from exploration ($\epsilon = 1.0$) to exploitation ($\epsilon = 0.05$).

---

### 2. `ver1` (Initial Embedded Proof-of-Concept)

This marks the **first successful translation** of the framework's core learning logic into a deployable, resource-constrained environment.

* **Focus:** Establishing the **basic infrastructure** and validating the fundamental feasibility of running a simplified, non-adaptive RL agent (specifically **Q-Learning for Lux control**) directly on the target low-power hardware (MCU). This prioritizes functional execution over efficiency.

* **Content Focus:**
  * **Basic RL Implementation:** Houses the C/C++ code for a simple tabular RL algorithm (Q-Learning) with a highly discretised state–action space, prioritizing computational simplicity and minimizing RAM usage.
  * **Hardware Abstraction Layer (HAL):** Initial, minimal code for interfacing directly with physical sensors (I²C lux sensor) and low-level actuators (PWM dimmer controls).
  * **Minimalist Codebase:** Emphasis on the core RL loop functionality (measurement, reward, update, action), with minimal memory optimisation or power-saving features.

* **Progress Highlight:** Successful execution of the full learning loop on the microcontroller, demonstrating the node’s foundational ability to take real-time environmental measurements, compute temporal difference rewards, and update the Q-table or policy in real time, confirming the viability of the embedded RL approach.

### 3. `ver2` (Optimisation and Algorithm Refinement)

Building upon the functional foundation of `ver1`, this version concentrates on improving computational efficiency, memory footprint, and employing more sophisticated algorithms to handle a wider operating range.

* **Focus:** **Computational and memory optimisation**, specifically addressing the constraints identified in `ver1`. This phase implements the **Adaptive Q-Learning** features (e.g., the adaptive $\epsilon$ and $\alpha$ decay mechanisms seen in the current implementation) to speed up convergence and improve policy stability.

* **Content Focus:**
  * **Advanced RL Agent:** Implementation of the improved algorithm, potentially featuring **function approximation** (such as lightweight, single-layer neural networks, if using a more capable MCU) or a memory-efficient sparse Q-table structure, such as **optimised tile-coding**.
  * **Optimisation Techniques:** Includes low-level memory optimisations (e.g., using fixed-point arithmetic instead of full floating-point operations), efficient data structures (e.g., hash maps for sparse Q-tables), and code profiling to reduce execution time.
  * **Enhanced Measurement Logic:** Refinement of sensor reading and data-processing pipelines (e.g., filtering, sensor fusion) to improve both accuracy and execution speed of the state determination process.

* **Progress Highlight:** Demonstrates significantly **faster convergence** and the ability to manage a **larger, continuous state space** than `ver1`, leading directly to improved control performance, reduced control oscillation, and enhanced energy savings.

### 4. `ver3` (Feature Completion and Robust Deployment)

This iteration integrates all necessary components for a robust field deployment, aligning with typical industrial SBAS requirements for longevity and connectivity. This version specifically focuses on expanding the framework to incorporate the **Temperature control variable**.

* **Focus:** **System robustness, connectivity and advanced power management** — preparing the framework for real-world, long-term operation without manual intervention, and achieving **multi-variable control (Lux + Temp)**.

* **Content Focus:**
  * **Multi-Variable State Space:** Implementation of a composite state space to handle both Lux and Temperature readings and actuator outputs (e.g., HVAC control).
  * **Power Management:** Integration of deep-sleep modes and scheduling logic to minimize power draw. This is critical and includes implementing functionality where a **PIR sensor wakes the node to take environmental measurements**, while the more energy-intensive wireless transmission is intentionally handled by a separate, scheduled wake-up module, ensuring high energy efficiency.
  * **Wireless Communication:** Code for communication modules (for example, Zigbee, Wi-Fi or LoRa) to reliably report system status, receive user-defined setpoints (e.g., "Comfort Mode"), and permit secure over-the-air policy or firmware updates.
  * **Safety & Fallback Mechanisms:** Implementation of essential safety limits (e.g., maximum PWM output) and non-RL fallback control policies (e.g., a simple timer-based control) to ensure system stability during sensor failure or unexpected environmental events.

* **Progress Highlight:** Represents a **highly optimised framework** suitable for industrial prototyping, combining adaptive RL control with essential low-power and networking capabilities.

### 5. `ver4` (Multi-Agent Coordination and Distributed RL)

This final version focuses on scaling the framework to manage complex, multi-zone building environments using distributed intelligence.

* **Focus:** **Decentralized control, inter-agent communication, and handling coupled environmental dynamics** (e.g., one zone's HVAC system affecting another's temperature).

* **Content Focus:**
  * **Multi-Agent RL (MARL) Implementation:** Deployment of coordination strategies (e.g., independent learners, cooperative centralized critic) where multiple embedded nodes (agents) interact and influence a shared environment.
  * **Local Communication Protocol:** Code for high-speed, low-latency inter-node communication (e.g., using protocols like MQTT or custom mesh networks) to share local Q-values, observations, or intentions.
  * **Fault Tolerance and Self-Healing:** Advanced logic to detect the failure of a neighboring agent and compensate for its absence by temporarily adjusting the local control policy.
  * **System-Level Energy Optimisation:** Algorithms focused on global objectives, where individual agent rewards are weighted to minimize total building energy consumption rather than just local zone energy.

* **Progress Highlight:** Demonstrates the framework's scalability, enabling the creation of a truly intelligent, distributed building management system capable of optimizing performance across an entire facility.

## Technology & Licensing

* **Core Languages:** C++ (Embedded), Python (Simulation)
* **Licence:** The project is distributed under the **Apache-2.0 Licence**.




