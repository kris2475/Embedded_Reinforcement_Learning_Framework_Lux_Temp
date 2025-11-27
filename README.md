# Embedded Reinforcement Learning Framework for Lux and Temperature Control

This project presents a comprehensive Embedded Reinforcement Learning (RL) Framework designed to create energy-efficient and highly adaptive Smart Building Automation Systems (SBAS). The framework’s core function is to deploy resource-optimised RL algorithms onto constrained embedded systems (for example, low-power microcontrollers) to autonomously manage and optimise settings for key environmental variables: Illuminance (Lux) and Temperature.

The aim is to move beyond static control systems, which rely on pre-programmed heuristics, by enabling the embedded node to learn optimal operational policies directly from its environment. This dynamic approach significantly reduces energy waste and system tuning time while ensuring robust maintenance of occupant comfort levels.

---

## Development Progress and Repository Structure

The repository is logically separated into a high-level training environment (simulations) and multiple, iterative embedded deployment versions (ver1, ver2, ver3, ver4). This structure clearly maps the progression from conceptual testing to highly optimised hardware implementation.

---

## 1. `simulations`

This folder represents the pre-deployment phase, where the Reinforcement Learning algorithms are rigorously developed and validated against modeled and historical data.

**Purpose:** To provide a controlled, high-level environment (typically Python) for training robust RL agents before porting the resultant policy or inference code to the embedded C++ environment. The simulation environment acts as a crucial testbed for hyperparameter tuning.

### Content Focus:

- **Environment Model:** Contains a Python implementation of the building physics, accurately simulating thermal and lighting dynamics, occupant behaviour (e.g., presence detection), and actuator responses (e.g., LED dimming curves, HVAC efficiency).

- **Agent Training Scripts:** Scripts for a variety of RL algorithms (for instance, tabular Q-Learning, SARSA, or scalable Deep Q-Network (DQN) implementations) used to explore and converge on optimal control policies under various seasonal and occupancy scenarios.

- **Validation Tools:** Includes plotting scripts, extensive logging handlers, and metrics calculations to quantitatively assess agent performance against baseline control systems (such as fixed-point PID controllers or heuristic-based methods), focusing on convergence speed, total energy consumption, and comfort compliance error.

**Key Outcome:**  
A proven, stable RL policy (either a Q-table or a set of learned network weights) ready for translation and efficient integration into the embedded C++ environment.

---

## 2. `ver1` — Initial Embedded Proof-of-Concept

This marks the first successful translation of the framework's core learning logic into a deployable, resource-constrained environment, addressing initial memory and processing limitations.

**Focus:**  
Establishing the basic infrastructure and validating the fundamental feasibility of running a simplified RL agent directly on the target low-power hardware (MCU). This prioritizes functional execution over efficiency.

### Content Focus:

- **Basic RL Implementation:** C/C++ code for a simple tabular RL algorithm (such as Q-Learning) with a highly discretised state–action space (e.g., 5 states, 3 actions), prioritizing computational simplicity and minimizing RAM usage.

- **Hardware Abstraction Layer (HAL):** Initial, minimal code for interfacing with physical sensors (e.g., I²C lux/temperature sensors) and low-level actuators (e.g., PWM dimmers, simple relay controls).

- **Minimalist Codebase:** Emphasis on the core RL loop functionality (measurement, reward, update, action), with minimal memory optimisation or power-saving features.

**Progress Highlight:**  
Successful execution of the full learning loop on the microcontroller, demonstrating the node’s foundational ability to take real-time environmental measurements, compute temporal difference rewards, and update the Q-table or policy in real time.

---

## 3. `ver2` — Optimisation and Algorithm Refinement

Building upon the functional foundation of ver1, this version concentrates on improving computational efficiency, memory footprint, and employing more sophisticated algorithms to handle a wider operating range.

**Focus:**  
Computational and memory optimisation, specifically addressing the constraints identified in ver1 (e.g., slow Q-table lookups, high RAM usage for large tables). This phase explores more advanced RL techniques for low-power hardware.

### Content Focus:

- **Advanced RL Agent:** Implementation of improved algorithms, potentially including function approximation (e.g., lightweight neural networks) or sparse Q-table structures such as tile-coding.

- **Optimisation Techniques:** Low-level memory optimisations (e.g., fixed-point arithmetic), efficient data structures (e.g., hash maps for sparse Q-tables), and code profiling to reduce execution time.

- **Enhanced Measurement Logic:** Refinement of sensor-reading and data-processing pipelines to improve state estimation accuracy and execution speed.

**Progress Highlight:**  
Demonstrates significantly faster convergence and the ability to manage a larger, continuous state space, enabling improved control performance and energy savings.

---

## 4. `ver3` — Feature Completion and Robust Deployment

This iteration integrates all necessary components for a robust field deployment, aligning with typical industrial SBAS requirements for longevity and connectivity.

**Focus:**  
System robustness, connectivity, and advanced power management — preparing the framework for long-term, real-world operation.

### Content Focus:

- **Power Management:** Integration of deep-sleep modes and scheduling logic to minimize power draw. Includes PIR-based wake-up logic and deferring wireless transmissions to scheduled intervals for efficiency.

- **Wireless Communication:** Code for Zigbee, Wi-Fi, or LoRa connectivity to report system status, receive user-defined setpoints, and permit secure OTA updates.

- **Safety & Fallback Mechanisms:** Implementation of safety limits (e.g., maximum PWM levels) and non-RL fallback control policies for handling faults or unexpected conditions.

**Progress Highlight:**  
A highly optimised framework suitable for industrial prototyping, combining adaptive RL control with essential low-power and networking capabilities.

---

## 5. `ver4` — Multi-Agent Coordination and Distributed RL

This version focuses on scaling the framework for multi-zone building environments using distributed intelligence.

**Focus:**  
Decentralized control, inter-agent communication, and handling coupled environmental dynamics.

### Content Focus:

- **Multi-Agent RL:** Deployment of MARL strategies (e.g., independent learners, cooperative critics) where multiple embedded nodes influence a shared environment.

- **Local Communication Protocol:** High-speed, low-latency inter-node communication (e.g., MQTT or custom mesh networks) for sharing Q-values, observations, or intentions.

- **Fault Tolerance and Self-Healing:** Logic to detect neighbouring node failure and adapt local policy accordingly.

- **System-Level Energy Optimisation:** Algorithms that weight rewards to minimize *global* building energy use, not just local energy consumption.

**Progress Highlight:**  
Shows framework scalability, enabling an intelligent, distributed building management system that optimises performance across entire facilities.

---

## Technology & Licensing

- **Core Languages:** C++ (Embedded), Python (Simulation)  
- **Licence:** Apache-2.0

```markdown




